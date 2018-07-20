# coding=utf-8

# Bled112 class to dongle interface
from threading import Event, RLock, Thread
from .packet_serial import PacketSerial, PacketReceiver
from .packets import *
from .waitables import *
from functools import partial
import re
import collections
from enum import Enum
import logging
from .worker_thread import WorkerThread


def bytes_to_hexstr(data, separator=' '):
	return separator.join([format(ord(x), '02x') for x in data])


def convert_str_address_to_tuple(str_addr):
	m = re.search(r'^(?P<b0>[A-F0-9]{2}):(?P<b1>[A-F0-9]{2}):(?P<b2>[A-F0-9]{2}):(?P<b3>[A-F0-9]{2}):(?P<b4>[A-F0-9]{2}):(?P<b5>[A-F0-9]{2})$', str_addr, re.IGNORECASE)
	if m is None:
		return None
	return tuple(int(x, 16) for x in m.groups())


class Bled112(WorkerThread):
	
	# Costants
	CLASS_ABBR = 'Bled'
	FIRST_ATTRIBUTE_HANDLE = 0x0001
	LAST_ATTRIBUTE_HANDLE = 0xffff

	# Costructors
	def __init__(self, packet_serial):
		super(Bled112, self).__init__()
		
		logging.info(self.__class__.CLASS_ABBR + ': init')
		
		# Lock for public methods
		self.__lock = RLock()
		
		# Object for event stop
		self.__ie_stop = WaitableEvent('stop')
		self.__stop = False
		
		# Instantiate packet serial
		self.__ps = packet_serial
		self.__wq_conn_disconnect = WaitableQueue('conn disconnect')
		
		# Connection info
		self.__connection_handle = None
		self.__device_address = None
		
		# Discovery state
		self.__discovery_enabled = False

	
	def _main(self):
		logging.info(self.__class__.CLASS_ABBR + ' > main')

		while not self.__stop:
			try:
				r, _, _ = select([self.__wq_conn_disconnect, self.__ie_stop], [], [])
				logging.debug(self.__class__.CLASS_ABBR + ': select ' + str(r))
				if self.__wq_conn_disconnect in r:
					with self.__lock:
						self.__connection_handle = None
						self.__device_address = None
				if self.__ie_stop in r:
					self.__stop = True
			except KeyboardInterrupt as e:
				break
		
		logging.info(self.__class__.CLASS_ABBR + ' < main')

	def _command(self, packet, filter=None, ender=lambda pkt: True):
		with self.__ps.receiver([(packet.index, filter)], ender) as pr:
			self.__ps.send(packet)
			ans = next(pr)
			return ans
	
	def _command_connection(self, packet, filter=None, conn_fieldname='connection', ender=lambda pkt: True):
		def ff(ch, fil, pkt):
			if not eval('pkt.{} == ch'.format(conn_fieldname), globals(), {'pkt': pkt, 'ch': ch}):
				return False
			if fil is not None and not fil(pkt):
				return False
			return True
		with self.__lock:
			ffp = partial(ff, self.__connection_handle, filter)
		return self._command(packet, ffp, ender)
	
	# Proprierties
	def _connected(self):
		with self.__lock:
			return self.__connection_handle is not None
	
	@property
	def connected(self):
		return self._connected()

	
	def stop(self):
		self.__ie_stop.set()
	
	# Procedures
	def connect_direct(self, address):
		if self.connected:
			return True
		
		if type(address) is str:
			address = convert_str_address_to_tuple(address)
		if address is None or not isinstance(address, collections.Iterable):
			raise ValueError('Invalid address')
		
		with self.__ps.receiver(ConnStatusEvt.index) as pr:
			res = self._command(GapConnectDirectCmd(
				address=address,
				addr_type=GapAddressType.ADDRESS_TYPE_PUBLIC,
				conn_interval_min=6,
				conn_interval_max=6,
				timeout=64,
				latency=0
			))
			if res is None or res.result > 0:
				return False
			evt = next(pr)
		
		if evt is not None and evt.flags & 1:
			with self.__lock:
				self.__connection_handle = evt.connection
				self.__device_address = evt.address
				return True
		return False
		
	def disconnect(self):
		if not self.connected:
			return True
		
		with self.__lock:
			with self.__ps.receiver(ConnDisconnectedEvt.index) as pr:
				res = self._command_connection(ConnDisconnectCmd(self.__connection_handle))
				if res is None or res.result > 0:
					return False
				return next(pr) is not None
	
	def discover(self, enable=True, force=False):
		if not force:
			if self.__discovery_enabled == enable:
				return True
		self.__discovery_enabled = enable
		if enable:
			res = self._command(GapDiscoverCmd(GapDiscoverMode.DISCOVER_GENERIC))
			if res is None or res.result > 0:
				return False
			return True
		res = self._command(GapEndProcedureCmd())
		if res is None or res.result > 0:
			return False
		return True
	
	# Attribute client procedures
	def attribute_write(self, attribute_handle, value):
		if not self.connected or not (Bled112.FIRST_ATTRIBUTE_HANDLE <= attribute_handle <= Bled112.LAST_ATTRIBUTE_HANDLE):
			return False

		with self.__lock:
			with self.__ps.receiver(AcProcedureCompletedEvt.index) as pr:
				res = self._command(AcAttributeWriteCmd(self.__connection_handle, attribute_handle, value))
				if res is None or res.result > 0:
					return False
				evt = next(pr)
				
			if evt is not None and evt.result > 0:
				return False
			return True
	
	def find_by_type_value(self, uuid, value, start_handle=FIRST_ATTRIBUTE_HANDLE, end_handle=LAST_ATTRIBUTE_HANDLE):
		if not self.connected:
			return None

		with self.__lock:
			result = []
			with self.__ps.receiver([AcProcedureCompletedEvt.index, AcGroupFoundEvt.index], lambda pkt: isinstance(pkt, AcProcedureCompletedEvt)) as pr:
				res = self._command_connection(AcFindByTypeValueCmd(self.__connection_handle, start_handle, end_handle, uuid, value))
				if res is None or res.result > 0:
					return None
				for evt in pr:
					if evt is None:
						return None
					if isinstance(evt, AcGroupFoundEvt):
						result.append((evt.start, evt.end, evt.uuid))
					elif isinstance(evt, AcProcedureCompletedEvt):
						if evt.result > 0:
							return None
				return result
	
	def find_information(self, start_handle=FIRST_ATTRIBUTE_HANDLE, end_handle=LAST_ATTRIBUTE_HANDLE):
		if not self.connected:
			return None

		with self.__lock:
			result = []
			with self.__ps.receiver([AcProcedureCompletedEvt.index, AcFindInformationFoundEvt.index], lambda pkt: isinstance(pkt, AcProcedureCompletedEvt)) as pr:
				res = self._command_connection(AcFindInformationCmd(self.__connection_handle, start_handle, end_handle))
				if res is None or res.result > 0:
					return None
				for evt in pr:
					if evt is None:
						return None
					if isinstance(evt, AcFindInformationFoundEvt):
						result.append((evt.chrhandle, evt.uuid))
					elif isinstance(evt, AcProcedureCompletedEvt):
						if evt.result > 0:
							return None
				return result
	
	def read_by_group_type(self, uuid, start_handle=FIRST_ATTRIBUTE_HANDLE, end_handle=LAST_ATTRIBUTE_HANDLE):
		if not self.connected:
			return None

		with self.__lock:
			result = []
			with self.__ps.receiver([AcProcedureCompletedEvt.index, AcGroupFoundEvt.index], lambda pkt: isinstance(pkt, AcProcedureCompletedEvt)) as pr:
				res = self._command_connection(AcReadByGroupTypeCmd(self.__connection_handle, start_handle, end_handle, uuid))
				if res is None or res.result > 0:
					return None
				for evt in pr:
					if evt is None:
						return None
					if isinstance(evt, AcGroupFoundEvt):
						result.append((evt.start, evt.end, evt.uuid))
					elif isinstance(evt, AcProcedureCompletedEvt):
						if evt.result > 0:
							return None
				return result
	
	def read_by_handle(self, attribute_handle):
		if not self.connected or not (Bled112.FIRST_ATTRIBUTE_HANDLE <= attribute_handle <= Bled112.LAST_ATTRIBUTE_HANDLE):
			return None
		
		with self.__lock:
			with self.__ps.receiver([AcProcedureCompletedEvt.index, AcAttributeValueEvt.index]) as pr:
				res = self._command_connection(AcReadByHandleCmd(self.__connection_handle, attribute_handle))
				if res is None or res.result > 0:
					return None
				evt = next(pr)
				if isinstance(evt, AcAttributeValueEvt):
					return evt.value
				return None
	
	def read_by_type(self, uuid, start_handle=FIRST_ATTRIBUTE_HANDLE, end_handle=LAST_ATTRIBUTE_HANDLE):
		if not self.connected:
			return None
		
		with self.__lock:
			result = []
			with self.__ps.receiver([AcProcedureCompletedEvt.index, AcAttributeValueEvt.index], lambda pkt: isinstance(pkt, AcProcedureCompletedEvt)) as pr:
				res = self._command_connection(AcReadByTypeCmd(self.__connection_handle, start_handle, end_handle, uuid))
				if res is None or res.result > 0:
					return None
				for evt in pr:
					if evt is None:
						return None
					if isinstance(evt, AcAttributeValueEvt):
						result.append((evt.atthandle, evt.type, evt.value))
					elif isinstance(evt, AcProcedureCompletedEvt):
						if evt.result > 0:
							return None
				return result
