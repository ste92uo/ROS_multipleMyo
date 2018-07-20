# coding=utf-8

from serial import Serial
from .waitables import WaitableQueue, WaitableEvent
from threading import RLock
from select import select
from .worker_thread import WorkerThread
import logging
from .packets import *
from Queue import Queue


def bytes_to_hexstr(data, separator=' '):
	return separator.join([format(ord(x), '02x') for x in data])


class PacketSerial(WorkerThread):

	# Costants
	
	CLASS_ABBR = 'PS'
	
	# Costructors
	def __init__(self, device):
		super(PacketSerial, self).__init__()
		
		# Lock for public methods
		self.__lock = RLock()
		
		# Objects for event stop
		self.__ie_stop = WaitableEvent('stop')
		self.__stop = False
		
		# Buffer
		self.__buffer = bytes()
		
		# Serial
		self.__ser = Serial(device, timeout=0)
		
		# Output queues
		self.__registered_queues = {}
	
	def __del__(self):
		self.__ser.close()


	def _main(self):
		logging.info(self.__class__.CLASS_ABBR + ' > main')

		while not self.__stop:
			try:
				r, _, _ = select([self.__ser, self.__ie_stop], [], [])
				logging.debug(self.__class__.CLASS_ABBR + ': select ' + str(r))
				if self.__ser in r:
					self._data_received(self.__ser.read(self.__ser.in_waiting))
				if self.__ie_stop in r:
					self.__stop = True
			except KeyboardInterrupt as e:
				break
		
		logging.info(self.__class__.CLASS_ABBR + ' < main')
		
	def _data_received(self, data):
		logging.debug(self.__class__.CLASS_ABBR + ': recv ' + bytes_to_hexstr(data))
		self.__buffer += data
		while len(self.__buffer) >= 4:
			payload_length = ord(self.__buffer[1])
			if len(self.__buffer) < 4 + payload_length:
				return
			pkt = select_packet_class(Header(self.__buffer[:4]), self.__buffer[:payload_length + 4])
			self.__buffer = self.__buffer[payload_length + 4:]
			with self.__lock:
				if pkt.index in self.__registered_queues:
					for q, f in self.__registered_queues[pkt.index]:
						if not f or f(pkt):
							q.push(pkt)
				else:
					logging.debug(self.__class__.CLASS_ABBR + ': lost packet ' + str(pkt))

	def stop(self):
		self.__ie_stop.set()


	# Methods for registering and unregistering queues
	
	def register(self, packet_index, queue, filter_func=None):
		with self.__lock:
			# Create the list if it does not exist
			if packet_index not in self.__registered_queues:
				self.__registered_queues[packet_index] = []

			logging.debug('Bled: registered queue for ' + str(packet_index))
			self.__registered_queues[packet_index].append((queue, filter_func))
	
	def is_registered(self, packet_index, queue, filter_func=None):
		with self.__lock:
			if packet_index in self.__registered_queues:
				return (queue, filter_func) in self.__registered_queues[packet_index]
			return False
	
	def unregister(self, packet_index, queue, filter_func=None):
		with self.__lock:
			if self.is_registered(packet_index, queue, filter_func):
				self.__registered_queues[packet_index].remove((queue, filter_func))
				return True
			return False
	
	def send(self, packet):
		if self.__ser.is_open:
			data = packet.serialize()
			self.__ser.write(data)
			self.__ser.flush()
			logging.debug(self.__class__.CLASS_ABBR + ': sent ' + bytes_to_hexstr(data))
	
	def receiver(self, filters, ender=lambda pkt: True):
		return PacketReceiver(self, filters, ender)
		

class PacketReceiver(object):
	
	def __init__(self, ps, filters, ender=lambda pkt: True):
		logging.debug('PR: > init')
		self.__ps = ps
		self.__filters = filters
		self.__ender = ender
		self.__qst = []
		
		# Internal queue
		self.__queue = Queue()
		
		# Flag for termination
		self.__ended = False
		

		if isinstance(filters, PacketIndex):
			self.__filters = [(filters, None)]
		elif isinstance(filters, list):
			temp_list = []
			for obj in filters:
				if isinstance(obj, tuple) and len(obj) >= 2 and isinstance(obj[0], PacketIndex):
					temp_list.append((obj[0], obj[1]))
				elif isinstance(obj, PacketIndex):
					temp_list.append((obj, None))
			self.__filters = temp_list
	
	def __enter__(self):
		logging.debug('PR: > enter')
		# Register to queues
		for pi, f in self.__filters:
			t = (pi, WaitableQueue(), f)
			self.__qst.append(t)
			self.__ps.register(*t)
		return self
		
	def __exit__(self, exc_type, exc_val, exc_tb):
		logging.debug('PR: > exit')
		# Deregister to queues
		for t in self.__qst:
			self.__ps.unregister(*t)
		
	def __iter__(self):
		logging.debug('PR: > iter')
		return self
	
	def next(self):
		logging.debug('PR: > next')
		while True:
			# Look for packets in queue
			if not self.__queue.empty():
				return self.__queue.get()

			if self.__ended:
				raise StopIteration

			qs = [q for pi, q, f in self.__qst]
			try:
				r, _, _ = select(qs, [], [])
			except KeyboardInterrupt:
				return None
			pkts = []
			for q in r:
				pkt = q.pop()
				if pkt is not None:
					if self.__ender(pkt):
						self.__ended = True
					self.__queue.put(pkt)
