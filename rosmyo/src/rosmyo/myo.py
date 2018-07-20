# coding=utf-8

# Myo Class for device setting
from .bled112 import Bled112
from .packets import *
from threading import Thread, RLock
from .waitables import WaitableQueue, WaitableEvent
from functools import partial
from select import select
from .worker_thread import WorkerThread
from .myo_comm import *
import logging
from enum import Enum

from .packet_serial import PacketSerial
from .uuid import BluetoothUUID, MyoUUID
import re
import rospy
import logging


# Note:
# CCC = Client Characteristic Configuration: Notify and/or Indicate enable

class AttributeHandles(Enum):
	# Bluetooth services
	
	# org.bluetooth.characteristic.gap.device_name
	DeviceName = 0x0003
	# org.bluetooth.characteristic.gap.appearance
	Appearance = 0x0005
	# org.bluetooth.characteristic.gap.peripheral_preferred_connection_parameters
	PheriperalPreferredConnectionParameters = 0x0007
	# org.bluetooth.characteristic.gatt.service_changed
	ServiceChanged = 0x000a
	ServiceChangedCCC = 0x000b
	# org.bluetooth.characteristic.manufacturer_name_string
	ManufacturerNameString = 0x000e
	# org.bluetooth.characteristic.battery_level
	BatteryLevel = 0x0011
	BatteryLevelCCC = 0x0012
	
	# Myo services
	
	# - Control Service -
	MyoInfo = 0x0015
	FirmwareVersion = 0x0017
	Command = 0x0019
	
	# - Imu Service -
	IMUData = 0x001c
	IMUDataCCC = 0x001d
	MotionEvent = 0x001f
	MotionEventCCC = 0x0020
	
	# - Classifier Service -
	ClassifierEvent = 0x0023
	ClassifierEventCCC = 0x0024
	
	# - Emg Service -
	EmgData0 = 0x002b
	EmgData0CCC = 0x002c
	EmgData1 = 0x002e
	EmgData1CCC = 0x002f
	EmgData2 = 0x0031
	EmgData2CCC = 0x0032
	EmgData3 = 0x0034
	EmgData3CCC = 0x0035


def bytes_to_hexstr(data, separator=' '):
	return separator.join([format(x, '02x') for x in data])

	
def _filter_imu(pkt):
	return pkt.atthandle == AttributeHandles.IMUData.value


def _filter_emg(pkt):
	hndls = (
		AttributeHandles.EmgData0.value,
		AttributeHandles.EmgData1.value,
		AttributeHandles.EmgData2.value,
		AttributeHandles.EmgData3.value
	)
	return pkt.atthandle in hndls


def _filter_classifier(pkt):
	return pkt.atthandle == AttributeHandles.ClassifierEvent.value


class Myo(WorkerThread):
	
	# Costants
	CLASS_ABBR = 'Myo'
	MAX_CONNECTIONS = 3
	MYO_ADVERTISE_UUID = '\x42\x48\x12\x4a\x7f\x2c\x48\x47\xb9\xde\x04\xa9\x01\x00\x06\xd5'

	# Costructors
	def __init__(self, dongle):
		super(Myo, self).__init__()
		
		logging.info(self.__class__.CLASS_ABBR + ': init')
		
		# Lock for public methods
		self.__lock = RLock()
		
		# Objects for event stop
		self.__ie_stop = WaitableEvent('stop')
		self.__stop = False
		
		# Instantiate PacketSerial for the proper dongle
		self.__packet_serial = PacketSerial(dongle)
		self.__packet_serial.start()
		
		# Queues for data receiving events
		self.__wq_imu = None
		self.__filter_imu = _filter_imu
		self.__wq_emg = None
		self.__filter_emg = _filter_emg
		self.__wq_classifier = None
		self.__filter_classifier = _filter_classifier
		
		# Adapter
		self.__bled = Bled112(self.__packet_serial)
		self.__bled.start()
	
	def __del__(self):
		if self.connected:
			self.disconnect()
		self.__bled.stop()
		self.__bled.join(timeout=1)
		self.__packet_serial.stop()
		self.__packet_serial.join(timeout=1)

	
	def _main(self):
		logging.info(self.__class__.CLASS_ABBR + ' > main')

		while not self.__stop:
			try:
				r, _, _ = select([self.__ie_stop], [], [])
				rospy.loginfo(self.__class__.CLASS_ABBR + ': select ' + str(r))
				if self.__ie_stop in r:
					self.__stop = True
			except KeyboardInterrupt as e:
				break
	
	def _enable_imu(self, enable=MyoClientCharacteristicConfiguration.NOTIFY):
		return self.__bled.attribute_write(AttributeHandles.IMUDataCCC.value, enable.value)
	
	def _enable_emg(self, enable=MyoClientCharacteristicConfiguration.NOTIFY):
		ahs = [
			AttributeHandles.EmgData0CCC.value,
			AttributeHandles.EmgData1CCC.value,
			AttributeHandles.EmgData2CCC.value,
			AttributeHandles.EmgData3CCC.value
		]
		return all([self.__bled.attribute_write(ah, enable.value) for ah in ahs])
	
	def _enable_classifier(self, enable=MyoClientCharacteristicConfiguration.INDICATE):
		return self.__bled.attribute_write(AttributeHandles.ClassifierEventCCC.value, enable.value)
	
	def _command(self, cmd):
		return self.__bled.attribute_write(AttributeHandles.Command.value, cmd.serialize())
	
	# Properties
	@property
	def firmware_version(self):
		values = self.__bled.read_by_type(MyoUUID.FIRMWARE_VERSION_CHARACTERISTIC.value)
		if values is None or len(values) == 0:
			return None
		_, _, v = values[0]
		return unpack('<HHHH', v)
	
	@property
	def name(self):
		values = self.__bled.read_by_type(BluetoothUUID.CH_DEVICE_NAME.value)
		if values is None or len(values) == 0:
			return None
		_, _, v = values[0]
		return v
	
	@name.setter
	def name(self, value):
		values = self.__bled.read_by_type(BluetoothUUID.CH_DEVICE_NAME.value)
		if values is None or len(values) == 0:
			return
		ah, _, cn = values[0]
	
	@property
	def battery_level(self):
		values = self.__bled.read_by_type(BluetoothUUID.CH_BATTERY_LEVEL.value)
		if values is None or len(values) == 0:
			return None
		_, _, v = values[0]
		return ord(v[0])
	
	@property
	def connected(self):
		return self.__bled.connected
	
	@property
	def imu_queue(self):
		return self.__wq_imu
	
	@imu_queue.setter
	def imu_queue(self, value):
		if self.imu_queue is not None:
			self.__packet_serial.unregister(AcAttributeValueEvt.index, self.__wq_imu, self.__filter_imu)
		self.__wq_imu = value
		self.__packet_serial.register(AcAttributeValueEvt.index, self.__wq_imu, self.__filter_imu)
	
	@property
	def emg_queue(self):
		return self.__wq_emg
	
	@emg_queue.setter
	def emg_queue(self, value):
		if self.emg_queue is not None:
			self.__packet_serial.unregister(AcAttributeValueEvt.index, self.__wq_emg, self.__filter_emg)
		self.__wq_emg = value
		self.__packet_serial.register(AcAttributeValueEvt.index, self.__wq_emg, self.__filter_emg)
	
	@property
	def classifier_queue(self):
		return self.__wq_classifier
	
	@classifier_queue.setter
	def classifier_queue(self, value):
		if self.classifier_queue is not None:
			self.__packet_serial.unregister(AcAttributeValueEvt.index, self.__wq_classifier, self.__filter_classifier)
		self.__wq_classifier = value
		self.__packet_serial.register(AcAttributeValueEvt.index, self.__wq_classifier, self.__filter_classifier)
	
	def connect(self, device):
		if isinstance(device, str):
			if re.search(r'^([0-9a-f]{2}:){5}[0-9a-f]{2}$', device, re.IGNORECASE):
				result = self.__bled.connect_direct(device)
			else:
				return self._connect_by_name(device)
		else:
			result = self.__bled.connect_direct(device)
		return result
	
	def _connect_by_name(self, device_name):
		if self.__bled.connected:
			if self.name != device_name:
				self.__bled.disconnect()
			else:
				return True
		
		checked_addresses = []
		address = None
		
		with self.__packet_serial.receiver(GapScanResponseEvt.index, lambda pkt: False) as pr:
			# Start discovery
			self.__bled.discover(enable=True, force=True)
			
			# Continue untill the correct device is found
			while True:
				evt = None
				while evt is None:
					evt = next(pr)
					# Verify the event is of type GapScanResponseEvt
					if not isinstance(evt, GapScanResponseEvt):
						evt = None
				
				# Verify the adress has not been analyzed yet
				if evt.sender in checked_addresses:
					continue
				
				rospy.loginfo('Myo: new device discovered [{}]'.format(bytes_to_hexstr(evt.sender, ':')))
				# Add the adress to the list of already analyzed ones
				checked_addresses.append(evt.sender)
				
				# Verify the adress belongs to a device of type MYO
				if len(evt.data) != 31:
					rospy.loginfo('Myo: wrong advertise data length [{}], skipping device'.format(len(evt.data)))
					continue
				if evt.data[-16:] != Myo.MYO_ADVERTISE_UUID:
					rospy.loginfo('Myo: advertise UUID not found, skipping device')
					continue
				
				# Check the first 8 characters of the name
				device_name_short = device_name[:8]
				adv_name = evt.data[2:2+len(device_name_short)]
				if device_name_short != adv_name:
					continue
				
				# Connect
				if not self.__bled.connect_direct(evt.sender):
					rospy.logwarn('Myo: connection failed, skipping device'.format(adv_name))
					self.__bled.discover(enable=True, force=True)
					continue
				
				# Check if the connected device is the correct one
				name = self.name
				if name != device_name:
					rospy.loginfo('Myo: wrong name [{}], skipping device'.format(name))
					self.__bled.disconnect()
					self.__bled.discover(enable=True, force=True)
					continue
				
				return True
	
	def disconnect(self):
		self.set_mode()
		self.set_sleep_mode(MyoSleepModes.NORMAL)
		return self.__bled.disconnect()
	
	def set_mode(self, emg_mode=MyoEmgMode.NONE, imu_mode=MyoImuMode.NONE, classifier_mode=MyoClassifierMode.DISABLED):
		if imu_mode == MyoImuMode.NONE:
			self._enable_imu(MyoClientCharacteristicConfiguration.NONE)
		else:
			self._enable_imu(MyoClientCharacteristicConfiguration.NOTIFY)
		
		if emg_mode == MyoEmgMode.NONE:
			self._enable_emg(MyoClientCharacteristicConfiguration.NONE)
		else:
			self._enable_emg(MyoClientCharacteristicConfiguration.NOTIFY)
		
		if classifier_mode == MyoClassifierMode.DISABLED:
			self._enable_classifier(MyoClientCharacteristicConfiguration.NONE)
		else:
			self._enable_classifier(MyoClientCharacteristicConfiguration.INDICATE)
		
		return self._command(MyoSetModeCommand(emg_mode, imu_mode, classifier_mode))
	
	def unlock(self, mode=MyoUnlockModes.HOLD):
		return self._command(MyoUnlockCommand(mode))
	
	def set_sleep_mode(self, mode=MyoSleepModes.NORMAL):
		return self._command(MyoSetSleepModeCommand(mode))
	
	def vibrate(self, mode=MyoVibrateModes.SHORT):
		return self._command(MyoVibrateCommand(mode))
