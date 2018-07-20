from enum import Enum
from struct import pack, unpack


class BluetoothUUID(Enum):
	# Declarations
	DECL_PRIMARY_SERVICE = 0x2800
	DECL_SECONDARY_SERVICE = 0x2801
	DECL_INCLUDE = 0x2802
	DECL_CHARACTERISTIC = 0x2803
	
	PS_GENERIC_ACCESS = 0x1800
	CH_DEVICE_NAME = 0x2a00
	
	PS_GENERIC_ATTRIBUTE = 0x1801
	PS_DEVICE_INFORMATION = 0x180a
	
	PS_BATTERY_SERVICE = 0x180f
	CH_BATTERY_LEVEL = 0x2a19
	
	def __new__(cls, value):
		if value > 0xffffffff or value < 0:
			raise ValueError('UUID not valid')
		obj = object.__new__(cls)
		# obj._value_ = b'\xfb\x34\x9b\x5f\x80\x00\x00\x80\x00\x10\x00\x00' + pack('<I', value)
		if value < 0xffff:
			obj._value_ = pack('<H', value)
			return obj
		obj._value_ = pack('<I', value)
		return obj
	
	def __int__(self):
		return unpack('<H', self._value_)[0]


class MyoUUID(Enum):
	CONTROL_SERVICE = 0x0001
	MYO_INFO_CHARACTERISTIC = 0x0101
	FIRMWARE_VERSION_CHARACTERISTIC = 0x0201
	COMMAND_CHARACTERISTIC = 0x0401
	
	IMU_DATA_SERVICE = 0x0002
	IMU_DATA_CHARACTERISTIC = 0x0402
	MOTION_EVENT_CHARACTERISTIC = 0x0502
	
	CLASSIFIER_SERVICE = 0x0003
	CLASSIFIER_EVENT_CHARACTERISTIC = 0x0103
	
	EMG_DATA_SERVICE = 0x0005
	EMG_DATA0_CHARACTERISTIC = 0x0105
	EMG_DATA1_CHARACTERISTIC = 0x0205
	EMG_DATA2_CHARACTERISTIC = 0x0305
	EMG_DATA3_CHARACTERISTIC = 0x0405
	
	def __new__(cls, value):
		if value > 0xffff or value < 0:
			raise ValueError('UUID not valid')
		obj = object.__new__(cls)
		obj._value_ = b'\x42\x48\x12\x4a\x7f\x2c\x48\x47\xb9\xde\x04\xa9' + pack('<I', value + 0xd5060000)
		return obj
