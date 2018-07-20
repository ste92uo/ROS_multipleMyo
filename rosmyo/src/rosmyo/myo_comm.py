from enum import Enum, IntEnum
from struct import pack


class MyoState(IntEnum):
	IDLE = 0
	CONNECTED = 1
	DISCOVERING = 2


class MyoQueues(IntEnum):
	IMU = 0
	EMG = 1
	SCAN_RESPONSE = 2


class MyoCommand(Enum):
	SET_MODE = 0x01
	VIBRATE = 0x03
	DEEP_SLEEP = 0x04
	VIBRATE2 = 0x07
	SET_SLEEP_MODE = 0x09
	UNLOCK = 0x0a
	USER_ACTION = 0x0b


class MyoEmgMode(Enum):
	NONE = 0x00
	SEND_EMG = 0x02
	SEND_EMG_RAW = 0x03


class MyoImuMode(Enum):
	NONE = 0x00
	SEND_DATA = 0x01
	SEND_EVENTS = 0x02
	SEND_ALL = 0x03
	SEND_RAW = 0x04


class MyoClassifierMode(Enum):
	DISABLED = 0x00
	ENABLED = 0x01


class MyoSleepModes(Enum):
	NORMAL = 0
	NEVER_SLEEP = 1


class MyoVibrateModes(Enum):
	NONE = 0x00
	SHORT = 0x01
	MEDIUM = 0x02
	LONG = 0x03


class MyoClientCharacteristicConfiguration(Enum):
	NONE = 0x0000
	NOTIFY = 0x0001
	INDICATE = 0x0002
	
	def __new__(cls, value):
		if value > 0xffff or value < 0:
			raise ValueError('Value not valid')
		obj = object.__new__(cls)
		obj._value_ = pack('<H', value)
		return obj


class MyoUnlockModes(Enum):
	LOCK = 0x00
	TIMED = 0x01
	HOLD = 0x02


# Only header packet
class MyoHeader(object):
	
	def __init__(self, command):
		self.__command = command
	
	def __str__(self):
		return "Command={}".format(self.__command.name)
	
	def __repr__(self):
		return self.__str__()
	
	@property
	def command(self):
		return self.__command
	
	def serialize(self):
		return pack('<B', self.__command.value)


class MyoSetModeCommand(MyoHeader):
	def __init__(self, emg_mode=MyoEmgMode.NONE, imu_mode=MyoImuMode.NONE, classifier_mode=MyoClassifierMode.DISABLED):
		super(MyoSetModeCommand, self).__init__(MyoCommand.SET_MODE)
		self.__emg_mode = emg_mode
		self.__imu_mode = imu_mode
		self.__classifier_mode = classifier_mode
	
	@property
	def imu_mode(self):
		return self.__imu_mode
	
	@property
	def emg_mode(self):
		return self.__emg_mode
	
	@property
	def classifier_mode(self):
		return self.__classifier_mode
	
	def __str__(self):
		return ', '.join([super(MyoSetModeCommand, self).__str__()] + ['EMG mode=' + self.__emg_mode.name, 'IMU mode=' + self.__imu_mode.name, 'Classifier mode=' + self.__classifier_mode.name])
	
	def serialize(self):
		payload = pack('<BBB', self.__emg_mode.value, self.__imu_mode.value, self.__classifier_mode.value)
		return super(MyoSetModeCommand, self).serialize() + pack('<B', len(payload)) + payload


class MyoVibrateCommand(MyoHeader):
	def __init__(self, mode=MyoVibrateModes.SHORT):
		super(MyoVibrateCommand, self).__init__(MyoCommand.VIBRATE)
		self.__mode = mode
	
	@property
	def mode(self):
		return self.__mode
	
	def __str__(self):
		return ', '.join([super(MyoVibrateCommand, self).__str__()] + ['Mode=' + self.__mode.name])
	
	def serialize(self):
		payload = pack('<B', self.__mode.value)
		return super(MyoVibrateCommand, self).serialize() + pack('<B', len(payload)) + payload


class MyoSetSleepModeCommand(MyoHeader):
	def __init__(self, mode=MyoSleepModes.NORMAL):
		super(MyoSetSleepModeCommand, self).__init__(MyoCommand.SET_SLEEP_MODE)
		self.__mode = mode
	
	@property
	def mode(self):
		return self.__mode
	
	def __str__(self):
		return ', '.join([super(MyoSetSleepModeCommand, self).__str__()] + ['Mode=' + self.__mode.name])
	
	def serialize(self):
		payload = pack('<B', self.__mode.value)
		return super(MyoSetSleepModeCommand, self).serialize() + pack('<B', len(payload)) + payload


class MyoUnlockCommand(MyoHeader):
	def __init__(self, mode=MyoUnlockModes.HOLD):
		super(MyoUnlockCommand, self).__init__(MyoCommand.UNLOCK)
		self.__mode = mode
	
	@property
	def mode(self):
		return self.__mode
	
	def __str__(self):
		return ', '.join([super(MyoUnlockCommand, self).__str__()] + ['Mode=' + self.__mode.name])
	
	def serialize(self):
		payload = pack('<B', self.__mode.value)
		return super(MyoUnlockCommand, self).serialize() + pack('<B', len(payload)) + payload
