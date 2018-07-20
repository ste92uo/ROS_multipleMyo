# Imports
from collections import namedtuple
from struct import pack, unpack
from functools import reduce
from enum import IntEnum

# Named tuples
PacketIndex = namedtuple('PacketIndex', ['type_id', 'class_id', 'message_id'])


# Type of packets
class PacketType(IntEnum):
	COMMAND = 0x00
	RESPONSE = 0x00
	EVENT = 0x80


# Packets class
class PacketClass(IntEnum):
	SYSTEM = 0
	PERSISTENT_STORE = 1
	ATTRIBUTE_DATABASE = 2
	CONNECTION = 3
	ATTRIBUTE_CLIENT = 4
	SECURITY_MANAGER = 5
	GENERIC_ACCESS_PROFILE = 6
	HARDWARE = 7
	TESTING = 8
	DEVICE_FIRMWARE_UPGRADE = 9


# Packets message
class PacketMessage(IntEnum):
	SYS_RESET = 0
	SYS_HELLO = 1
	SYS_ADDRESS_GET = 2
	SYS_REG_WRITE = 3
	SYS_REG_READ = 4
	SYS_GET_COUNTERS = 5
	SYS_GET_CONNECTIONS = 6
	SYS_READ_MEMORY = 7
	SYS_GET_INFO = 8
	SYS_ENDPOINT_TX = 9
	SYS_WHITELIST_APPEND = 10
	SYS_WHITELIST_REMOVE = 11
	SYS_WHITELIST_CLEAR = 12
	SYS_ENDPOINT_RX = 13
	SYS_ENDPOINT_SET_WATERMARKS = 14
	SYS_BOOT = 0
	SYS_DEBUG = 1
	SYS_ENDPOINT_WATERMARK_RX = 2
	SYS_ENDPOINT_WATERMARK_TX = 3
	SYS_SCRIPT_FAILURE = 4
	SYS_NO_LICENSE_KEY = 5
	SYS_PROTOCOL_ERROR = 6
	PS_PS_DEFRAG = 0
	PS_PS_DUMP = 1
	PS_PS_ERASE_ALL = 2
	PS_PS_SAVE = 3
	PS_PS_LOAD = 4
	PS_PS_ERASE = 5
	PS_ERASE_PAGE = 6
	PS_WRITE_WORDS = 7
	PS_PS_KEY = 0
	ADB_WRITE = 0
	ADB_READ = 1
	ADB_READ_TYPE = 2
	ADB_USER_READ_RESPONSE = 3
	ADB_USER_WRITE_RESPONSE = 4
	ADB_VALUE = 0
	ADB_USER_READ_REQUEST = 1
	ADB_STATUS = 2
	CONN_DISCONNECT = 0
	CONN_GET_RSSI = 1
	CONN_UPDATE = 2
	CONN_VERSION_UPDATE = 3
	CONN_CHANNEL_MAP_GET = 4
	CONN_CHANNEL_MAP_SET = 5
	CONN_FEATURES_GET = 6
	CONN_GET_STATUS = 7
	CONN_RAW_TX = 8
	CONN_STATUS = 0
	CONN_VERSION_IND = 1
	CONN_FEATURE_IND = 2
	CONN_RAW_RX = 3
	CONN_DISCONNECTED = 4
	AC_FIND_BY_TYPE_VALUE = 0
	AC_READ_BY_GROUP_TYPE = 1
	AC_READ_BY_TYPE = 2
	AC_FIND_INFORMATION = 3
	AC_READ_BY_HANDLE = 4
	AC_ATTRIBUTE_WRITE = 5
	AC_WRITE_COMMAND = 6
	AC_INDICATE_CONFIRM = 7
	AC_READ_LONG = 8
	AC_PREPARE_WRITE = 9
	AC_EXECUTE_WRITE = 10
	AC_READ_MULTIPLE = 11
	AC_INDICATED = 0
	AC_PROCEDURE_COMPLETED = 1
	AC_GROUP_FOUND = 2
	AC_ATTRIBUTE_FOUND = 3
	AC_FIND_INFORMATION_FOUND = 4
	AC_ATTRIBUTE_VALUE = 5
	AC_READ_MULTIPLE_RESPONSE = 6
	SM_ENCRYPT_START = 0
	SM_SET_BONDABLE_MODE = 1
	SM_DELETE_BONDING = 2
	SM_SET_PARAMETERS = 3
	SM_PASSKEY_ENTRY = 4
	SM_GET_BONDS = 5
	SM_SET_OOB_DATA = 6
	SM_SMP_DATA = 0
	SM_BONDING_FAIL = 1
	SM_PASSKEY_DISPLAY = 2
	SM_PASSKEY_REQUEST = 3
	SM_BOND_STATUS = 4
	GAP_SET_PRIVACY_FLAGS = 0
	GAP_SET_MODE = 1
	GAP_DISCOVER = 2
	GAP_CONNECT_DIRECT = 3
	GAP_END_PROCEDURE = 4
	GAP_CONNECT_SELECTIVE = 5
	GAP_SET_FILTERING = 6
	GAP_SET_SCAN_PARAMETERS = 7
	GAP_SET_ADV_PARAMETERS = 8
	GAP_SET_ADV_DATA = 9
	GAP_SET_DIRECTED_CONNECTABLE_MODE = 10
	GAP_SCAN_RESPONSE = 0
	GAP_MODE_CHANGED = 1
	HW_IO_PORT_CONFIG_IRQ = 0
	HW_SET_SOFT_TIMER = 1
	HW_ADC_READ = 2
	HW_IO_PORT_CONFIG_DIRECTION = 3
	HW_IO_PORT_CONFIG_FUNCTION = 4
	HW_IO_PORT_CONFIG_PULL = 5
	HW_IO_PORT_WRITE = 6
	HW_IO_PORT_READ = 7
	HW_SPI_CONFIG = 8
	HW_SPI_TRANSFER = 9
	HW_I2C_READ = 10
	HW_I2C_WRITE = 11
	HW_SET_TXPOWER = 12
	HW_TIMER_COMPARATOR = 13
	HW_IO_PORT_STATUS = 0
	HW_SOFT_TIMER = 1
	HW_ADC_RESULT = 2
	TEST_PHY_TX = 0
	TEST_PHY_RX = 1
	TEST_PHY_END = 2
	TEST_PHY_RESET = 3
	TEST_GET_CHANNEL_MAP = 4
	TEST_DEBUG = 5
	DFU_RESET = 0
	DFU_FLASH_SET_ADDRESS = 1
	DFU_FLASH_UPLOAD = 2
	DFU_FLASH_UPLOAD_FINISH = 3
	DFU_BOOT = 0


# Enums
class SysEndpoints(IntEnum):
	ENDPOINT_API = 0
	ENDPOINT_TEST = 1
	ENDPOINT_SCRIPT = 2
	ENDPOINT_USB = 3
	ENDPOINT_UART0 = 4
	ENDPOINT_UART1 = 5


class AdbAttributeChangeReason(IntEnum):
	ATTRIBUTE_CHANGE_REASON_WRITE_REQUEST = 0
	ATTRIBUTE_CHANGE_REASON_WRITE_COMMAND = 1
	ATTRIBUTE_CHANGE_REASON_WRITE_REQUEST_USER = 2


class AdbAttributeStatusFlag(IntEnum):
	ATTRIBUTE_STATUS_FLAG_NOTIFY = 1
	ATTRIBUTE_STATUS_FLAG_INDICATE = 2


class ConnConnstatus(IntEnum):
	CONNECTED = 1
	ENCRYPTED = 2
	COMPLETED = 4
	PARAMETERS_CHANGE = 8


class AcAttributeValueTypes(IntEnum):
	ATTRIBUTE_VALUE_TYPE_READ = 0
	ATTRIBUTE_VALUE_TYPE_NOTIFY = 1
	ATTRIBUTE_VALUE_TYPE_INDICATE = 2
	ATTRIBUTE_VALUE_TYPE_READ_BY_TYPE = 3
	ATTRIBUTE_VALUE_TYPE_READ_BLOB = 4
	ATTRIBUTE_VALUE_TYPE_INDICATE_RSP_REQ = 5


class SmBondingKey(IntEnum):
	BONDING_KEY_LTK = 1
	BONDING_KEY_ADDR_PUBLIC = 2
	BONDING_KEY_ADDR_STATIC = 4
	BONDING_KEY_IRK = 8
	BONDING_KEY_EDIVRAND = 16
	BONDING_KEY_CSRK = 32
	BONDING_KEY_MASTERID = 64


class SmIoCapability(IntEnum):
	IO_CAPABILITY_DISPLAYONLY = 0
	IO_CAPABILITY_DISPLAYYESNO = 1
	IO_CAPABILITY_KEYBOARDONLY = 2
	IO_CAPABILITY_NOINPUTNOOUTPUT = 3
	IO_CAPABILITY_KEYBOARDDISPLAY = 4


class GapAddressType(IntEnum):
	ADDRESS_TYPE_PUBLIC = 0
	ADDRESS_TYPE_RANDOM = 1


class GapDiscoverableMode(IntEnum):
	NON_DISCOVERABLE = 0
	LIMITED_DISCOVERABLE = 1
	GENERAL_DISCOVERABLE = 2
	BROADCAST = 3
	USER_DATA = 4


class GapConnectableMode(IntEnum):
	NON_CONNECTABLE = 0
	DIRECTED_CONNECTABLE = 1
	UNDIRECTED_CONNECTABLE = 2
	SCANNABLE_CONNECTABLE = 3


class GapDiscoverMode(IntEnum):
	DISCOVER_LIMITED = 0
	DISCOVER_GENERIC = 1
	DISCOVER_OBSERVATION = 2


class GapAdTypes(IntEnum):
	AD_TYPE_NONE = 0
	AD_TYPE_FLAGS = 1
	AD_TYPE_SERVICES_16BIT_MORE = 2
	AD_TYPE_SERVICES_16BIT_ALL = 3
	AD_TYPE_SERVICES_32BIT_MORE = 4
	AD_TYPE_SERVICES_32BIT_ALL = 5
	AD_TYPE_SERVICES_128BIT_MORE = 6
	AD_TYPE_SERVICES_128BIT_ALL = 7
	AD_TYPE_LOCALNAME_SHORT = 8
	AD_TYPE_LOCALNAME_COMPLETE = 9
	AD_TYPE_TXPOWER = 10


class GapAdvertisingPolicy(IntEnum):
	ADV_POLICY_ALL = 0
	ADV_POLICY_WHITELIST_SCAN = 1
	ADV_POLICY_WHITELIST_CONNECT = 2
	ADV_POLICY_WHITELIST_ALL = 3


class GapScanPolicy(IntEnum):
	SCAN_POLICY_ALL = 0
	SCAN_POLICY_WHITELIST = 1


# Only header packet
class Header(object):
	
	def __init__(self, init_data=None, type_id=None, payload_length=None, class_id=None, message_id=None):
		
		# Header initialized by another Header
		if isinstance(init_data, Header):
			self.__typ, self.__pl, self.__cls, self.__msg = init_data.type_id, init_data.payload_length, init_data.class_id, init_data.message_id
			return
		
		# Header initialized by bytes
		if isinstance(init_data, bytes):
			if len(init_data) < 4:
				raise ValueError('Header data must be at least 4 bytes length')
			# Extract data of interest
			self.__typ, self.__pl, self.__cls, self.__msg = unpack('4B', init_data)
			return
		
		# Init empty data
		if init_data is None:
			self.__typ, self.__pl, self.__cls, self.__msg = type_id, payload_length, class_id, message_id
			return
		
		raise ValueError('Wrong init data: [{}] {} '.format(str(type(init_data)), init_data))
	
	def __str__(self):
		return "Type={}, Payload Length={}, Class={}, Message={}".format(self.type_id, self.payload_length, self.class_id, self.message_id)
	
	def __repr__(self):
		return self.__str__()
	
	def check_validity(self):
		if not self.is_valid:
			raise ValueError('Instance not correctly initialized')
	
	@property
	def is_valid(self):
		if reduce(lambda a, b: a or b, [x is None for x in (self.type_id, self.payload_length, self.class_id, self.message_id)], False):
			return False
		return True
	
	@property
	def index(self):
		self.check_validity()
		return PacketIndex(int(self.__typ), int(self.__cls), int(self.__msg))
	
	@property
	def type_id(self):
		return self.__typ
	
	@property
	def payload_length(self):
		return self.__pl
	
	@payload_length.setter
	def payload_length(self, val):
		self.__pl = val
	
	@property
	def class_id(self):
		return self.__cls
	
	@property
	def message_id(self):
		return self.__msg
	
	def serialize(self):
		self.check_validity()
		return pack('4B', int(self.__typ), self.__pl, int(self.__cls), int(self.__msg))


# Generic packet
class Packet(Header):
	
	def __init__(self, init_data=None, payload=bytes(), **kvargs):
		super(Packet, self).__init__(init_data, **kvargs)
		self._payload = payload
	
	def serialize(self):
		return super(Packet, self).serialize() + self._payload
	
	@property
	def payload_length(self):
		if self._payload is None:
			return None
		return len(self._payload)
	
	def __str__(self):
		return super(Packet, self).__str__() + ', Payload=' + str(self._payload)
	
	def __repr__(self):
		return self.__str__()


# Command subclasses
class SysResetCmd(Header):
	def __init__(self, boot_in_dfu):
		super(SysResetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_RESET
		)
		self._boot_in_dfu = boot_in_dfu
	
	@property
	def boot_in_dfu(self):
		return self._boot_in_dfu
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._boot_in_dfu))
		self.payload_length = len(payload)
		return super(SysResetCmd, self).serialize() + payload


class SysHelloCmd(Header):
	def __init__(self):
		super(SysHelloCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_HELLO
		)


class SysAddressGetCmd(Header):
	def __init__(self):
		super(SysAddressGetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_ADDRESS_GET
		)


class SysRegWriteCmd(Header):
	def __init__(self, address, value):
		super(SysRegWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_REG_WRITE
		)
		self._address = address
		self._value = value
	
	@property
	def address(self):
		return self._address
	
	@property
	def value(self):
		return self._value
	
	def serialize(self):
		payload = pack('<' + 'H' + 'B', int(self._address), int(self._value))
		self.payload_length = len(payload)
		return super(SysRegWriteCmd, self).serialize() + payload


class SysRegReadCmd(Header):
	def __init__(self, address):
		super(SysRegReadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_REG_READ
		)
		self._address = address
	
	@property
	def address(self):
		return self._address
	
	def serialize(self):
		payload = pack('<' + 'H', int(self._address))
		self.payload_length = len(payload)
		return super(SysRegReadCmd, self).serialize() + payload


class SysGetCountersCmd(Header):
	def __init__(self):
		super(SysGetCountersCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_GET_COUNTERS
		)


class SysGetConnectionsCmd(Header):
	def __init__(self):
		super(SysGetConnectionsCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_GET_CONNECTIONS
		)


class SysReadMemoryCmd(Header):
	def __init__(self, address, length):
		super(SysReadMemoryCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_READ_MEMORY
		)
		self._address = address
		self._length = length
	
	@property
	def address(self):
		return self._address
	
	@property
	def length(self):
		return self._length
	
	def serialize(self):
		payload = pack('<' + 'I' + 'B', int(self._address), int(self._length))
		self.payload_length = len(payload)
		return super(SysReadMemoryCmd, self).serialize() + payload


class SysGetInfoCmd(Header):
	def __init__(self):
		super(SysGetInfoCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_GET_INFO
		)


class SysEndpointTxCmd(Header):
	def __init__(self, endpoint, data):
		super(SysEndpointTxCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_ENDPOINT_TX
		)
		self._endpoint = endpoint
		self._data = data
	
	@property
	def endpoint(self):
		return self._endpoint
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + str(len(self._data) + 1) + 'B', int(self._endpoint), len(self._data), *self._data)
		self.payload_length = len(payload)
		return super(SysEndpointTxCmd, self).serialize() + payload


class SysWhitelistAppendCmd(Header):
	def __init__(self, address, address_type):
		super(SysWhitelistAppendCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_WHITELIST_APPEND
		)
		self._address = address
		self._address_type = address_type
	
	@property
	def address(self):
		return self._address
	
	@property
	def address_type(self):
		return self._address_type
	
	def serialize(self):
		payload = pack('<' + '6B', *self._address) + pack('<B', int(self._address_type))
		self.payload_length = len(payload)
		return super(SysWhitelistAppendCmd, self).serialize() + payload


class SysWhitelistRemoveCmd(Header):
	def __init__(self, address, address_type):
		super(SysWhitelistRemoveCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_WHITELIST_REMOVE
		)
		self._address = address
		self._address_type = address_type
	
	@property
	def address(self):
		return self._address
	
	@property
	def address_type(self):
		return self._address_type
	
	def serialize(self):
		payload = pack('<' + '6B', *self._address) + pack('<B', int(self._address_type))
		self.payload_length = len(payload)
		return super(SysWhitelistRemoveCmd, self).serialize() + payload


class SysWhitelistClearCmd(Header):
	def __init__(self):
		super(SysWhitelistClearCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_WHITELIST_CLEAR
		)


class SysEndpointRxCmd(Header):
	def __init__(self, endpoint, size):
		super(SysEndpointRxCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_ENDPOINT_RX
		)
		self._endpoint = endpoint
		self._size = size
	
	@property
	def endpoint(self):
		return self._endpoint
	
	@property
	def size(self):
		return self._size
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._endpoint), int(self._size))
		self.payload_length = len(payload)
		return super(SysEndpointRxCmd, self).serialize() + payload


class SysEndpointSetWatermarksCmd(Header):
	def __init__(self, endpoint, rx, tx):
		super(SysEndpointSetWatermarksCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SYSTEM,
			message_id=PacketMessage.SYS_ENDPOINT_SET_WATERMARKS
		)
		self._endpoint = endpoint
		self._rx = rx
		self._tx = tx
	
	@property
	def endpoint(self):
		return self._endpoint
	
	@property
	def rx(self):
		return self._rx
	
	@property
	def tx(self):
		return self._tx
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._endpoint), int(self._rx), int(self._tx))
		self.payload_length = len(payload)
		return super(SysEndpointSetWatermarksCmd, self).serialize() + payload


class PsPsDefragCmd(Header):
	def __init__(self):
		super(PsPsDefragCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_PS_DEFRAG
		)


class PsPsDumpCmd(Header):
	def __init__(self):
		super(PsPsDumpCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_PS_DUMP
		)


class PsPsEraseAllCmd(Header):
	def __init__(self):
		super(PsPsEraseAllCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_PS_ERASE_ALL
		)


class PsPsSaveCmd(Header):
	def __init__(self, key, value):
		super(PsPsSaveCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_PS_SAVE
		)
		self._key = key
		self._value = value
	
	@property
	def key(self):
		return self._key
	
	@property
	def value(self):
		return self._value
	
	def serialize(self):
		payload = pack('<' + 'H' + str(len(self._value) + 1) + 'B', int(self._key), len(self._value), *self._value)
		self.payload_length = len(payload)
		return super(PsPsSaveCmd, self).serialize() + payload


class PsPsLoadCmd(Header):
	def __init__(self, key):
		super(PsPsLoadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_PS_LOAD
		)
		self._key = key
	
	@property
	def key(self):
		return self._key
	
	def serialize(self):
		payload = pack('<' + 'H', int(self._key))
		self.payload_length = len(payload)
		return super(PsPsLoadCmd, self).serialize() + payload


class PsPsEraseCmd(Header):
	def __init__(self, key):
		super(PsPsEraseCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_PS_ERASE
		)
		self._key = key
	
	@property
	def key(self):
		return self._key
	
	def serialize(self):
		payload = pack('<' + 'H', int(self._key))
		self.payload_length = len(payload)
		return super(PsPsEraseCmd, self).serialize() + payload


class PsErasePageCmd(Header):
	def __init__(self, page):
		super(PsErasePageCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_ERASE_PAGE
		)
		self._page = page
	
	@property
	def page(self):
		return self._page
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._page))
		self.payload_length = len(payload)
		return super(PsErasePageCmd, self).serialize() + payload


class PsWriteWordsCmd(Header):
	def __init__(self, address, words):
		super(PsWriteWordsCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.PERSISTENT_STORE,
			message_id=PacketMessage.PS_WRITE_WORDS
		)
		self._address = address
		self._words = words
	
	@property
	def address(self):
		return self._address
	
	@property
	def words(self):
		return self._words
	
	def serialize(self):
		payload = pack('<' + 'H' + str(len(self._words) + 1) + 'B', int(self._address), len(self._words), *self._words)
		self.payload_length = len(payload)
		return super(PsWriteWordsCmd, self).serialize() + payload


class AdbWriteCmd(Header):
	def __init__(self, handle, offset, value):
		super(AdbWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_DATABASE,
			message_id=PacketMessage.ADB_WRITE
		)
		self._handle = handle
		self._offset = offset
		self._value = value
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def offset(self):
		return self._offset
	
	@property
	def value(self):
		return self._value
	
	def serialize(self):
		payload = pack('<' + 'H' + 'B' + str(len(self._value) + 1) + 'B', int(self._handle), int(self._offset), len(self._value), *self._value)
		self.payload_length = len(payload)
		return super(AdbWriteCmd, self).serialize() + payload


class AdbReadCmd(Header):
	def __init__(self, handle, offset):
		super(AdbReadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_DATABASE,
			message_id=PacketMessage.ADB_READ
		)
		self._handle = handle
		self._offset = offset
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def offset(self):
		return self._offset
	
	def serialize(self):
		payload = pack('<' + 'H' + 'H', int(self._handle), int(self._offset))
		self.payload_length = len(payload)
		return super(AdbReadCmd, self).serialize() + payload


class AdbReadTypeCmd(Header):
	def __init__(self, handle):
		super(AdbReadTypeCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_DATABASE,
			message_id=PacketMessage.ADB_READ_TYPE
		)
		self._handle = handle
	
	@property
	def handle(self):
		return self._handle
	
	def serialize(self):
		payload = pack('<' + 'H', int(self._handle))
		self.payload_length = len(payload)
		return super(AdbReadTypeCmd, self).serialize() + payload


class AdbUserReadResponseCmd(Header):
	def __init__(self, connection, att_error, value):
		super(AdbUserReadResponseCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_DATABASE,
			message_id=PacketMessage.ADB_USER_READ_RESPONSE
		)
		self._connection = connection
		self._att_error = att_error
		self._value = value
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def att_error(self):
		return self._att_error
	
	@property
	def value(self):
		return self._value
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + str(len(self._value) + 1) + 'B', int(self._connection), int(self._att_error), len(self._value), *self._value)
		self.payload_length = len(payload)
		return super(AdbUserReadResponseCmd, self).serialize() + payload


class AdbUserWriteResponseCmd(Header):
	def __init__(self, connection, att_error):
		super(AdbUserWriteResponseCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_DATABASE,
			message_id=PacketMessage.ADB_USER_WRITE_RESPONSE
		)
		self._connection = connection
		self._att_error = att_error
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def att_error(self):
		return self._att_error
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._connection), int(self._att_error))
		self.payload_length = len(payload)
		return super(AdbUserWriteResponseCmd, self).serialize() + payload


class ConnDisconnectCmd(Header):
	def __init__(self, connection):
		super(ConnDisconnectCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_DISCONNECT
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(ConnDisconnectCmd, self).serialize() + payload


class ConnGetRssiCmd(Header):
	def __init__(self, connection):
		super(ConnGetRssiCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_GET_RSSI
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(ConnGetRssiCmd, self).serialize() + payload


class ConnUpdateCmd(Header):
	def __init__(self, connection, interval_min, interval_max, latency, timeout):
		super(ConnUpdateCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_UPDATE
		)
		self._connection = connection
		self._interval_min = interval_min
		self._interval_max = interval_max
		self._latency = latency
		self._timeout = timeout
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def interval_min(self):
		return self._interval_min
	
	@property
	def interval_max(self):
		return self._interval_max
	
	@property
	def latency(self):
		return self._latency
	
	@property
	def timeout(self):
		return self._timeout
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + 'H' + 'H' + 'H', int(self._connection), int(self._interval_min), int(self._interval_max), int(self._latency), int(self._timeout))
		self.payload_length = len(payload)
		return super(ConnUpdateCmd, self).serialize() + payload


class ConnVersionUpdateCmd(Header):
	def __init__(self, connection):
		super(ConnVersionUpdateCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_VERSION_UPDATE
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(ConnVersionUpdateCmd, self).serialize() + payload


class ConnChannelMapGetCmd(Header):
	def __init__(self, connection):
		super(ConnChannelMapGetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_CHANNEL_MAP_GET
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(ConnChannelMapGetCmd, self).serialize() + payload


class ConnChannelMapSetCmd(Header):
	def __init__(self, connection, map):
		super(ConnChannelMapSetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_CHANNEL_MAP_SET
		)
		self._connection = connection
		self._map = map
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def map(self):
		return self._map
	
	def serialize(self):
		payload = pack('<' + 'B' + str(len(self._map) + 1) + 'B', int(self._connection), len(self._map), *self._map)
		self.payload_length = len(payload)
		return super(ConnChannelMapSetCmd, self).serialize() + payload


class ConnFeaturesGetCmd(Header):
	def __init__(self, connection):
		super(ConnFeaturesGetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_FEATURES_GET
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(ConnFeaturesGetCmd, self).serialize() + payload


class ConnGetStatusCmd(Header):
	def __init__(self, connection):
		super(ConnGetStatusCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_GET_STATUS
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(ConnGetStatusCmd, self).serialize() + payload


class ConnRawTxCmd(Header):
	def __init__(self, connection, data):
		super(ConnRawTxCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.CONNECTION,
			message_id=PacketMessage.CONN_RAW_TX
		)
		self._connection = connection
		self._data = data
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + str(len(self._data) + 1) + 'B', int(self._connection), len(self._data), *self._data)
		self.payload_length = len(payload)
		return super(ConnRawTxCmd, self).serialize() + payload


class AcFindByTypeValueCmd(Header):
	def __init__(self, connection, start, end, uuid, value):
		super(AcFindByTypeValueCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_FIND_BY_TYPE_VALUE
		)
		self._connection = connection
		self._start = start
		self._end = end
		self._uuid = uuid
		self._value = value
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def start(self):
		return self._start
	
	@property
	def end(self):
		return self._end
	
	@property
	def uuid(self):
		return self._uuid
	
	@property
	def value(self):
		return self._value
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + 'H' + 'H' + str(len(self._value) + 1) + 'B', int(self._connection), int(self._start), int(self._end), int(self._uuid), len(self._value), *self._value)
		self.payload_length = len(payload)
		return super(AcFindByTypeValueCmd, self).serialize() + payload


class AcReadByGroupTypeCmd(Header):
	def __init__(self, connection, start, end, uuid):
		super(AcReadByGroupTypeCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_READ_BY_GROUP_TYPE
		)
		self._connection = connection
		self._start = start
		self._end = end
		self._uuid = uuid
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def start(self):
		return self._start
	
	@property
	def end(self):
		return self._end
	
	@property
	def uuid(self):
		return self._uuid
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + 'H' + str(len(self._uuid) + 1) + 'B', int(self._connection), int(self._start), int(self._end), len(self._uuid), *[ord(x) for x in self._uuid])
		self.payload_length = len(payload)
		return super(AcReadByGroupTypeCmd, self).serialize() + payload


class AcReadByTypeCmd(Header):
	def __init__(self, connection, start, end, uuid):
		super(AcReadByTypeCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_READ_BY_TYPE
		)
		self._connection = connection
		self._start = start
		self._end = end
		self._uuid = uuid
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def start(self):
		return self._start
	
	@property
	def end(self):
		return self._end
	
	@property
	def uuid(self):
		return self._uuid
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + 'H' + str(len(self._uuid) + 1) + 'B', int(self._connection), int(self._start), int(self._end), len(self._uuid), *[ord(x) for x in self._uuid])
		self.payload_length = len(payload)
		return super(AcReadByTypeCmd, self).serialize() + payload


class AcFindInformationCmd(Header):
	def __init__(self, connection, start, end):
		super(AcFindInformationCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_FIND_INFORMATION
		)
		self._connection = connection
		self._start = start
		self._end = end
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def start(self):
		return self._start
	
	@property
	def end(self):
		return self._end
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + 'H', int(self._connection), int(self._start), int(self._end))
		self.payload_length = len(payload)
		return super(AcFindInformationCmd, self).serialize() + payload


class AcReadByHandleCmd(Header):
	def __init__(self, connection, chrhandle):
		super(AcReadByHandleCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_READ_BY_HANDLE
		)
		self._connection = connection
		self._chrhandle = chrhandle
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def chrhandle(self):
		return self._chrhandle
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H', int(self._connection), int(self._chrhandle))
		self.payload_length = len(payload)
		return super(AcReadByHandleCmd, self).serialize() + payload


class AcAttributeWriteCmd(Header):
	def __init__(self, connection, atthandle, data):
		super(AcAttributeWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_ATTRIBUTE_WRITE
		)
		self._connection = connection
		self._atthandle = atthandle
		self._data = data
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def atthandle(self):
		return self._atthandle
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + str(len(self._data) + 1) + 'B', int(self._connection), int(self._atthandle), len(self._data), *[ord(x) for x in self._data])
		self.payload_length = len(payload)
		return super(AcAttributeWriteCmd, self).serialize() + payload


class AcWriteCommandCmd(Header):
	def __init__(self, connection, atthandle, data):
		super(AcWriteCommandCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_WRITE_COMMAND
		)
		self._connection = connection
		self._atthandle = atthandle
		self._data = data
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def atthandle(self):
		return self._atthandle
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + str(len(self._data) + 1) + 'B', int(self._connection), int(self._atthandle), len(self._data), *[ord(x) for x in self._data])
		self.payload_length = len(payload)
		return super(AcWriteCommandCmd, self).serialize() + payload


class AcIndicateConfirmCmd(Header):
	def __init__(self, connection):
		super(AcIndicateConfirmCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_INDICATE_CONFIRM
		)
		self._connection = connection
	
	@property
	def connection(self):
		return self._connection
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._connection))
		self.payload_length = len(payload)
		return super(AcIndicateConfirmCmd, self).serialize() + payload


class AcReadLongCmd(Header):
	def __init__(self, connection, chrhandle):
		super(AcReadLongCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_READ_LONG
		)
		self._connection = connection
		self._chrhandle = chrhandle
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def chrhandle(self):
		return self._chrhandle
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H', int(self._connection), int(self._chrhandle))
		self.payload_length = len(payload)
		return super(AcReadLongCmd, self).serialize() + payload


class AcPrepareWriteCmd(Header):
	def __init__(self, connection, atthandle, offset, data):
		super(AcPrepareWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_PREPARE_WRITE
		)
		self._connection = connection
		self._atthandle = atthandle
		self._offset = offset
		self._data = data
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def atthandle(self):
		return self._atthandle
	
	@property
	def offset(self):
		return self._offset
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + 'H' + 'H' + str(len(self._data) + 1) + 'B', int(self._connection), int(self._atthandle), int(self._offset), len(self._data), *[ord(x) for x in self._data])
		self.payload_length = len(payload)
		return super(AcPrepareWriteCmd, self).serialize() + payload


class AcExecuteWriteCmd(Header):
	def __init__(self, connection, commit):
		super(AcExecuteWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_EXECUTE_WRITE
		)
		self._connection = connection
		self._commit = commit
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def commit(self):
		return self._commit
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._connection), int(self._commit))
		self.payload_length = len(payload)
		return super(AcExecuteWriteCmd, self).serialize() + payload


class AcReadMultipleCmd(Header):
	def __init__(self, connection, handles):
		super(AcReadMultipleCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.ATTRIBUTE_CLIENT,
			message_id=PacketMessage.AC_READ_MULTIPLE
		)
		self._connection = connection
		self._handles = handles
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def handles(self):
		return self._handles
	
	def serialize(self):
		payload = pack('<' + 'B' + str(len(self._handles) + 1) + 'B', int(self._connection), len(self._handles), *[ord(x) for x in self._handles])
		self.payload_length = len(payload)
		return super(AcReadMultipleCmd, self).serialize() + payload


class SmEncryptStartCmd(Header):
	def __init__(self, handle, bonding):
		super(SmEncryptStartCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_ENCRYPT_START
		)
		self._handle = handle
		self._bonding = bonding
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def bonding(self):
		return self._bonding
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._handle), int(self._bonding))
		self.payload_length = len(payload)
		return super(SmEncryptStartCmd, self).serialize() + payload


class SmSetBondableModeCmd(Header):
	def __init__(self, bondable):
		super(SmSetBondableModeCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_SET_BONDABLE_MODE
		)
		self._bondable = bondable
	
	@property
	def bondable(self):
		return self._bondable
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._bondable))
		self.payload_length = len(payload)
		return super(SmSetBondableModeCmd, self).serialize() + payload


class SmDeleteBondingCmd(Header):
	def __init__(self, handle):
		super(SmDeleteBondingCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_DELETE_BONDING
		)
		self._handle = handle
	
	@property
	def handle(self):
		return self._handle
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._handle))
		self.payload_length = len(payload)
		return super(SmDeleteBondingCmd, self).serialize() + payload


class SmSetParametersCmd(Header):
	def __init__(self, mitm, min_key_size, io_capabilities):
		super(SmSetParametersCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_SET_PARAMETERS
		)
		self._mitm = mitm
		self._min_key_size = min_key_size
		self._io_capabilities = io_capabilities
	
	@property
	def mitm(self):
		return self._mitm
	
	@property
	def min_key_size(self):
		return self._min_key_size
	
	@property
	def io_capabilities(self):
		return self._io_capabilities
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._mitm), int(self._min_key_size), int(self._io_capabilities))
		self.payload_length = len(payload)
		return super(SmSetParametersCmd, self).serialize() + payload


class SmPasskeyEntryCmd(Header):
	def __init__(self, handle, passkey):
		super(SmPasskeyEntryCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_PASSKEY_ENTRY
		)
		self._handle = handle
		self._passkey = passkey
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def passkey(self):
		return self._passkey
	
	def serialize(self):
		payload = pack('<' + 'B' + 'I', int(self._handle), int(self._passkey))
		self.payload_length = len(payload)
		return super(SmPasskeyEntryCmd, self).serialize() + payload


class SmGetBondsCmd(Header):
	def __init__(self):
		super(SmGetBondsCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_GET_BONDS
		)


class SmSetOobDataCmd(Header):
	def __init__(self, oob):
		super(SmSetOobDataCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.SECURITY_MANAGER,
			message_id=PacketMessage.SM_SET_OOB_DATA
		)
		self._oob = oob
	
	@property
	def oob(self):
		return self._oob
	
	def serialize(self):
		payload = pack('<' + str(len(self._oob) + 1) + 'B', len(self._oob), *[ord(x) for x in self._oob])
		self.payload_length = len(payload)
		return super(SmSetOobDataCmd, self).serialize() + payload


class GapSetPrivacyFlagsCmd(Header):
	def __init__(self, peripheral_privacy, central_privacy):
		super(GapSetPrivacyFlagsCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_PRIVACY_FLAGS
		)
		self._peripheral_privacy = peripheral_privacy
		self._central_privacy = central_privacy
	
	@property
	def peripheral_privacy(self):
		return self._peripheral_privacy
	
	@property
	def central_privacy(self):
		return self._central_privacy
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._peripheral_privacy), int(self._central_privacy))
		self.payload_length = len(payload)
		return super(GapSetPrivacyFlagsCmd, self).serialize() + payload


class GapSetModeCmd(Header):
	def __init__(self, discover, connect):
		super(GapSetModeCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_MODE
		)
		self._discover = discover
		self._connect = connect
	
	@property
	def discover(self):
		return self._discover
	
	@property
	def connect(self):
		return self._connect
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._discover), int(self._connect))
		self.payload_length = len(payload)
		return super(GapSetModeCmd, self).serialize() + payload


class GapDiscoverCmd(Header):
	def __init__(self, mode):
		super(GapDiscoverCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_DISCOVER
		)
		self._mode = mode
	
	@property
	def mode(self):
		return self._mode
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._mode))
		self.payload_length = len(payload)
		return super(GapDiscoverCmd, self).serialize() + payload


class GapConnectDirectCmd(Header):
	def __init__(self, address, addr_type, conn_interval_min, conn_interval_max, timeout, latency):
		super(GapConnectDirectCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_CONNECT_DIRECT
		)
		self._address = address
		self._addr_type = addr_type
		self._conn_interval_min = conn_interval_min
		self._conn_interval_max = conn_interval_max
		self._timeout = timeout
		self._latency = latency
	
	@property
	def address(self):
		return self._address
	
	@property
	def addr_type(self):
		return self._addr_type
	
	@property
	def conn_interval_min(self):
		return self._conn_interval_min
	
	@property
	def conn_interval_max(self):
		return self._conn_interval_max
	
	@property
	def timeout(self):
		return self._timeout
	
	@property
	def latency(self):
		return self._latency
	
	def serialize(self):
		payload = pack('<' + '6B', *self._address) + pack('<BHHHH', int(self._addr_type), int(self._conn_interval_min), int(self._conn_interval_max), int(self._timeout), int(self._latency))
		self.payload_length = len(payload)
		return super(GapConnectDirectCmd, self).serialize() + payload


class GapEndProcedureCmd(Header):
	def __init__(self):
		super(GapEndProcedureCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_END_PROCEDURE
		)


class GapConnectSelectiveCmd(Header):
	def __init__(self, conn_interval_min, conn_interval_max, timeout, latency):
		super(GapConnectSelectiveCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_CONNECT_SELECTIVE
		)
		self._conn_interval_min = conn_interval_min
		self._conn_interval_max = conn_interval_max
		self._timeout = timeout
		self._latency = latency
	
	@property
	def conn_interval_min(self):
		return self._conn_interval_min
	
	@property
	def conn_interval_max(self):
		return self._conn_interval_max
	
	@property
	def timeout(self):
		return self._timeout
	
	@property
	def latency(self):
		return self._latency
	
	def serialize(self):
		payload = pack('<' + 'H' + 'H' + 'H' + 'H', int(self._conn_interval_min), int(self._conn_interval_max), int(self._timeout), int(self._latency))
		self.payload_length = len(payload)
		return super(GapConnectSelectiveCmd, self).serialize() + payload


class GapSetFilteringCmd(Header):
	def __init__(self, scan_policy, adv_policy, scan_duplicate_filtering):
		super(GapSetFilteringCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_FILTERING
		)
		self._scan_policy = scan_policy
		self._adv_policy = adv_policy
		self._scan_duplicate_filtering = scan_duplicate_filtering
	
	@property
	def scan_policy(self):
		return self._scan_policy
	
	@property
	def adv_policy(self):
		return self._adv_policy
	
	@property
	def scan_duplicate_filtering(self):
		return self._scan_duplicate_filtering
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._scan_policy), int(self._adv_policy), int(self._scan_duplicate_filtering))
		self.payload_length = len(payload)
		return super(GapSetFilteringCmd, self).serialize() + payload


class GapSetScanParametersCmd(Header):
	def __init__(self, scan_interval, scan_window, active):
		super(GapSetScanParametersCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_SCAN_PARAMETERS
		)
		self._scan_interval = scan_interval
		self._scan_window = scan_window
		self._active = active
	
	@property
	def scan_interval(self):
		return self._scan_interval
	
	@property
	def scan_window(self):
		return self._scan_window
	
	@property
	def active(self):
		return self._active
	
	def serialize(self):
		payload = pack('<' + 'H' + 'H' + 'B', int(self._scan_interval), int(self._scan_window), int(self._active))
		self.payload_length = len(payload)
		return super(GapSetScanParametersCmd, self).serialize() + payload


class GapSetAdvParametersCmd(Header):
	def __init__(self, adv_interval_min, adv_interval_max, adv_channels):
		super(GapSetAdvParametersCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_ADV_PARAMETERS
		)
		self._adv_interval_min = adv_interval_min
		self._adv_interval_max = adv_interval_max
		self._adv_channels = adv_channels
	
	@property
	def adv_interval_min(self):
		return self._adv_interval_min
	
	@property
	def adv_interval_max(self):
		return self._adv_interval_max
	
	@property
	def adv_channels(self):
		return self._adv_channels
	
	def serialize(self):
		payload = pack('<' + 'H' + 'H' + 'B', int(self._adv_interval_min), int(self._adv_interval_max), int(self._adv_channels))
		self.payload_length = len(payload)
		return super(GapSetAdvParametersCmd, self).serialize() + payload


class GapSetAdvDataCmd(Header):
	def __init__(self, set_scanrsp, adv_data):
		super(GapSetAdvDataCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_ADV_DATA
		)
		self._set_scanrsp = set_scanrsp
		self._adv_data = adv_data
	
	@property
	def set_scanrsp(self):
		return self._set_scanrsp
	
	@property
	def adv_data(self):
		return self._adv_data
	
	def serialize(self):
		payload = pack('<' + 'B' + str(len(self._adv_data) + 1) + 'B', int(self._set_scanrsp), len(self._adv_data), *[ord(x) for x in self._adv_data])
		self.payload_length = len(payload)
		return super(GapSetAdvDataCmd, self).serialize() + payload


class GapSetDirectedConnectableModeCmd(Header):
	def __init__(self, address, addr_type):
		super(GapSetDirectedConnectableModeCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.GENERIC_ACCESS_PROFILE,
			message_id=PacketMessage.GAP_SET_DIRECTED_CONNECTABLE_MODE
		)
		self._address = address
		self._addr_type = addr_type
	
	@property
	def address(self):
		return self._address
	
	@property
	def addr_type(self):
		return self._addr_type
	
	def serialize(self):
		payload = pack('<' + '6B', *self._address) + pack('<B', int(self._addr_type))
		self.payload_length = len(payload)
		return super(GapSetDirectedConnectableModeCmd, self).serialize() + payload


class HwIoPortConfigIrqCmd(Header):
	def __init__(self, port, enable_bits, falling_edge):
		super(HwIoPortConfigIrqCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_IO_PORT_CONFIG_IRQ
		)
		self._port = port
		self._enable_bits = enable_bits
		self._falling_edge = falling_edge
	
	@property
	def port(self):
		return self._port
	
	@property
	def enable_bits(self):
		return self._enable_bits
	
	@property
	def falling_edge(self):
		return self._falling_edge
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._port), int(self._enable_bits), int(self._falling_edge))
		self.payload_length = len(payload)
		return super(HwIoPortConfigIrqCmd, self).serialize() + payload


class HwSetSoftTimerCmd(Header):
	def __init__(self, time, handle, single_shot):
		super(HwSetSoftTimerCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_SET_SOFT_TIMER
		)
		self._time = time
		self._handle = handle
		self._single_shot = single_shot
	
	@property
	def time(self):
		return self._time
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def single_shot(self):
		return self._single_shot
	
	def serialize(self):
		payload = pack('<' + 'I' + 'B' + 'B', int(self._time), int(self._handle), int(self._single_shot))
		self.payload_length = len(payload)
		return super(HwSetSoftTimerCmd, self).serialize() + payload


class HwAdcReadCmd(Header):
	def __init__(self, input, decimation, reference_selection):
		super(HwAdcReadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_ADC_READ
		)
		self._input = input
		self._decimation = decimation
		self._reference_selection = reference_selection
	
	@property
	def input(self):
		return self._input
	
	@property
	def decimation(self):
		return self._decimation
	
	@property
	def reference_selection(self):
		return self._reference_selection
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._input), int(self._decimation), int(self._reference_selection))
		self.payload_length = len(payload)
		return super(HwAdcReadCmd, self).serialize() + payload


class HwIoPortConfigDirectionCmd(Header):
	def __init__(self, port, direction):
		super(HwIoPortConfigDirectionCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_IO_PORT_CONFIG_DIRECTION
		)
		self._port = port
		self._direction = direction
	
	@property
	def port(self):
		return self._port
	
	@property
	def direction(self):
		return self._direction
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._port), int(self._direction))
		self.payload_length = len(payload)
		return super(HwIoPortConfigDirectionCmd, self).serialize() + payload


class HwIoPortConfigFunctionCmd(Header):
	def __init__(self, port, function):
		super(HwIoPortConfigFunctionCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_IO_PORT_CONFIG_FUNCTION
		)
		self._port = port
		self._function = function
	
	@property
	def port(self):
		return self._port
	
	@property
	def function(self):
		return self._function
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._port), int(self._function))
		self.payload_length = len(payload)
		return super(HwIoPortConfigFunctionCmd, self).serialize() + payload


class HwIoPortConfigPullCmd(Header):
	def __init__(self, port, tristate_mask, pull_up):
		super(HwIoPortConfigPullCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_IO_PORT_CONFIG_PULL
		)
		self._port = port
		self._tristate_mask = tristate_mask
		self._pull_up = pull_up
	
	@property
	def port(self):
		return self._port
	
	@property
	def tristate_mask(self):
		return self._tristate_mask
	
	@property
	def pull_up(self):
		return self._pull_up
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._port), int(self._tristate_mask), int(self._pull_up))
		self.payload_length = len(payload)
		return super(HwIoPortConfigPullCmd, self).serialize() + payload


class HwIoPortWriteCmd(Header):
	def __init__(self, port, mask, data):
		super(HwIoPortWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_IO_PORT_WRITE
		)
		self._port = port
		self._mask = mask
		self._data = data
	
	@property
	def port(self):
		return self._port
	
	@property
	def mask(self):
		return self._mask
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._port), int(self._mask), int(self._data))
		self.payload_length = len(payload)
		return super(HwIoPortWriteCmd, self).serialize() + payload


class HwIoPortReadCmd(Header):
	def __init__(self, port, mask):
		super(HwIoPortReadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_IO_PORT_READ
		)
		self._port = port
		self._mask = mask
	
	@property
	def port(self):
		return self._port
	
	@property
	def mask(self):
		return self._mask
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B', int(self._port), int(self._mask))
		self.payload_length = len(payload)
		return super(HwIoPortReadCmd, self).serialize() + payload


class HwSpiConfigCmd(Header):
	def __init__(self, channel, polarity, phase, bit_order, baud_e, baud_m):
		super(HwSpiConfigCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_SPI_CONFIG
		)
		self._channel = channel
		self._polarity = polarity
		self._phase = phase
		self._bit_order = bit_order
		self._baud_e = baud_e
		self._baud_m = baud_m
	
	@property
	def channel(self):
		return self._channel
	
	@property
	def polarity(self):
		return self._polarity
	
	@property
	def phase(self):
		return self._phase
	
	@property
	def bit_order(self):
		return self._bit_order
	
	@property
	def baud_e(self):
		return self._baud_e
	
	@property
	def baud_m(self):
		return self._baud_m
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B' + 'B' + 'B' + 'B', int(self._channel), int(self._polarity), int(self._phase), int(self._bit_order), int(self._baud_e), int(self._baud_m))
		self.payload_length = len(payload)
		return super(HwSpiConfigCmd, self).serialize() + payload


class HwSpiTransferCmd(Header):
	def __init__(self, channel, data):
		super(HwSpiTransferCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_SPI_TRANSFER
		)
		self._channel = channel
		self._data = data
	
	@property
	def channel(self):
		return self._channel
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + str(len(self._data) + 1) + 'B', int(self._channel), len(self._data), *[ord(x) for x in self._data])
		self.payload_length = len(payload)
		return super(HwSpiTransferCmd, self).serialize() + payload


class HwI2CReadCmd(Header):
	def __init__(self, address, stop, length):
		super(HwI2CReadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_I2C_READ
		)
		self._address = address
		self._stop = stop
		self._length = length
	
	@property
	def address(self):
		return self._address
	
	@property
	def stop(self):
		return self._stop
	
	@property
	def length(self):
		return self._length
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._address), int(self._stop), int(self._length))
		self.payload_length = len(payload)
		return super(HwI2CReadCmd, self).serialize() + payload


class HwI2CWriteCmd(Header):
	def __init__(self, address, stop, data):
		super(HwI2CWriteCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_I2C_WRITE
		)
		self._address = address
		self._stop = stop
		self._data = data
	
	@property
	def address(self):
		return self._address
	
	@property
	def stop(self):
		return self._stop
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + str(len(self._data) + 1) + 'B', int(self._address), int(self._stop), len(self._data), *[ord(x) for x in self._data])
		self.payload_length = len(payload)
		return super(HwI2CWriteCmd, self).serialize() + payload


class HwSetTxpowerCmd(Header):
	def __init__(self, power):
		super(HwSetTxpowerCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_SET_TXPOWER
		)
		self._power = power
	
	@property
	def power(self):
		return self._power
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._power))
		self.payload_length = len(payload)
		return super(HwSetTxpowerCmd, self).serialize() + payload


class HwTimerComparatorCmd(Header):
	def __init__(self, timer, channel, mode, comparator_value):
		super(HwTimerComparatorCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.HARDWARE,
			message_id=PacketMessage.HW_TIMER_COMPARATOR
		)
		self._timer = timer
		self._channel = channel
		self._mode = mode
		self._comparator_value = comparator_value
	
	@property
	def timer(self):
		return self._timer
	
	@property
	def channel(self):
		return self._channel
	
	@property
	def mode(self):
		return self._mode
	
	@property
	def comparator_value(self):
		return self._comparator_value
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B' + 'H', int(self._timer), int(self._channel), int(self._mode), int(self._comparator_value))
		self.payload_length = len(payload)
		return super(HwTimerComparatorCmd, self).serialize() + payload


class TestPhyTxCmd(Header):
	def __init__(self, channel, length, type):
		super(TestPhyTxCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.TESTING,
			message_id=PacketMessage.TEST_PHY_TX
		)
		self._channel = channel
		self._length = length
		self._type = type
	
	@property
	def channel(self):
		return self._channel
	
	@property
	def length(self):
		return self._length
	
	@property
	def type(self):
		return self._type
	
	def serialize(self):
		payload = pack('<' + 'B' + 'B' + 'B', int(self._channel), int(self._length), int(self._type))
		self.payload_length = len(payload)
		return super(TestPhyTxCmd, self).serialize() + payload


class TestPhyRxCmd(Header):
	def __init__(self, channel):
		super(TestPhyRxCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.TESTING,
			message_id=PacketMessage.TEST_PHY_RX
		)
		self._channel = channel
	
	@property
	def channel(self):
		return self._channel
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._channel))
		self.payload_length = len(payload)
		return super(TestPhyRxCmd, self).serialize() + payload


class TestPhyEndCmd(Header):
	def __init__(self):
		super(TestPhyEndCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.TESTING,
			message_id=PacketMessage.TEST_PHY_END
		)


class TestPhyResetCmd(Header):
	def __init__(self):
		super(TestPhyResetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.TESTING,
			message_id=PacketMessage.TEST_PHY_RESET
		)


class TestGetChannelMapCmd(Header):
	def __init__(self):
		super(TestGetChannelMapCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.TESTING,
			message_id=PacketMessage.TEST_GET_CHANNEL_MAP
		)


class TestDebugCmd(Header):
	def __init__(self, input):
		super(TestDebugCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.TESTING,
			message_id=PacketMessage.TEST_DEBUG
		)
		self._input = input
	
	@property
	def input(self):
		return self._input
	
	def serialize(self):
		payload = pack('<' + str(len(self._input) + 1) + 'B', len(self._input), *[ord(x) for x in self._input])
		self.payload_length = len(payload)
		return super(TestDebugCmd, self).serialize() + payload


class DfuResetCmd(Header):
	def __init__(self, dfu):
		super(DfuResetCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.DEVICE_FIRMWARE_UPGRADE,
			message_id=PacketMessage.DFU_RESET
		)
		self._dfu = dfu
	
	@property
	def dfu(self):
		return self._dfu
	
	def serialize(self):
		payload = pack('<' + 'B', int(self._dfu))
		self.payload_length = len(payload)
		return super(DfuResetCmd, self).serialize() + payload


class DfuFlashSetAddressCmd(Header):
	def __init__(self, address):
		super(DfuFlashSetAddressCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.DEVICE_FIRMWARE_UPGRADE,
			message_id=PacketMessage.DFU_FLASH_SET_ADDRESS
		)
		self._address = address
	
	@property
	def address(self):
		return self._address
	
	def serialize(self):
		payload = pack('<' + 'I', int(self._address))
		self.payload_length = len(payload)
		return super(DfuFlashSetAddressCmd, self).serialize() + payload


class DfuFlashUploadCmd(Header):
	def __init__(self, data):
		super(DfuFlashUploadCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.DEVICE_FIRMWARE_UPGRADE,
			message_id=PacketMessage.DFU_FLASH_UPLOAD
		)
		self._data = data
	
	@property
	def data(self):
		return self._data
	
	def serialize(self):
		payload = pack('<' + str(len(self._data) + 1) + 'B', len(self._data), *[ord(x) for x in self._data])
		self.payload_length = len(payload)
		return super(DfuFlashUploadCmd, self).serialize() + payload


class DfuFlashUploadFinishCmd(Header):
	def __init__(self):
		super(DfuFlashUploadFinishCmd, self).__init__(
			type_id=PacketType.COMMAND,
			payload_length=0,
			class_id=PacketClass.DEVICE_FIRMWARE_UPGRADE,
			message_id=PacketMessage.DFU_FLASH_UPLOAD_FINISH
		)


# Response subclasses
class SysHelloRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_HELLO))
	
	def __init__(self, init_data):
		super(SysHelloRes, self).__init__(init_data[:4])


class SysAddressGetRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ADDRESS_GET))
	
	def __init__(self, init_data):
		super(SysAddressGetRes, self).__init__(init_data[:4])
		self._address = unpack('<6B', init_data[4:10])
	
	@property
	def address(self):
		return self._address


class SysRegWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_REG_WRITE))
	
	def __init__(self, init_data):
		super(SysRegWriteRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class SysRegReadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_REG_READ))
	
	def __init__(self, init_data):
		super(SysRegReadRes, self).__init__(init_data[:4])
		self._address = unpack('<H', init_data[4:6])[0]
		self._value = unpack('<B', init_data[6:7])[0]
	
	@property
	def address(self):
		return self._address
	
	@property
	def value(self):
		return self._value


class SysGetCountersRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_GET_COUNTERS))
	
	def __init__(self, init_data):
		super(SysGetCountersRes, self).__init__(init_data[:4])
		self._txok = unpack('<B', init_data[4:5])[0]
		self._txretry = unpack('<B', init_data[5:6])[0]
		self._rxok = unpack('<B', init_data[6:7])[0]
		self._rxfail = unpack('<B', init_data[7:8])[0]
		self._mbuf = unpack('<B', init_data[8:9])[0]
	
	@property
	def txok(self):
		return self._txok
	
	@property
	def txretry(self):
		return self._txretry
	
	@property
	def rxok(self):
		return self._rxok
	
	@property
	def rxfail(self):
		return self._rxfail
	
	@property
	def mbuf(self):
		return self._mbuf


class SysGetConnectionsRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_GET_CONNECTIONS))
	
	def __init__(self, init_data):
		super(SysGetConnectionsRes, self).__init__(init_data[:4])
		self._maxconn = unpack('<B', init_data[4:5])[0]
	
	@property
	def maxconn(self):
		return self._maxconn


class SysReadMemoryRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_READ_MEMORY))
	
	def __init__(self, init_data):
		super(SysReadMemoryRes, self).__init__(init_data[:4])
		self._address = unpack('<I', init_data[4:8])[0]
		self._data = init_data[9:]
	
	@property
	def address(self):
		return self._address
	
	@property
	def data(self):
		return self._data


class SysGetInfoRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_GET_INFO))
	
	def __init__(self, init_data):
		super(SysGetInfoRes, self).__init__(init_data[:4])
		self._version_major = unpack('<H', init_data[4:6])[0]
		self._version_minor = unpack('<H', init_data[6:8])[0]
		self._version_patch = unpack('<H', init_data[8:10])[0]
		self._version_build = unpack('<H', init_data[10:12])[0]
		self._ll_version = unpack('<H', init_data[12:14])[0]
		self._protocol_version = unpack('<B', init_data[14:15])[0]
		self._hw = unpack('<B', init_data[15:16])[0]
	
	@property
	def version_major(self):
		return self._version_major
	
	@property
	def version_minor(self):
		return self._version_minor
	
	@property
	def version_patch(self):
		return self._version_patch
	
	@property
	def version_build(self):
		return self._version_build
	
	@property
	def ll_version(self):
		return self._ll_version
	
	@property
	def protocol_version(self):
		return self._protocol_version
	
	@property
	def hw(self):
		return self._hw


class SysEndpointTxRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_TX))
	
	def __init__(self, init_data):
		super(SysEndpointTxRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class SysWhitelistAppendRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_WHITELIST_APPEND))
	
	def __init__(self, init_data):
		super(SysWhitelistAppendRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class SysWhitelistRemoveRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_WHITELIST_REMOVE))
	
	def __init__(self, init_data):
		super(SysWhitelistRemoveRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class SysWhitelistClearRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_WHITELIST_CLEAR))
	
	def __init__(self, init_data):
		super(SysWhitelistClearRes, self).__init__(init_data[:4])


class SysEndpointRxRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_RX))
	
	def __init__(self, init_data):
		super(SysEndpointRxRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._data = init_data[7:]
	
	@property
	def result(self):
		return self._result
	
	@property
	def data(self):
		return self._data


class SysEndpointSetWatermarksRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_SET_WATERMARKS))
	
	def __init__(self, init_data):
		super(SysEndpointSetWatermarksRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class PsPsDefragRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_DEFRAG))
	
	def __init__(self, init_data):
		super(PsPsDefragRes, self).__init__(init_data[:4])


class PsPsDumpRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_DUMP))
	
	def __init__(self, init_data):
		super(PsPsDumpRes, self).__init__(init_data[:4])


class PsPsEraseAllRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_ERASE_ALL))
	
	def __init__(self, init_data):
		super(PsPsEraseAllRes, self).__init__(init_data[:4])


class PsPsSaveRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_SAVE))
	
	def __init__(self, init_data):
		super(PsPsSaveRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class PsPsLoadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_LOAD))
	
	def __init__(self, init_data):
		super(PsPsLoadRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._value = init_data[7:]
	
	@property
	def result(self):
		return self._result
	
	@property
	def value(self):
		return self._value


class PsPsEraseRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_ERASE))
	
	def __init__(self, init_data):
		super(PsPsEraseRes, self).__init__(init_data[:4])


class PsErasePageRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_ERASE_PAGE))
	
	def __init__(self, init_data):
		super(PsErasePageRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class PsWriteWordsRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_WRITE_WORDS))
	
	def __init__(self, init_data):
		super(PsWriteWordsRes, self).__init__(init_data[:4])


class AdbWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_WRITE))
	
	def __init__(self, init_data):
		super(AdbWriteRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class AdbReadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_READ))
	
	def __init__(self, init_data):
		super(AdbReadRes, self).__init__(init_data[:4])
		self._handle = unpack('<H', init_data[4:6])[0]
		self._offset = unpack('<H', init_data[6:8])[0]
		self._result = unpack('<H', init_data[8:10])[0]
		self._value = init_data[11:]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def offset(self):
		return self._offset
	
	@property
	def result(self):
		return self._result
	
	@property
	def value(self):
		return self._value


class AdbReadTypeRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_READ_TYPE))
	
	def __init__(self, init_data):
		super(AdbReadTypeRes, self).__init__(init_data[:4])
		self._handle = unpack('<H', init_data[4:6])[0]
		self._result = unpack('<H', init_data[6:8])[0]
		self._value = init_data[9:]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def result(self):
		return self._result
	
	@property
	def value(self):
		return self._value


class AdbUserReadResponseRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_USER_READ_RESPONSE))
	
	def __init__(self, init_data):
		super(AdbUserReadResponseRes, self).__init__(init_data[:4])


class AdbUserWriteResponseRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_USER_WRITE_RESPONSE))
	
	def __init__(self, init_data):
		super(AdbUserWriteResponseRes, self).__init__(init_data[:4])


class ConnDisconnectRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_DISCONNECT))
	
	def __init__(self, init_data):
		super(ConnDisconnectRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class ConnGetRssiRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_GET_RSSI))
	
	def __init__(self, init_data):
		super(ConnGetRssiRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._rssi = unpack('<b', init_data[5:6])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def rssi(self):
		return self._rssi


class ConnUpdateRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_UPDATE))
	
	def __init__(self, init_data):
		super(ConnUpdateRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class ConnVersionUpdateRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_VERSION_UPDATE))
	
	def __init__(self, init_data):
		super(ConnVersionUpdateRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class ConnChannelMapGetRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_CHANNEL_MAP_GET))
	
	def __init__(self, init_data):
		super(ConnChannelMapGetRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._map = init_data[6:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def map(self):
		return self._map


class ConnChannelMapSetRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_CHANNEL_MAP_SET))
	
	def __init__(self, init_data):
		super(ConnChannelMapSetRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class ConnFeaturesGetRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_FEATURES_GET))
	
	def __init__(self, init_data):
		super(ConnFeaturesGetRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class ConnGetStatusRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_GET_STATUS))
	
	def __init__(self, init_data):
		super(ConnGetStatusRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
	
	@property
	def connection(self):
		return self._connection


class ConnRawTxRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_RAW_TX))
	
	def __init__(self, init_data):
		super(ConnRawTxRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
	
	@property
	def connection(self):
		return self._connection


class AcFindByTypeValueRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_FIND_BY_TYPE_VALUE))
	
	def __init__(self, init_data):
		super(AcFindByTypeValueRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcReadByGroupTypeRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_BY_GROUP_TYPE))
	
	def __init__(self, init_data):
		super(AcReadByGroupTypeRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcReadByTypeRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_BY_TYPE))
	
	def __init__(self, init_data):
		super(AcReadByTypeRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcFindInformationRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_FIND_INFORMATION))
	
	def __init__(self, init_data):
		super(AcFindInformationRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcReadByHandleRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_BY_HANDLE))
	
	def __init__(self, init_data):
		super(AcReadByHandleRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcAttributeWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_ATTRIBUTE_WRITE))
	
	def __init__(self, init_data):
		super(AcAttributeWriteRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcWriteCommandRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_WRITE_COMMAND))
	
	def __init__(self, init_data):
		super(AcWriteCommandRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcIndicateConfirmRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_INDICATE_CONFIRM))
	
	def __init__(self, init_data):
		super(AcIndicateConfirmRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class AcReadLongRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_LONG))
	
	def __init__(self, init_data):
		super(AcReadLongRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcPrepareWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_PREPARE_WRITE))
	
	def __init__(self, init_data):
		super(AcPrepareWriteRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcExecuteWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_EXECUTE_WRITE))
	
	def __init__(self, init_data):
		super(AcExecuteWriteRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class AcReadMultipleRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_MULTIPLE))
	
	def __init__(self, init_data):
		super(AcReadMultipleRes, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result


class SmEncryptStartRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_ENCRYPT_START))
	
	def __init__(self, init_data):
		super(SmEncryptStartRes, self).__init__(init_data[:4])
		self._handle = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def result(self):
		return self._result


class SmSetBondableModeRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SET_BONDABLE_MODE))
	
	def __init__(self, init_data):
		super(SmSetBondableModeRes, self).__init__(init_data[:4])


class SmDeleteBondingRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_DELETE_BONDING))
	
	def __init__(self, init_data):
		super(SmDeleteBondingRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class SmSetParametersRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SET_PARAMETERS))
	
	def __init__(self, init_data):
		super(SmSetParametersRes, self).__init__(init_data[:4])


class SmPasskeyEntryRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_PASSKEY_ENTRY))
	
	def __init__(self, init_data):
		super(SmPasskeyEntryRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class SmGetBondsRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_GET_BONDS))
	
	def __init__(self, init_data):
		super(SmGetBondsRes, self).__init__(init_data[:4])
		self._bonds = unpack('<B', init_data[4:5])[0]
	
	@property
	def bonds(self):
		return self._bonds


class SmSetOobDataRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SET_OOB_DATA))
	
	def __init__(self, init_data):
		super(SmSetOobDataRes, self).__init__(init_data[:4])


class GapSetPrivacyFlagsRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_PRIVACY_FLAGS))
	
	def __init__(self, init_data):
		super(GapSetPrivacyFlagsRes, self).__init__(init_data[:4])


class GapSetModeRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_MODE))
	
	def __init__(self, init_data):
		super(GapSetModeRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapDiscoverRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_DISCOVER))
	
	def __init__(self, init_data):
		super(GapDiscoverRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapConnectDirectRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_CONNECT_DIRECT))
	
	def __init__(self, init_data):
		super(GapConnectDirectRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._connection_handle = unpack('<B', init_data[6:7])[0]
	
	@property
	def result(self):
		return self._result
	
	@property
	def connection_handle(self):
		return self._connection_handle


class GapEndProcedureRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_END_PROCEDURE))
	
	def __init__(self, init_data):
		super(GapEndProcedureRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapConnectSelectiveRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_CONNECT_SELECTIVE))
	
	def __init__(self, init_data):
		super(GapConnectSelectiveRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._connection_handle = unpack('<B', init_data[6:7])[0]
	
	@property
	def result(self):
		return self._result
	
	@property
	def connection_handle(self):
		return self._connection_handle


class GapSetFilteringRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_FILTERING))
	
	def __init__(self, init_data):
		super(GapSetFilteringRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapSetScanParametersRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_SCAN_PARAMETERS))
	
	def __init__(self, init_data):
		super(GapSetScanParametersRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapSetAdvParametersRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_ADV_PARAMETERS))
	
	def __init__(self, init_data):
		super(GapSetAdvParametersRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapSetAdvDataRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_ADV_DATA))
	
	def __init__(self, init_data):
		super(GapSetAdvDataRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class GapSetDirectedConnectableModeRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_DIRECTED_CONNECTABLE_MODE))
	
	def __init__(self, init_data):
		super(GapSetDirectedConnectableModeRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwIoPortConfigIrqRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_IRQ))
	
	def __init__(self, init_data):
		super(HwIoPortConfigIrqRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwSetSoftTimerRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SET_SOFT_TIMER))
	
	def __init__(self, init_data):
		super(HwSetSoftTimerRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwAdcReadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_ADC_READ))
	
	def __init__(self, init_data):
		super(HwAdcReadRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwIoPortConfigDirectionRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_DIRECTION))
	
	def __init__(self, init_data):
		super(HwIoPortConfigDirectionRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwIoPortConfigFunctionRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_FUNCTION))
	
	def __init__(self, init_data):
		super(HwIoPortConfigFunctionRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwIoPortConfigPullRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_PULL))
	
	def __init__(self, init_data):
		super(HwIoPortConfigPullRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwIoPortWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_WRITE))
	
	def __init__(self, init_data):
		super(HwIoPortWriteRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwIoPortReadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_READ))
	
	def __init__(self, init_data):
		super(HwIoPortReadRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._port = unpack('<B', init_data[6:7])[0]
		self._data = unpack('<B', init_data[7:8])[0]
	
	@property
	def result(self):
		return self._result
	
	@property
	def port(self):
		return self._port
	
	@property
	def data(self):
		return self._data


class HwSpiConfigRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SPI_CONFIG))
	
	def __init__(self, init_data):
		super(HwSpiConfigRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class HwSpiTransferRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SPI_TRANSFER))
	
	def __init__(self, init_data):
		super(HwSpiTransferRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._channel = unpack('<B', init_data[6:7])[0]
		self._data = init_data[8:]
	
	@property
	def result(self):
		return self._result
	
	@property
	def channel(self):
		return self._channel
	
	@property
	def data(self):
		return self._data


class HwI2CReadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_I2C_READ))
	
	def __init__(self, init_data):
		super(HwI2CReadRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
		self._data = init_data[7:]
	
	@property
	def result(self):
		return self._result
	
	@property
	def data(self):
		return self._data


class HwI2CWriteRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_I2C_WRITE))
	
	def __init__(self, init_data):
		super(HwI2CWriteRes, self).__init__(init_data[:4])
		self._written = unpack('<B', init_data[4:5])[0]
	
	@property
	def written(self):
		return self._written


class HwSetTxpowerRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SET_TXPOWER))
	
	def __init__(self, init_data):
		super(HwSetTxpowerRes, self).__init__(init_data[:4])


class HwTimerComparatorRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_TIMER_COMPARATOR))
	
	def __init__(self, init_data):
		super(HwTimerComparatorRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class TestPhyTxRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_TX))
	
	def __init__(self, init_data):
		super(TestPhyTxRes, self).__init__(init_data[:4])


class TestPhyRxRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_RX))
	
	def __init__(self, init_data):
		super(TestPhyRxRes, self).__init__(init_data[:4])


class TestPhyEndRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_END))
	
	def __init__(self, init_data):
		super(TestPhyEndRes, self).__init__(init_data[:4])
		self._counter = unpack('<H', init_data[4:6])[0]
	
	@property
	def counter(self):
		return self._counter


class TestPhyResetRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_RESET))
	
	def __init__(self, init_data):
		super(TestPhyResetRes, self).__init__(init_data[:4])


class TestGetChannelMapRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_GET_CHANNEL_MAP))
	
	def __init__(self, init_data):
		super(TestGetChannelMapRes, self).__init__(init_data[:4])
		self._channel_map = init_data[5:]
	
	@property
	def channel_map(self):
		return self._channel_map


class TestDebugRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_DEBUG))
	
	def __init__(self, init_data):
		super(TestDebugRes, self).__init__(init_data[:4])
		self._output = init_data[5:]
	
	@property
	def output(self):
		return self._output


class DfuFlashSetAddressRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_FLASH_SET_ADDRESS))
	
	def __init__(self, init_data):
		super(DfuFlashSetAddressRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class DfuFlashUploadRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_FLASH_UPLOAD))
	
	def __init__(self, init_data):
		super(DfuFlashUploadRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


class DfuFlashUploadFinishRes(Header):
	index = PacketIndex(int(PacketType.RESPONSE), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_FLASH_UPLOAD_FINISH))
	
	def __init__(self, init_data):
		super(DfuFlashUploadFinishRes, self).__init__(init_data[:4])
		self._result = unpack('<H', init_data[4:6])[0]
	
	@property
	def result(self):
		return self._result


# Event subclasses
class SysBootEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_BOOT))
	
	def __init__(self, init_data):
		super(SysBootEvt, self).__init__(init_data[:4])
		self._version_major = unpack('<H', init_data[4:6])[0]
		self._version_minor = unpack('<H', init_data[6:8])[0]
		self._version_patch = unpack('<H', init_data[8:10])[0]
		self._version_build = unpack('<H', init_data[10:12])[0]
		self._ll_version = unpack('<H', init_data[12:14])[0]
		self._protocol_version = unpack('<B', init_data[14:15])[0]
		self._hw = unpack('<B', init_data[15:16])[0]
	
	@property
	def version_major(self):
		return self._version_major
	
	@property
	def version_minor(self):
		return self._version_minor
	
	@property
	def version_patch(self):
		return self._version_patch
	
	@property
	def version_build(self):
		return self._version_build
	
	@property
	def ll_version(self):
		return self._ll_version
	
	@property
	def protocol_version(self):
		return self._protocol_version
	
	@property
	def hw(self):
		return self._hw


class SysDebugEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_DEBUG))
	
	def __init__(self, init_data):
		super(SysDebugEvt, self).__init__(init_data[:4])
		self._data = init_data[5:]
	
	@property
	def data(self):
		return self._data


class SysEndpointWatermarkRxEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_WATERMARK_RX))
	
	def __init__(self, init_data):
		super(SysEndpointWatermarkRxEvt, self).__init__(init_data[:4])
		self._endpoint = unpack('<B', init_data[4:5])[0]
		self._data = unpack('<B', init_data[5:6])[0]
	
	@property
	def endpoint(self):
		return self._endpoint
	
	@property
	def data(self):
		return self._data


class SysEndpointWatermarkTxEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_WATERMARK_TX))
	
	def __init__(self, init_data):
		super(SysEndpointWatermarkTxEvt, self).__init__(init_data[:4])
		self._endpoint = unpack('<B', init_data[4:5])[0]
		self._data = unpack('<B', init_data[5:6])[0]
	
	@property
	def endpoint(self):
		return self._endpoint
	
	@property
	def data(self):
		return self._data


class SysScriptFailureEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_SCRIPT_FAILURE))
	
	def __init__(self, init_data):
		super(SysScriptFailureEvt, self).__init__(init_data[:4])
		self._address = unpack('<H', init_data[4:6])[0]
		self._reason = unpack('<H', init_data[6:8])[0]
	
	@property
	def address(self):
		return self._address
	
	@property
	def reason(self):
		return self._reason


class SysNoLicenseKeyEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_NO_LICENSE_KEY))
	
	def __init__(self, init_data):
		super(SysNoLicenseKeyEvt, self).__init__(init_data[:4])


class SysProtocolErrorEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_PROTOCOL_ERROR))
	
	def __init__(self, init_data):
		super(SysProtocolErrorEvt, self).__init__(init_data[:4])
		self._reason = unpack('<H', init_data[4:6])[0]
	
	@property
	def reason(self):
		return self._reason


class PsPsKeyEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_KEY))
	
	def __init__(self, init_data):
		super(PsPsKeyEvt, self).__init__(init_data[:4])
		self._key = unpack('<H', init_data[4:6])[0]
		self._value = init_data[7:]
	
	@property
	def key(self):
		return self._key
	
	@property
	def value(self):
		return self._value


class AdbValueEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_VALUE))
	
	def __init__(self, init_data):
		super(AdbValueEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._reason = unpack('<B', init_data[5:6])[0]
		self._handle = unpack('<H', init_data[6:8])[0]
		self._offset = unpack('<H', init_data[8:10])[0]
		self._value = init_data[11:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def reason(self):
		return self._reason
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def offset(self):
		return self._offset
	
	@property
	def value(self):
		return self._value


class AdbUserReadRequestEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_USER_READ_REQUEST))
	
	def __init__(self, init_data):
		super(AdbUserReadRequestEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._handle = unpack('<H', init_data[5:7])[0]
		self._offset = unpack('<H', init_data[7:9])[0]
		self._maxsize = unpack('<B', init_data[9:10])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def offset(self):
		return self._offset
	
	@property
	def maxsize(self):
		return self._maxsize


class AdbStatusEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_STATUS))
	
	def __init__(self, init_data):
		super(AdbStatusEvt, self).__init__(init_data[:4])
		self._handle = unpack('<H', init_data[4:6])[0]
		self._flags = unpack('<B', init_data[6:7])[0]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def flags(self):
		return self._flags


class ConnStatusEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_STATUS))
	
	def __init__(self, init_data):
		super(ConnStatusEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._flags = unpack('<B', init_data[5:6])[0]
		self._address = unpack('<6B', init_data[6:12])
		self._address_type = unpack('<B', init_data[12:13])[0]
		self._conn_interval = unpack('<H', init_data[13:15])[0]
		self._timeout = unpack('<H', init_data[15:17])[0]
		self._latency = unpack('<H', init_data[17:19])[0]
		self._bonding = unpack('<B', init_data[19:20])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def flags(self):
		return self._flags
	
	@property
	def address(self):
		return self._address
	
	@property
	def address_type(self):
		return self._address_type
	
	@property
	def conn_interval(self):
		return self._conn_interval
	
	@property
	def timeout(self):
		return self._timeout
	
	@property
	def latency(self):
		return self._latency
	
	@property
	def bonding(self):
		return self._bonding


class ConnVersionIndEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_VERSION_IND))
	
	def __init__(self, init_data):
		super(ConnVersionIndEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._vers_nr = unpack('<B', init_data[5:6])[0]
		self._comp_id = unpack('<H', init_data[6:8])[0]
		self._sub_vers_nr = unpack('<H', init_data[8:10])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def vers_nr(self):
		return self._vers_nr
	
	@property
	def comp_id(self):
		return self._comp_id
	
	@property
	def sub_vers_nr(self):
		return self._sub_vers_nr


class ConnFeatureIndEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_FEATURE_IND))
	
	def __init__(self, init_data):
		super(ConnFeatureIndEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._features = init_data[6:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def features(self):
		return self._features


class ConnRawRxEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_RAW_RX))
	
	def __init__(self, init_data):
		super(ConnRawRxEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._data = init_data[6:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def data(self):
		return self._data


class ConnDisconnectedEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_DISCONNECTED))
	
	def __init__(self, init_data):
		super(ConnDisconnectedEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._reason = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def reason(self):
		return self._reason


class AcIndicatedEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_INDICATED))
	
	def __init__(self, init_data):
		super(AcIndicatedEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._attrhandle = unpack('<H', init_data[5:7])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def attrhandle(self):
		return self._attrhandle


class AcProcedureCompletedEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_PROCEDURE_COMPLETED))
	
	def __init__(self, init_data):
		super(AcProcedureCompletedEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
		self._chrhandle = unpack('<H', init_data[7:9])[0]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def result(self):
		return self._result
	
	@property
	def chrhandle(self):
		return self._chrhandle


class AcGroupFoundEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_GROUP_FOUND))
	
	def __init__(self, init_data):
		super(AcGroupFoundEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._start = unpack('<H', init_data[5:7])[0]
		self._end = unpack('<H', init_data[7:9])[0]
		self._uuid = init_data[10:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def start(self):
		return self._start
	
	@property
	def end(self):
		return self._end
	
	@property
	def uuid(self):
		return self._uuid


class AcAttributeFoundEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_ATTRIBUTE_FOUND))
	
	def __init__(self, init_data):
		super(AcAttributeFoundEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._chrdecl = unpack('<H', init_data[5:7])[0]
		self._value = unpack('<H', init_data[7:9])[0]
		self._properties = unpack('<B', init_data[9:10])[0]
		self._uuid = init_data[11:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def chrdecl(self):
		return self._chrdecl
	
	@property
	def value(self):
		return self._value
	
	@property
	def properties(self):
		return self._properties
	
	@property
	def uuid(self):
		return self._uuid


class AcFindInformationFoundEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_FIND_INFORMATION_FOUND))
	
	def __init__(self, init_data):
		super(AcFindInformationFoundEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._chrhandle = unpack('<H', init_data[5:7])[0]
		self._uuid = init_data[8:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def chrhandle(self):
		return self._chrhandle
	
	@property
	def uuid(self):
		return self._uuid


class AcAttributeValueEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_ATTRIBUTE_VALUE))
	
	def __init__(self, init_data):
		super(AcAttributeValueEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._atthandle = unpack('<H', init_data[5:7])[0]
		self._type = unpack('<B', init_data[7:8])[0]
		self._value = init_data[9:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def atthandle(self):
		return self._atthandle
	
	@property
	def type(self):
		return self._type
	
	@property
	def value(self):
		return self._value


class AcReadMultipleResponseEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_MULTIPLE_RESPONSE))
	
	def __init__(self, init_data):
		super(AcReadMultipleResponseEvt, self).__init__(init_data[:4])
		self._connection = unpack('<B', init_data[4:5])[0]
		self._handles = init_data[6:]
	
	@property
	def connection(self):
		return self._connection
	
	@property
	def handles(self):
		return self._handles


class SmSmpDataEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SMP_DATA))
	
	def __init__(self, init_data):
		super(SmSmpDataEvt, self).__init__(init_data[:4])
		self._handle = unpack('<B', init_data[4:5])[0]
		self._packet = unpack('<B', init_data[5:6])[0]
		self._data = init_data[7:]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def packet(self):
		return self._packet
	
	@property
	def data(self):
		return self._data


class SmBondingFailEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_BONDING_FAIL))
	
	def __init__(self, init_data):
		super(SmBondingFailEvt, self).__init__(init_data[:4])
		self._handle = unpack('<B', init_data[4:5])[0]
		self._result = unpack('<H', init_data[5:7])[0]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def result(self):
		return self._result


class SmPasskeyDisplayEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_PASSKEY_DISPLAY))
	
	def __init__(self, init_data):
		super(SmPasskeyDisplayEvt, self).__init__(init_data[:4])
		self._handle = unpack('<B', init_data[4:5])[0]
		self._passkey = unpack('<I', init_data[5:9])[0]
	
	@property
	def handle(self):
		return self._handle
	
	@property
	def passkey(self):
		return self._passkey


class SmPasskeyRequestEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_PASSKEY_REQUEST))
	
	def __init__(self, init_data):
		super(SmPasskeyRequestEvt, self).__init__(init_data[:4])
		self._handle = unpack('<B', init_data[4:5])[0]
	
	@property
	def handle(self):
		return self._handle


class SmBondStatusEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_BOND_STATUS))
	
	def __init__(self, init_data):
		super(SmBondStatusEvt, self).__init__(init_data[:4])
		self._bond = unpack('<B', init_data[4:5])[0]
		self._keysize = unpack('<B', init_data[5:6])[0]
		self._mitm = unpack('<B', init_data[6:7])[0]
		self._keys = unpack('<B', init_data[7:8])[0]
	
	@property
	def bond(self):
		return self._bond
	
	@property
	def keysize(self):
		return self._keysize
	
	@property
	def mitm(self):
		return self._mitm
	
	@property
	def keys(self):
		return self._keys


class GapScanResponseEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SCAN_RESPONSE))
	
	def __init__(self, init_data):
		super(GapScanResponseEvt, self).__init__(init_data[:4])
		self._rssi = unpack('<b', init_data[4:5])[0]
		self._packet_type = unpack('<B', init_data[5:6])[0]
		self._sender = unpack('<6B', init_data[6:12])
		self._address_type = unpack('<B', init_data[12:13])[0]
		self._bond = unpack('<B', init_data[13:14])[0]
		self._data = init_data[15:]
	
	@property
	def rssi(self):
		return self._rssi
	
	@property
	def packet_type(self):
		return self._packet_type
	
	@property
	def sender(self):
		return self._sender
	
	@property
	def address_type(self):
		return self._address_type
	
	@property
	def bond(self):
		return self._bond
	
	@property
	def data(self):
		return self._data


class GapModeChangedEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_MODE_CHANGED))
	
	def __init__(self, init_data):
		super(GapModeChangedEvt, self).__init__(init_data[:4])
		self._discover = unpack('<B', init_data[4:5])[0]
		self._connect = unpack('<B', init_data[5:6])[0]
	
	@property
	def discover(self):
		return self._discover
	
	@property
	def connect(self):
		return self._connect


class HwIoPortStatusEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_STATUS))
	
	def __init__(self, init_data):
		super(HwIoPortStatusEvt, self).__init__(init_data[:4])
		self._timestamp = unpack('<I', init_data[4:8])[0]
		self._port = unpack('<B', init_data[8:9])[0]
		self._irq = unpack('<B', init_data[9:10])[0]
		self._state = unpack('<B', init_data[10:11])[0]
	
	@property
	def timestamp(self):
		return self._timestamp
	
	@property
	def port(self):
		return self._port
	
	@property
	def irq(self):
		return self._irq
	
	@property
	def state(self):
		return self._state


class HwSoftTimerEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.HARDWARE), int(PacketMessage.HW_SOFT_TIMER))
	
	def __init__(self, init_data):
		super(HwSoftTimerEvt, self).__init__(init_data[:4])
		self._handle = unpack('<B', init_data[4:5])[0]
	
	@property
	def handle(self):
		return self._handle


class HwAdcResultEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.HARDWARE), int(PacketMessage.HW_ADC_RESULT))
	
	def __init__(self, init_data):
		super(HwAdcResultEvt, self).__init__(init_data[:4])
		self._input = unpack('<B', init_data[4:5])[0]
		self._value = unpack('<h', init_data[5:7])[0]
	
	@property
	def input(self):
		return self._input
	
	@property
	def value(self):
		return self._value


class DfuBootEvt(Header):
	index = PacketIndex(int(PacketType.EVENT), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_BOOT))
	
	def __init__(self, init_data):
		super(DfuBootEvt, self).__init__(init_data[:4])
		self._version = unpack('<I', init_data[4:8])[0]
	
	@property
	def version(self):
		return self._version


# Dizionario per la selezione della classe
_packet_index_to_subclass = {
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_HELLO)): SysHelloRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ADDRESS_GET)): SysAddressGetRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_REG_WRITE)): SysRegWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_REG_READ)): SysRegReadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_GET_COUNTERS)): SysGetCountersRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_GET_CONNECTIONS)): SysGetConnectionsRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_READ_MEMORY)): SysReadMemoryRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_GET_INFO)): SysGetInfoRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_TX)): SysEndpointTxRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_WHITELIST_APPEND)): SysWhitelistAppendRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_WHITELIST_REMOVE)): SysWhitelistRemoveRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_WHITELIST_CLEAR)): SysWhitelistClearRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_RX)): SysEndpointRxRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_SET_WATERMARKS)): SysEndpointSetWatermarksRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_BOOT)): SysBootEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_DEBUG)): SysDebugEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_WATERMARK_RX)): SysEndpointWatermarkRxEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_ENDPOINT_WATERMARK_TX)): SysEndpointWatermarkTxEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_SCRIPT_FAILURE)): SysScriptFailureEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_NO_LICENSE_KEY)): SysNoLicenseKeyEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SYSTEM), int(PacketMessage.SYS_PROTOCOL_ERROR)): SysProtocolErrorEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_DEFRAG)): PsPsDefragRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_DUMP)): PsPsDumpRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_ERASE_ALL)): PsPsEraseAllRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_SAVE)): PsPsSaveRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_LOAD)): PsPsLoadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_ERASE)): PsPsEraseRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_ERASE_PAGE)): PsErasePageRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_WRITE_WORDS)): PsWriteWordsRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.PERSISTENT_STORE), int(PacketMessage.PS_PS_KEY)): PsPsKeyEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_WRITE)): AdbWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_READ)): AdbReadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_READ_TYPE)): AdbReadTypeRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_USER_READ_RESPONSE)): AdbUserReadResponseRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_USER_WRITE_RESPONSE)): AdbUserWriteResponseRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_VALUE)): AdbValueEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_USER_READ_REQUEST)): AdbUserReadRequestEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_DATABASE), int(PacketMessage.ADB_STATUS)): AdbStatusEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_DISCONNECT)): ConnDisconnectRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_GET_RSSI)): ConnGetRssiRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_UPDATE)): ConnUpdateRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_VERSION_UPDATE)): ConnVersionUpdateRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_CHANNEL_MAP_GET)): ConnChannelMapGetRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_CHANNEL_MAP_SET)): ConnChannelMapSetRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_FEATURES_GET)): ConnFeaturesGetRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_GET_STATUS)): ConnGetStatusRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.CONNECTION), int(PacketMessage.CONN_RAW_TX)): ConnRawTxRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_STATUS)): ConnStatusEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_VERSION_IND)): ConnVersionIndEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_FEATURE_IND)): ConnFeatureIndEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_RAW_RX)): ConnRawRxEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.CONNECTION), int(PacketMessage.CONN_DISCONNECTED)): ConnDisconnectedEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_FIND_BY_TYPE_VALUE)): AcFindByTypeValueRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_BY_GROUP_TYPE)): AcReadByGroupTypeRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_BY_TYPE)): AcReadByTypeRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_FIND_INFORMATION)): AcFindInformationRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_BY_HANDLE)): AcReadByHandleRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_ATTRIBUTE_WRITE)): AcAttributeWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_WRITE_COMMAND)): AcWriteCommandRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_INDICATE_CONFIRM)): AcIndicateConfirmRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_LONG)): AcReadLongRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_PREPARE_WRITE)): AcPrepareWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_EXECUTE_WRITE)): AcExecuteWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_MULTIPLE)): AcReadMultipleRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_INDICATED)): AcIndicatedEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_PROCEDURE_COMPLETED)): AcProcedureCompletedEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_GROUP_FOUND)): AcGroupFoundEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_ATTRIBUTE_FOUND)): AcAttributeFoundEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_FIND_INFORMATION_FOUND)): AcFindInformationFoundEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_ATTRIBUTE_VALUE)): AcAttributeValueEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.ATTRIBUTE_CLIENT), int(PacketMessage.AC_READ_MULTIPLE_RESPONSE)): AcReadMultipleResponseEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_ENCRYPT_START)): SmEncryptStartRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SET_BONDABLE_MODE)): SmSetBondableModeRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_DELETE_BONDING)): SmDeleteBondingRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SET_PARAMETERS)): SmSetParametersRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_PASSKEY_ENTRY)): SmPasskeyEntryRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_GET_BONDS)): SmGetBondsRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SET_OOB_DATA)): SmSetOobDataRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_SMP_DATA)): SmSmpDataEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_BONDING_FAIL)): SmBondingFailEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_PASSKEY_DISPLAY)): SmPasskeyDisplayEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_PASSKEY_REQUEST)): SmPasskeyRequestEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.SECURITY_MANAGER), int(PacketMessage.SM_BOND_STATUS)): SmBondStatusEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_PRIVACY_FLAGS)): GapSetPrivacyFlagsRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_MODE)): GapSetModeRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_DISCOVER)): GapDiscoverRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_CONNECT_DIRECT)): GapConnectDirectRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_END_PROCEDURE)): GapEndProcedureRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_CONNECT_SELECTIVE)): GapConnectSelectiveRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_FILTERING)): GapSetFilteringRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_SCAN_PARAMETERS)): GapSetScanParametersRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_ADV_PARAMETERS)): GapSetAdvParametersRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_ADV_DATA)): GapSetAdvDataRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SET_DIRECTED_CONNECTABLE_MODE)): GapSetDirectedConnectableModeRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_SCAN_RESPONSE)): GapScanResponseEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.GENERIC_ACCESS_PROFILE), int(PacketMessage.GAP_MODE_CHANGED)): GapModeChangedEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_IRQ)): HwIoPortConfigIrqRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SET_SOFT_TIMER)): HwSetSoftTimerRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_ADC_READ)): HwAdcReadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_DIRECTION)): HwIoPortConfigDirectionRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_FUNCTION)): HwIoPortConfigFunctionRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_CONFIG_PULL)): HwIoPortConfigPullRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_WRITE)): HwIoPortWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_READ)): HwIoPortReadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SPI_CONFIG)): HwSpiConfigRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SPI_TRANSFER)): HwSpiTransferRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_I2C_READ)): HwI2CReadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_I2C_WRITE)): HwI2CWriteRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_SET_TXPOWER)): HwSetTxpowerRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.HARDWARE), int(PacketMessage.HW_TIMER_COMPARATOR)): HwTimerComparatorRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.HARDWARE), int(PacketMessage.HW_IO_PORT_STATUS)): HwIoPortStatusEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.HARDWARE), int(PacketMessage.HW_SOFT_TIMER)): HwSoftTimerEvt,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.HARDWARE), int(PacketMessage.HW_ADC_RESULT)): HwAdcResultEvt,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_TX)): TestPhyTxRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_RX)): TestPhyRxRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_END)): TestPhyEndRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_PHY_RESET)): TestPhyResetRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_GET_CHANNEL_MAP)): TestGetChannelMapRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.TESTING), int(PacketMessage.TEST_DEBUG)): TestDebugRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_FLASH_SET_ADDRESS)): DfuFlashSetAddressRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_FLASH_UPLOAD)): DfuFlashUploadRes,
	PacketIndex(int(PacketType.RESPONSE), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_FLASH_UPLOAD_FINISH)): DfuFlashUploadFinishRes,
	PacketIndex(int(PacketType.EVENT), int(PacketClass.DEVICE_FIRMWARE_UPGRADE), int(PacketMessage.DFU_BOOT)): DfuBootEvt
}


def select_packet_class(hdr, init_data):
	return _packet_index_to_subclass[hdr.index](init_data)


