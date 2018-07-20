import rospy
import re
from serial.tools.list_ports import comports
from rosmyo.msg import *
from sensor_msgs.msg import Imu
from struct import unpack
from .waitables import WaitableQueue
from .myo import *
from select import select


def bytes_to_hexstr(data, separator=' '):
	return separator.join([format(ord(x), '02x') for x in data])


class RosMyo:
	
	# Costants
	NODE_DEFAULT_NAME = 'rosmyo'
	TOPIC_QUEUE_SIZE = 10
	ORIENTATION_SCALE = 16384.0
	ACCELEROMETER_SCALE = 2048.0
	GYROSCOPE_SCALE = 16.0

	# Constructors
	def __init__(self):
		# ROS init
		rospy.init_node(RosMyo.NODE_DEFAULT_NAME, disable_signals=True)
		
		# Register topics
		self.__pub_imu = rospy.Publisher('{}/{}'.format(rospy.get_name(), rospy.get_param('~imu_topic', 'imu')), Imu, queue_size=RosMyo.TOPIC_QUEUE_SIZE)
		self.__pub_emg = rospy.Publisher('{}/{}'.format(rospy.get_name(), rospy.get_param('~emg_topic', 'emg')), Emg, queue_size=RosMyo.TOPIC_QUEUE_SIZE)
		self.__pub_classifier_armsync = rospy.Publisher('{}/{}'.format(rospy.get_name(), rospy.get_param('~classifier_armsync_topic', 'classifier_armsync')), ClassifierArmSync, queue_size=RosMyo.TOPIC_QUEUE_SIZE)
		self.__pub_classifier_locked = rospy.Publisher('{}/{}'.format(rospy.get_name(), rospy.get_param('~classifier_locked_topic', 'classifier_locked')), ClassifierLocked, queue_size=RosMyo.TOPIC_QUEUE_SIZE)
		self.__pub_classifier_pose = rospy.Publisher('{}/{}'.format(rospy.get_name(), rospy.get_param('~classifier_pose_topic', 'classifier_pose')), ClassifierPose, queue_size=RosMyo.TOPIC_QUEUE_SIZE)
		self.__pub_classifier_syncfailed = rospy.Publisher('{}/{}'.format(rospy.get_name(), rospy.get_param('~classifier_syncfailed_topic', 'classifier_syncfailed')), ClassifierSyncFailed, queue_size=RosMyo.TOPIC_QUEUE_SIZE)
		
		# Myo
		sp = rospy.get_param('~serial_port', RosMyo.find_dongle())
		rospy.loginfo('Using device {}'.format(sp))
		self.__myo = Myo(sp)
		
		# Parameters
		self.__imu_enabled = rospy.get_param('~imu_enabled', False)
		rospy.loginfo('IMU is {}'.format('enabled' if self.__imu_enabled else 'disabled'))
		self.__emg_enabled = rospy.get_param('~emg_enabled', False)
		rospy.loginfo('EMG is {}'.format('enabled' if self.__emg_enabled else 'disabled'))
		self.__classifier_enabled = rospy.get_param('~classifier_enabled', False)
		rospy.loginfo('Classifier is {}'.format('enabled' if self.__classifier_enabled else 'disabled'))
		
		self.__myo_identifier = rospy.get_param('~myo_identifier', 'Myo')
		rospy.loginfo('Myo identifier is {}'.format(self.__myo_identifier))
	
	@staticmethod
	def find_dongle():
		for p in comports():
			if re.search(r'PID=2458:0*1', p[2]):
				return p[0]
		return None
	

	# Publishers
	
	def pub_imu(self, value):
		m = Imu()
		m.header.stamp = rospy.Time.now()
		m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = [x / RosMyo.ORIENTATION_SCALE for x in unpack('<hhhh', value[:8])]
		m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = [x / RosMyo.ACCELEROMETER_SCALE for x in unpack('<hhh', value[8:14])]
		m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = [x / RosMyo.GYROSCOPE_SCALE for x in unpack('<hhh', value[14:20])]
		self.__pub_imu.publish(m)
		
	def pub_emg(self, value):
		e = Emg()
		e.header.stamp = rospy.Time.now()
		e.sample1 = unpack('<8b', value[:8])
		e.sample2 = unpack('<8b', value[8:16])
		self.__pub_emg.publish(e)
	
	def pub_classifier(self, value):
		if len(value) < 3:
			return
		
		typ, = unpack('<B', value[0])
		if typ in (0x01, 0x02):
			# Arm Synced
			c = ClassifierArmSync()
			c.arm, c.x_direction = unpack('<BB', value[1:3])
			pub = self.__pub_classifier_armsync.publish
		elif typ == 0x03:
			c = ClassifierPose()
			c.pose, = unpack('<H', value[1:3])
			pub = self.__pub_classifier_pose.publish
		elif typ in (0x04, 0x05):
			c = ClassifierLocked()
			pub = self.__pub_classifier_locked.publish
		else:
			# Sync Failed
			c = ClassifierSyncFailed()
			c.sync_result, = unpack('<B', value[1])
			pub = self.__pub_classifier_syncfailed.publish
		
		c.header.stamp = rospy.Time.now()
		pub(c)
		
	# Run()
	def run(self):
		rospy.loginfo('RosMyo running')
		
		if not self.__emg_enabled and not self.__imu_enabled and not self.__classifier_enabled:
			rospy.loginfo('Nothing to listen for! Closing...')
			return
		
		rospy.loginfo('RosMyo: connecting to the device')
		if not self.__myo.connect(self.__myo_identifier):
			rospy.logerr('Failed to connect to Myo[{}]'.format(self.__myo_identifier))
			return
		
		self.__myo.set_sleep_mode(MyoSleepModes.NEVER_SLEEP)
		rospy.loginfo('Sleep mode set to NEVER_SLEEP')
		
		imu_mode = MyoImuMode.NONE
		emg_mode = MyoEmgMode.NONE
		classifier_mode = MyoClassifierMode.DISABLED
		queues = []
		
		iq = None
		ie = None
		ic = None
		
		rospy.loginfo('Battery level is {}'.format(self.__myo.battery_level))
		fv_major, fv_minor, fv_patch, hv_rev = self.__myo.firmware_version
		rospy.loginfo('Firmware version is {}.{}.{}, Hardware version is {}'.format(fv_major, fv_minor, fv_patch, hv_rev))
		
		if self.__imu_enabled:
			rospy.loginfo('Enabling IMU')
			iq = WaitableQueue('IMU')
			self.__myo.imu_queue = iq
			imu_mode = MyoImuMode.SEND_RAW
			queues.append(iq)
		
		if self.__emg_enabled:
			rospy.loginfo('Enabling EMG')
			ie = WaitableQueue('EMG')
			self.__myo.emg_queue = ie
			emg_mode = MyoEmgMode.SEND_EMG_RAW
			queues.append(ie)
		
		if self.__classifier_enabled:
			rospy.loginfo('Enabling Classifier')
			ic = WaitableQueue('Classifier')
			self.__myo.classifier_queue = ic
			classifier_mode = MyoClassifierMode.ENABLED
			queues.append(ic)
		
		rospy.loginfo('Setting IMU, EMG and Classifier modes')
		self.__myo.set_mode(emg_mode, imu_mode, classifier_mode)
		self.__myo.unlock(MyoUnlockModes.HOLD)
		
		while True:
			try:
				r, _, _ = select(queues, [], [])
			except:
				break
			if self.__imu_enabled and iq in r:
				while True:
					evt = iq.pop()
					if evt is None:
						break
					self.pub_imu(evt.value)
			if self.__emg_enabled and ie in r:
				while True:
					evt = ie.pop()
					if evt is None:
						break
					self.pub_emg(evt.value)
			if self.__classifier_enabled and ic in r:
				while True:
					evt = ic.pop()
					if evt is None:
						break
					self.pub_classifier(evt.value)
		
		rospy.loginfo('RosMyo quitting')
		
		self.__myo.set_mode()
		self.__myo.set_sleep_mode()
		self.__myo.disconnect()
