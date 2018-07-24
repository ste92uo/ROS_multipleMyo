# coding=utf-8

import rospy
from rosmyo.msg import ClassifierPose
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import *
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import math
from collections import deque
import numpy
from myoarmpose.msg import Euler

import sys
import motion
import almath
from naoqi import ALProxy


# Transform Quaternion to iterable
def quat_to_iterable(q):
    return q.x, q.y, q.z, q.w

# Quaternion multiplication
def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

# Compute the norm of a vector
def vect_norm(v):
    return math.sqrt(sum([t ** 2 for t in vect_to_iterable(v)]))

# Transform Vector to iterable
def vect_to_iterable(v):
    return v.x, v.y, v.z

# Vector normalization
def vect_normalize(v):
    norm = vect_norm(v)
    return tuple(t / norm for t in v)

# Rotate vector v by quaternion q
def vect_rotation(v, q):
    vi = [v.x, v.y, v.z]
    rotm = [row[:3] for row in quaternion_matrix(q)[:3]]
    rv = [sum([row[i] * vi[i] for i in range(len(row))]) for row in rotm]
    return Vector3(*rv)


class MyoArmPose(object):
    # Costants
    NODE_DEFAULT_NAME = 'myoarmpose'
    TOPIC_QUEUE_SIZE = 10

    def __init__(self):
        rospy.init_node(MyoArmPose.NODE_DEFAULT_NAME, disable_signals=False)

        # Parameters
        arm_imu_topic = rospy.get_param('~arm_imu_topic', '/myos/arm/imu')
        forearm_imu_topic = rospy.get_param('~forearm_imu_topic', '/myos/forearm/imu')
        arm_euler_topic = rospy.get_param('~arm_euler_topic', '/myos/arm/euler')
        forearm_euler_topic = rospy.get_param('~forearm_euler_topic', '/myos/forearm/euler')
        forearm_pose_topic = rospy.get_param('~forearm_pose_topic', '/myos/forearm/pose')
        nao_joints_topic = rospy.get_param('~nao_joints_topic', '/nao_robot/pose/joint_angles')

        # Subscribers
        self.__sub_arm_imu = rospy.Subscriber(arm_imu_topic, Imu, self.sub_arm_imu, queue_size=MyoArmPose.TOPIC_QUEUE_SIZE)
        self.__sub_forearm_imu = rospy.Subscriber(forearm_imu_topic, Imu, self.sub_forearm_imu, queue_size=MyoArmPose.TOPIC_QUEUE_SIZE)
        self.__sub_forearm_pose = rospy.Subscriber(forearm_pose_topic, ClassifierPose, self.sub_forearm_pose)

        # Publishers
        self.__pub_joints = rospy.Publisher(nao_joints_topic, JointAnglesWithSpeed,
                                            queue_size=MyoArmPose.TOPIC_QUEUE_SIZE)
        self.nao_arm = rospy.get_param('~nao_arm', 'right')
        if self.nao_arm.lower() == 'left':
            self.__joint_names = ['LShoulderRoll', 'LShoulderPitch', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
        else:
            self.__joint_names = ['RShoulderRoll', 'RShoulderPitch', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']

        self.__pub_arm_eul = rospy.Publisher(arm_euler_topic, Euler, queue_size=MyoArmPose.TOPIC_QUEUE_SIZE)
        self.__pub_forearm_eul = rospy.Publisher(forearm_euler_topic, Euler, queue_size=MyoArmPose.TOPIC_QUEUE_SIZE)

        # Initial orientation offset
        self.__arm_orientation = [0, 0, 0, 1]
        self.__arm_orientation_offset = None
        self.__forearm_orientation = [0, 0, 0, 1]
        self.__forearm_orientation_offset = None

        self.reset_arms()
        self.__q_to_ny = quaternion_from_euler(0, 0, math.pi)

    def calculate_angles(self, qa, qf):
        shoulder_yaw, shoulder_roll, shoulder_pitch = euler_from_quaternion(qa, 'sxzy')
        forearm_yaw, forearm_roll, forearm_pitch = euler_from_quaternion(qf, 'sxzy')
        self.publish_arm_eul(shoulder_yaw, shoulder_roll, shoulder_pitch)

        qa_no_yaw = quaternion_from_euler(0, shoulder_roll, shoulder_pitch, 'sxzy')
        qf_no_yaw = quaternion_from_euler(0, forearm_roll, forearm_pitch, 'sxzy')

        qfa = quaternion_multiply(qf_no_yaw, quaternion_inverse(qa_no_yaw))

        wrist_yaw, Myo_E_roll, Myo_E_pitch = euler_from_quaternion(qfa, 'sxzy')
        self.publish_forearm_eul(wrist_yaw, Myo_E_roll, Myo_E_pitch)

        elbow_roll = math.acos(math.cos(Myo_E_roll)*math.cos(Myo_E_pitch))      # theta_R_4

        if math.degrees(Myo_E_roll) <= 4 and abs(math.degrees(Myo_E_pitch)) <= 4:
            elbow_yaw = 0
        elif math.degrees(Myo_E_roll) <= 4 and math.degrees(Myo_E_pitch) > 4:
            elbow_yaw = -math.pi / 2
        elif math.degrees(Myo_E_roll) <= 4 and math.degrees(Myo_E_pitch) < -4:
            elbow_yaw = math.pi / 2
        else:
            elbow_yaw = -math.atan(math.tan(Myo_E_pitch)/math.sin(Myo_E_roll))       # theta_R_3

        print('SROLL =  ' + str(math.degrees(shoulder_roll)))
        print('SPITCH =  ' + str(math.degrees(shoulder_pitch)))
        print('EYAW =  ' + str(math.degrees(elbow_yaw)))
        print('EROLL =  ' + str(math.degrees(elbow_roll)))

        return shoulder_roll, shoulder_pitch, elbow_yaw, elbow_roll, 0

    # Publishers

    def publish_joints(self, names, angles):
        m = JointAnglesWithSpeed()
        m.header.stamp = rospy.Time.now()
        m.joint_names = names
        m.joint_angles = angles
        m.relative = 0
        m.speed = 1
        self.__pub_joints.publish(m)

    def publish_arm_eul(self, angles_yaw, angles_roll, angles_pitch):
        m = Euler()
        m.roll = angles_roll
        m.pitch = angles_pitch
        m.yaw = angles_yaw
        self.__pub_arm_eul.publish(m)

    def publish_forearm_eul(self, angles_yaw, angles_roll, angles_pitch):
        m = Euler()
        m.roll = angles_roll
        m.pitch = angles_pitch
        m.yaw = angles_yaw
        self.__pub_forearm_eul.publish(m)

    def reset_arms(self):
        self.publish_joints([
            'RShoulderRoll', 'RShoulderPitch', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
            'LShoulderRoll', 'LShoulderPitch', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'
        ], [0] * 10)

    # Subscribers

    def sub_forearm_pose(self, msg):
        if msg.pose == ClassifierPose.DOUBLE_TAP:
            self.__arm_orientation_offset = None
            self.__forearm_orientation_offset = None

    def sub_forearm_imu(self, msg):
        orientation = quat_to_iterable(msg.orientation)

        # Offset removal to have X+ axis in forward direction and Y+ axis medially oriented
        orientation = quaternion_multiply(quaternion_multiply(self.__q_to_ny, orientation), self.__q_to_ny)

        # Verify if an initial orientation offset has been acquired
        if self.__forearm_orientation_offset is None:
            self.__forearm_orientation_offset = orientation

        # Remove initial orientation offset
        qi = quaternion_inverse(self.__forearm_orientation_offset)
        self.__forearm_orientation = quaternion_multiply(orientation, qi)

        # Update robot pose if both Myos are sending their orientation
        if self.__arm_orientation is not None:
            angles = self.calculate_angles(self.__arm_orientation, self.__forearm_orientation)
            self.publish_joints(self.__joint_names, angles)

    def sub_arm_imu(self, msg):
        orientation = quat_to_iterable(msg.orientation)

        # Offset removal to have X+ axis in forward direction and Y+ axis medially oriented
        orientation = quaternion_multiply(quaternion_multiply(self.__q_to_ny, orientation), self.__q_to_ny)

        # Verify if an initial orientation offset has been acquired
        if self.__arm_orientation_offset is None:
            self.__arm_orientation_offset = orientation

        # Remove initial orientation offset
        qi = quaternion_inverse(self.__arm_orientation_offset)
        self.__arm_orientation = quaternion_multiply(orientation, qi)

        # Update robot pose if both Myos are sending their orientation
        if self.__forearm_orientation is not None:
            angles = self.calculate_angles(self.__arm_orientation, self.__forearm_orientation)
            self.publish_joints(self.__joint_names, angles)

    def run(self):
        rospy.spin()
