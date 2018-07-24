#!/usr/bin/env bash

rostopic pub /nao_robot/pose/joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['RShoulderRoll', 'RShoulderPitch', 'LShoulderRoll', 'LShoulderPitch', 'LElbowRoll', 'LElbowYaw', 'RElbowRoll', 'RElbowYaw']
joint_angles: [0, 0.1, 0, 0.1, 0, 0, 0, 0]
speed: 1.0
relative: 1" -r 10
