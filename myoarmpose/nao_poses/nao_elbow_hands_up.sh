#!/usr/bin/env bash

rostopic pub /nao_robot/pose/joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['LElbowYaw', 'RElbowYaw']
joint_angles: [-3.14, 3.14]
speed: 1.0
relative: 0"
