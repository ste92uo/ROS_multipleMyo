# ROS_multipleMyo

REQUIREMENTS:
1. Install Robot Operating System (ROS), Kinetic or latter versions.
2. Install NAOqi SDK to test the system on NAO or Pepper robot.
 
 CONFIGURATION STEPS:
 
 1. Connect two or more Myo's dongles (one for each device) to USB ports of your computer.
 2. Configure the Myo devices to be used by changing their device name. You can do it from the software MyoConnect, provided by ThalmicLabs (only for Windows and MacOS).
 3. Identify the path of each dongle (normally /dev/ttyACM*).
 
 LAUNCH ROSMYO:
 
 The rosmyo package is used to connect multiple Myo armbands to the same computer and public their sensors' data on separated topics. To use it, follow these steps:
 1. In rosmyo.launch, add a node of type rosmyo_node for each device you want to connect simultaneously.
 2. In "~myo_identifier", write the name of the Myo device the node should connect to.
 3. In "~serial_port", write the path of the dongle to be used for the connection.
 4. In terminal, run: roslaunch rosmyo rosmyo.launch
 
 LAUNCH MYOARMPOSE:
 
 The myoarmpose package is used to estimate the user arm pose from a pair of Myo armbands, worn on the upper arm and forearm respectively, and publish the joints' angles to control the arm of a humanoid robot. To use it, follow these steps:
 1. Open myoarmpose.launch and change the parameters inside "myos" group as following explained.
 2. In each "~myo_identifier", write the name of the Myo device the node should connect to.
 3. In each "~serial_port", write the path of the dongle to be used for the connection.
 4. In pose_estimator node, in "~nao_joints_topic", write the topic for the pubblication of the estimated joints' angles (leave the default if you want to test the system on a NAO or Pepper robot).
 5. In "~nao_arm", write the robot arm to be controlled, either "right" (default) or "left"
 6. In terminal, run: roslaunch myoarmpose myoarmpose.launch
 
N.B. The two Myo devices should be worn with the blue led pointing TOWARDS the hand.

TO VISUALIZE THE RESULTS:
1. Install NAOqi SDK
2. Launch a virtual robot avatar on Rviz

INSTALL NAOQI:

sudo apt-get update

sudo apt-get install ros-kinetic-nao-* ros-kinetic-octomap-ros ros-kinetic-octomap-msgs ros-kinetic-map-server ros-kinetic-fake-localization ros-kinetic-sbpl ros-kinetic-naoqi-* ros-kinetic-moveit*

cd ~/Downloads

git clone --branch 2.1.2.17 https://<your_bb_username>@bitbucket.org/iaslab-unipd/naoqi.git

sudo mv naoqi /opt/

echo "export PYTHONPATH=$PYTHONPATH:/opt/naoqi/pynaoqi-python2.7-2.1.2.17-linux64" >> ~/.bashrc

echo "export AL_DIR=/opt/naoqi/naoqi-sdk-2.1.2.17-linux64" >> ~/.bashrc

cd ~/workspace/ros/catkin/src

git clone https://github.com/michieletto/naoqi_driver.git

git clone https://github.com/ros-naoqi/naoqi_bridge.git

git clone https://github.com/ros-naoqi/naoqi_bridge_msgs.git

git clone https://github.com/FelipMarti/nao_robot.git

git clone https://github.com/ros-naoqi/nao_extras.git

cd ..
catkin_make

USE NAOQI:

cd $AL_DIR

./naoqi -b 127.0.0.1

roslaunch nao_bringup nao_full.launch nao_ip:=127.0.0.1 network_interface:=lo

rviz
 -> Open the config file at src/nao_robot/nao_description/config/nao.rviz




