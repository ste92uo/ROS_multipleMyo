# ROS_multipleMyo

REQUIREMENTS:
Install Robot Operating System (ROS), Kinetic or latter versions
 
 CONFIGURATION STEPS:
 
 1. Connect two or more Myo's dongles (one for each device) to USB ports of your computer
 2. Configure the Myo devices to be used by changing their device name. You can do it from the software MyoConnect, provided by ThalmicLabs (only for Windows and MacOS)
 3. Identify the path of each dongle (normally /dev/ttyACM*)
 
 LAUNCH ROSMYO:
 
 The rosmyo package is used to connect multiple Myo armbands to the same computer and public their sensors' data on separated topics. To use it, follow these steps:
 1. In rosmyo.launch, add a node of type rosmyo_node for each device you want to connect simultaneously
 2. In "~myo_identifier", write the name of the Myo device the node should connect to
 3. In "~serial_port", write the path of the dongle to be used for the connection
 
 roslaunch rosmyo rosmyo.launch
