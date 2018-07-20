cmake_minimum_required(VERSION 2.8.3)
project(rosmyo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rospy
	sensor_msgs
	std_msgs
	message_generation
)

catkin_python_setup()

###########
## Build ##
###########

# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    ClassifierArmSync.msg
    ClassifierLocked.msg
    ClassifierPose.msg
    ClassifierSyncFailed.msg
    Emg.msg
)

## Generate added messages and services with any dependencies
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime
)
