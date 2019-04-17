cmake_minimum_required(VERSION 2.8.3)
project(mbot_dst_rule_based)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	rospy
)

add_message_files(
	DIRECTORY ros/msg
   	FILES
		InformSlot.msg
	  	DialogAct.msg
	  	DialogActArray.msg
)

catkin_python_setup()

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS
	message_runtime
)