cmake_minimum_required(VERSION 3.0.2)

project(gazebo_crp450)

find_package(catkin REQUIRED
	roscpp
	rospy
	gazebo_msgs
	mavros_msgs
	geometry_msgs
	tf
	dynamic_reconfigure
)

find_package(gazebo REQUIRED)

set (DYNAMIC_RECONFIGURE_PATH config/arm_pid.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
      		  	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      		  	RESULT_VARIABLE cmd_result
      		 	OUTPUT_VARIABLE cmd_ver)
generate_dynamic_reconfigure_options(
	${DYNAMIC_RECONFIGURE_PATH}
)


catkin_package()
include_directories(
	include/
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# plugin
add_library(joint_plugin SHARED plugins/joint_plugin.cc)
target_link_libraries(joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


# class
add_library(PID_controller src/PID_controller/PID_controller.h src/PID_controller/PID_controller.cpp)
target_link_libraries(PID_controller ${catkin_LIBRARIES})

add_library(joint_controller src/joint_controller_gazebo/joint_controller.h src/joint_controller_gazebo/joint_controller.cpp)
target_include_directories(joint_controller PRIVATE src/PID_controller)
target_link_libraries(joint_controller PID_controller ${catkin_LIBRARIES})

# node
add_executable(gazebo_tools_init src/gazebo_tools_init.cpp)
target_include_directories(gazebo_tools_init PRIVATE src/PID_controller src/joint_controller_gazebo )
target_link_libraries(gazebo_tools_init joint_controller ${catkin_LIBRARIES})

add_executable(fly_demo src/fly_demo.cpp)
target_link_libraries(fly_demo ${catkin_LIBRARIES})

add_executable(dynamic_param src/dynamic_param.cpp)
add_dependencies(dynamic_param ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_param ${catkin_LIBRARIES})