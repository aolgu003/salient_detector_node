cmake_minimum_required(VERSION 2.8.3)
project(salient_detector)
find_package(catkin REQUIRED COMPONENTS 
	std_msgs
	sensor_msgs
	opencv_apps
	rostime 
	roscpp 
	rosconsole 
	roscpp_serialization 
	cv_bridge 
	image_transport
	message_generation 
  image_geometry
)

find_package( OpenCV REQUIRED)

add_message_files(
  FILES
  uav_targets.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  opencv_apps
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_subdirectory( include )

add_executable(salient_detection_node src/salient_detection_node.cpp)
target_link_libraries(salient_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} saliency)

