# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_ros)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy nxt_python sensor_msgs nxt_msgs)
find_package(orocos_kdl REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS kdl
    CATKIN_DEPENDS rospy nxt_python sensor_msgs nxt_msgs
    INCLUDE_DIRS
    LIBRARIES
)

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# install Python-scripts
install(PROGRAMS 
	scripts/nxt_ros.py
	scripts/joint_states_aggregator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})