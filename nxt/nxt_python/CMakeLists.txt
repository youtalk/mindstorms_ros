# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_python)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-usb python-bluez
    CATKIN_DEPENDS rospy
    INCLUDE_DIRS
    LIBRARIES
)

## Install Python Examples
install(PROGRAMS
  sensor_tests/color_sensor_test.py
  sensor_tests/touch_sensor_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)