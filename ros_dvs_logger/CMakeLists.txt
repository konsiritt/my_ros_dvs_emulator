cmake_minimum_required(VERSION 2.8.3)
project(ros_dvs_logger)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(ros_dvs_logger
  src/logger.cpp
  src/logger_node.cpp
)

# make the nodelet into a library
cs_add_library(ros_dvs_logger_nodelet
  src/logger_nodelet.cpp
  src/logger.cpp
)

# link the executable to the necesarry libs
target_link_libraries(ros_dvs_logger
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(ros_dvs_logger_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
install(FILES ros_dvs_logger_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
