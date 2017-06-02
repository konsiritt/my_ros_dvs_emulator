cmake_minimum_required(VERSION 2.8.3)
project(ros_dvs_renderer)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(ros_dvs_renderer
  src/image_tracking.cpp
  src/renderer.cpp
  src/renderer_node.cpp
)

# make the nodelet into a library
cs_add_library(ros_dvs_renderer_nodelet
  src/image_tracking.cpp
  src/renderer_nodelet.cpp
  src/renderer.cpp
)

# link the executable to the necesarry libs
target_link_libraries(ros_dvs_renderer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(ros_dvs_renderer_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
install(FILES ros_dvs_renderer_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
