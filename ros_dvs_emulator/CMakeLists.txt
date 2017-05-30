cmake_minimum_required(VERSION 2.8.3)
project(ros_dvs_emulator)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system thread )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
set(CMAKE_BUILD_TYPE Debug)

# make the executable
cs_add_executable(ros_dvs_emulator
  src/emulator_node.cpp
  src/emulator.cpp
  src/Timer.cpp
)

# make the nodelet into a library
cs_add_library(ros_dvs_emulator_nodelet
  src/emulator_nodelet.cpp
  src/emulator.cpp
)

# link the executable to the necesarry libs
target_link_libraries(ros_dvs_emulator
  ${catkin_LIBRARIES}
  rt
  ${Boost_LIBRARIES}
  
)

# link the executable to the necesarry libs
target_link_libraries(ros_dvs_emulator_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the nodelet library
cs_install()

# Install other support files for installation
install(FILES ros_dvs_emulator_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
