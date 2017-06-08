cmake_minimum_required(VERSION 2.8.3)
project(ros_dvs_benchmark)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system thread )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
set(CMAKE_BUILD_TYPE Debug)

# make the executable
cs_add_executable(ros_bench_emul
  src/bench_emul_node.cpp
  src/bench_emul.cpp
)

# link the executable to the necesarry libs
target_link_libraries(ros_bench_emul
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# make the executable
cs_add_executable(ros_bench_subscr
  src/bench_subscr_node.cpp
  src/bench_subscr.cpp
)

# link the executable to the necesarry libs
target_link_libraries(ros_bench_subscr
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the nodelet library
cs_install()

# Install other support files for installation
#install(FILES ros_dvs_emulator_nodelet.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
