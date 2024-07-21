cmake_minimum_required(VERSION 3.0.2)
project(Apollo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Apollo
#  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#add_executable(global src/global_test.cpp src/Global_Path.cpp)


# target_link_libraries(global
#   ${catkin_LIBRARIES}
#   libosqp.so  
#   libOsqpEigen.so 
#   libqdldl.so 
# )

add_executable(emplanner src/main.cpp  src/emplanner.cpp src/FrenetToCartesian.cpp src/Lattice.cpp 
    src/PID_Control.cpp src/LQR_Control.cpp
    src/Global_Path.cpp src/Path_QP.cpp src/Referenceline_info.cpp src/SpeedPlanning.cpp)

target_link_libraries(emplanner
${catkin_LIBRARIES}
libosqp.so  
libOsqpEigen.so 
libqdldl.so 
)