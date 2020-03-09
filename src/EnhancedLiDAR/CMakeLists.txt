cmake_minimum_required(VERSION 2.8.3)
project(EnhancedLiDAR)

add_compile_options(-std=c++11)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${OpenCV_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/EnhancedLiDAR_node.cpp src/EnhancedLiDAR_core.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

