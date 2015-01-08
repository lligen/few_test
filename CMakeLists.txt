cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_TEST)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})


add_executable (normal_estimation normal_estimation_using_integral_images.cpp)
target_link_libraries (normal_estimation ${PCL_LIBRARIES})

add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})

add_executable (test test.cpp)
target_link_libraries (test ${PCL_LIBRARIES})

add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})
