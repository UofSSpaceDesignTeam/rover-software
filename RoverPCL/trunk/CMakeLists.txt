cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)
find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (openni_grabber src/openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

add_executable (pairwise_incremental_registration src/pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})

add_executable (downsample src/downsample.cpp)
target_link_libraries (downsample ${PCL_LIBRARIES})

add_executable (merge src/merge.cpp)
target_link_libraries (merge ${PCL_LIBRARIES})

add_executable (stream src/stream.cpp)
target_link_libraries (stream ${PCL_LIBRARIES})

add_executable (extract_indices src/extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})