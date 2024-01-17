cmake_minimum_required(VERSION 3.8)
project(mapping)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)

set(CMAKE_CXX_STANDARD 17)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


ament_auto_add_executable(cloud_reader src/cloud_reader.cpp)
target_link_libraries (cloud_reader ${PCL_LIBRARIES})

ament_auto_add_executable(mapping src/mapping.cpp)
target_link_libraries (mapping ${PCL_LIBRARIES})

install(PROGRAMS scripts/obstacle_node.py scripts/map_node.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scripts/utm
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})


ament_auto_package()
