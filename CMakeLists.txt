cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)
add_executable(calibrate
  src/camera_calibration.cpp
)
target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}  
)
add_executable(see
  src/map_watcher.cpp
)
target_link_libraries(see
  ${OpenCV_LIBRARIES}  
  stdc++fs
)
target_compile_definitions(see PRIVATE EXAMPLE_USAGE=1)
## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   src/map_watcher.cpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES} 
  stdc++fs 
)
