cmake_minimum_required( VERSION 2.8.3 )
project(simple_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    message_generation
)

find_package(PCL REQUIRED)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(simple_slam src/main.cpp)

add_dependencies(simple_slam ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(simple_slam ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES})