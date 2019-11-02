cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar_planner)
set (CMAKE_CXX_STANDARD 11)

# find_package(catkin REQUIRED
#   COMPONENTS
#     roscpp
#     rosbag
#     std_msgs
#     tf
#     cv_bridge
# )

find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)

# catkin_package()

file(GLOB_RECURSE SRCS "src/*.cpp")

add_executable(hybrid_astar_planner main.cpp ${SRCS})
target_link_libraries(hybrid_astar_planner
  # ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ompl
  glog
)

# install(TARGETS hybrid_astar_planner
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )