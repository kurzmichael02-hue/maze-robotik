cmake_minimum_required(VERSION 3.8)
project(maze_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# planner nodes kommen hier rein
# add_executable(wall_follower src/wall_follower.cpp)
# add_executable(bfs_planner   src/bfs_planner.cpp)
# add_executable(astar_planner src/astar_planner.cpp)
# add_executable(floodfill_planner src/floodfill_planner.cpp)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
