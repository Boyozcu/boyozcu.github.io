cmake_minimum_required(VERSION 3.0.2)
project(blm6191_coverage_planners)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  geometry_msgs
  pluginlib
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  visualization_msgs
  base_local_planner
)

catkin_package(
  LIBRARIES blm6191_coverage_planners_global blm6191_coverage_planners_local
  CATKIN_DEPENDS roscpp nav_core costmap_2d geometry_msgs pluginlib nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs base_local_planner angles
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(blm6191_coverage_planners_global
  full_coverage_planner.cpp
)
target_link_libraries(blm6191_coverage_planners_global
  ${catkin_LIBRARIES}
)

add_library(blm6191_coverage_planners_local
  pure_pursuit_local_planner.cpp
)
target_link_libraries(blm6191_coverage_planners_local
  ${catkin_LIBRARIES}
)

# Opsiyonel C++ Polygon Publisher Düğümü (eğer kullanılıyorsa)
# add_executable(polygon_publisher_node polygon_publisher_node.cpp)
# target_link_libraries(polygon_publisher_node ${catkin_LIBRARIES})

install(TARGETS blm6191_coverage_planners_global blm6191_coverage_planners_local
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  full_coverage_planner_plugin.xml
  pure_pursuit_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  polygon_visualizer_node.py # Paket kökündeyse
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS polygon_publisher_node # Eğer C++ versiyonu varsa
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(FILES
  my_coverage_move_base.launch
  costmap_common_params.yaml
  local_costmap_params.yaml
  global_costmap_params.yaml
  planner_params.yaml
  coverage_assignment.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
