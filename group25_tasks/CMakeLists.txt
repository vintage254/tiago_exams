cmake_minimum_required(VERSION 3.4.0)
project(group25_tasks)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    scripts/task1_exploration.py
    scripts/task2_aruco_navigation.py
    scripts/task3_pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
