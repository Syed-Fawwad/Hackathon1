cmake_minimum_required(VERSION 3.8)
project(gazebo_models)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install model files
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install world files
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()