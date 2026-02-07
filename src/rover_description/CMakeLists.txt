cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

# Install URDF/Xacro resources into the share directory.
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
