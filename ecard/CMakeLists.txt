cmake_minimum_required(VERSION 3.5)
project(ecard)

# ament_cmake
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
