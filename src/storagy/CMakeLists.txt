cmake_minimum_required(VERSION 3.8)
project(storagy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install resource directories. Add new dirs (worlds, maps, urdf, param, ...)
# here as the project grows so colcon copies them into install/share/storagy.
install(DIRECTORY
  launch
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
