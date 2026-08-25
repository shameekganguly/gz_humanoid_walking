cmake_minimum_required(VERSION 3.16)
project(gz_humanoid_walking VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-cmake REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(Eigen3 REQUIRED)

include(CTest)
include(GNUInstallDirs)

# Install models, worlds, and scripts
install(DIRECTORY models worlds scripts
  DESTINATION share/${PROJECT_NAME}
)
