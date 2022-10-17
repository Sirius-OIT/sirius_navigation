cmake_minimum_required(VERSION 3.8)
project(sirius_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_finid_build_dependencies()

ament_auto_add_library(sirius_navigation SHARED yaml-cpp)

ament_auto_add_executable(move_goal move_goal.cpp)

install(DIRECTORY launch
  DESTINATION share/sirius_navigation)
install(DIRECTORY config
  DESTINATION share/sirius_navigation)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
