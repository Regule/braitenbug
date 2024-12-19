cmake_minimum_required(VERSION 3.8)
project(braitenbug)

# CONFIG
set(PACKAGE_DEPENDENCIES 
  ament_cmake
  ament_cmake_python
  rclcpp
  sensor_msgs
  braitenbug_msgs
)

set(PACKAGE_COMPILE_FEATURES
  c_std_99
  cxx_std_17
)

set(PACKAGE_INCLUDE_DIRECTORIES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

foreach(package_name ${PACKAGE_DEPENDENCIES})
  find_package(${package_name} REQUIRED)
endforeach()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# EXECUTABLES
add_executable(whiskers-node src/whiskers-node.cpp)
target_include_directories(whiskers-node PUBLIC ${PACKAGE_INCLUDE_DIRECTORIES})
target_compile_features(whiskers-node PUBLIC ${PACKAGE_COMPILE_FEATURES})  
ament_target_dependencies(whiskers-node
  rclcpp
  sensor_msgs
  braitenbug_msgs
)

add_executable(fake_lidar src/fake_lidar.cpp)
target_include_directories(fake_lidar PUBLIC ${PACKAGE_INCLUDE_DIRECTORIES})
target_compile_features(fake_lidar PUBLIC ${PACKAGE_COMPILE_FEATURES})  
ament_target_dependencies(fake_lidar
  rclcpp
  sensor_msgs
)

# INSTALLATION
install(TARGETS whiskers-node fake_lidar
  DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS
scripts/ui_test.py
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
