cmake_minimum_required(VERSION 3.8)
project(hikvision_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)

# Driver node
add_executable(driver_node src/driver_node.cpp)
target_include_directories(driver_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/incEn)
target_link_directories(driver_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/lib)
target_link_libraries(driver_node ${OpenCV_LIBS} hcnetsdk PlayCtrl pthread dl)
ament_target_dependencies(driver_node rclcpp sensor_msgs cv_bridge)

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

# Install executables
install(TARGETS 
  driver_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install Hikvision libraries
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/lib/
  DESTINATION lib
  FILES_MATCHING 
  PATTERN "*.so*"
  PATTERN "*.xml"
)

# Set RPATH for the executable to find libraries in the install directory
set_target_properties(driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

ament_package()