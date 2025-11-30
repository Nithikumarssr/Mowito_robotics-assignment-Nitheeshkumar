cmake_minimum_required(VERSION 3.8)
project(ros_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find external dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces (must be before executables)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  DEPENDENCIES std_msgs  # std_msgs for any base types
)

# Get the C++ typesupport target for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Publisher (uses std_msgs only)
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher
  rclcpp
  std_msgs
)

# Subscriber (uses std_msgs only)
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber
  rclcpp
  std_msgs
)

# Service Server (uses custom srv)
add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server
  rclcpp
)
target_link_libraries(service_server
  ${cpp_typesupport_target}
)

# Service Client (uses custom srv)
add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client
  rclcpp
)
target_link_libraries(service_client
  ${cpp_typesupport_target}
)

# Install all executables
install(TARGETS
  publisher
  subscriber
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME}
)

# Export the package (for other packages to use interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_find_test_dependencies RELEASE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

