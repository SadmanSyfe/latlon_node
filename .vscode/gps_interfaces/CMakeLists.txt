cmake_minimum_required(VERSION 3.8)
project(gps_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Custom Start
find_package(rosidl_default_generators REQUIRED)
# Find builtin_interfaces because you use its Time message
find_package(builtin_interfaces REQUIRED)

# Specify your message files for generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Location.msg" # <--- Specify your message file here
  # You can add more messages, services, or actions here
  DEPENDENCIES builtin_interfaces # Declare message-specific dependencies here
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
# Custom End
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
