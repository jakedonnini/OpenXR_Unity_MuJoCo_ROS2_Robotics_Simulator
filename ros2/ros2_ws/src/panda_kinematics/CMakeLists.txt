cmake_minimum_required(VERSION 3.8)
project(panda_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Build the arm controller node
add_executable(panda_arm_controller 
  src/panda_arm_controller.cpp
  src/kinematics.cpp
)
ament_target_dependencies(panda_arm_controller rclcpp Eigen3)

# Link the custom message to the executable
rosidl_target_interfaces(panda_arm_controller 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  panda_arm_controller
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()