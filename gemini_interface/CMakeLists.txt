cmake_minimum_required(VERSION 3.8)
project(gemini_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/GeminiService.srv"
)

ament_package()