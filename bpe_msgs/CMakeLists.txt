cmake_minimum_required(VERSION 3.10.2)
project(bpe_msgs)

# Default to C++17 and compiler flags to give all warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
if(MSVC)
  add_compile_options(-WX -W2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install the header in the include folder, necessary for the actions to work in Windows properly
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

# Services, messages and actions to generate
rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/BpeStatistics.msg"
  
  # Depedencies for generating the messages, services and actions
  DEPENDENCIES 
  builtin_interfaces 
  std_msgs
  geometry_msgs
  nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
