cmake_minimum_required(VERSION 3.8)
project(can_transceiver)

# Set to C99
set(CMAKE_C_STANDARD 99)

# Set to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(can_interfaces REQUIRED)

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

include_directories(include)

set(CAN_DRIVER ${CMAKE_CURRENT_BINARY_DIR}/lib/libcontrolcan.so)
configure_file(lib/libcontrolcan.so lib/libcontrolcan.so COPYONLY)

add_library(transceiver_lib SHARED
            src/can_transceiver.cpp
)
target_link_libraries(transceiver_lib)
target_link_libraries(transceiver_lib ${YAML_CPP_LIBRARIES} ${CAN_DRIVER} pthread)
ament_target_dependencies(transceiver_lib 
                          rclcpp 
                          yaml-cpp
                          can_interfaces       
)

add_executable(can_transceiver_node src/main.cpp)
target_link_libraries(can_transceiver_node transceiver_lib)
ament_target_dependencies(can_transceiver_node rclcpp)


install(TARGETS
  transceiver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  can_transceiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
  lib
  DESTINATION lib
)

ament_package()
