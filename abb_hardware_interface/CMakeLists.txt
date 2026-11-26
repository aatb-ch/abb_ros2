cmake_minimum_required(VERSION 3.5)
project(abb_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
      -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
      -Wno-unused-parameter -Wno-unused-function)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    abb_egm_rws_managers
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(
  ${PROJECT_NAME}
  SHARED
  src/abb_hardware_interface.cpp
  src/utilities.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(hardware_interface abb_hardware_interface.xml)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable ament_copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Hardware-interface tests against a fake robot controller (EGM over UDP
  # on localhost, see test/fake_egm_robot.hpp).
  find_package(ament_cmake_gtest REQUIRED)
  find_package(abb_libegm REQUIRED)
  ament_add_gtest(test_abb_hardware_interface
    test/test_abb_hardware_interface.cpp
    TIMEOUT 300)
  target_include_directories(test_abb_hardware_interface PRIVATE include test)
  target_link_libraries(test_abb_hardware_interface
    ${PROJECT_NAME} abb_libegm::abb_libegm)
  ament_target_dependencies(test_abb_hardware_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
