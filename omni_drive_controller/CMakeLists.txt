cmake_minimum_required(VERSION 3.5)
project(omni_drive_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# List Dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp_lifecycle
  rclcpp
)

# Check Dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Add kinematics library (source and include)
add_library(kinematics SHARED)

target_sources(kinematics 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics.cpp
)

target_include_directories(kinematics
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add main library
add_library(${PROJECT_NAME} SHARED
  src/omni_drive_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(${PROJECT_NAME}
  kinematics
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "omni_drive_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface omni_drive_controller.xml)

# target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# pluginlib_export_plugin_description_file(controller_interface omni_drive_controller.xml)






# INSTALLS
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_omni_drive_arduino
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


## TESTING
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gmock REQUIRED)

#   ament_lint_auto_find_test_dependencies()

#   ament_add_gmock(
#     test_kinematics
#     test/test_kinematics.cpp
#     src/kinematics.cpp
#   )

#   target_link_libraries(test_kinematics
#     kinematics
#   )

#   target_include_directories(test_kinematics PRIVATE include)
#   ament_target_dependencies(test_kinematics)
# endif()


## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()





