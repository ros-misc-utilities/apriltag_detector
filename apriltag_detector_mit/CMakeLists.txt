#
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(apriltag_detector_mit)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(apriltag_mit)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(ament_dependencies
  "apriltag_detector"
  "apriltag_msgs"
  "cv_bridge"
  "image_transport"
  "pluginlib"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs")

foreach(pkg ${ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

if(${cv_bridge_VERSION} GREATER "3.3.0")
  add_definitions(-DUSE_CV_BRIDGE_HPP)
endif()

pluginlib_export_plugin_description_file(apriltag_detector plugins.xml)


#
# --------- plugin library with node-free detector (exported)
#
add_library(${PROJECT_NAME} SHARED
  src/detector.cpp)

ament_target_dependencies(${PROJECT_NAME} ${ament_dependencies})
target_link_libraries(${PROJECT_NAME} apriltag_mit::apriltag_mit)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

#
# --------- composable node library (exported)
#
add_library(${PROJECT_NAME}_component SHARED
  src/component.cpp)

ament_target_dependencies(${PROJECT_NAME}_component ${ament_dependencies})
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_libraries(${PROJECT_NAME}_component)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "${PROJECT_NAME}::Component")

#
# -------- node
#
add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)

# the node must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_component
  EXPORT ${PROJECT_NAME}_export
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "apriltag_detector.hpp"
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()

ament_package()
