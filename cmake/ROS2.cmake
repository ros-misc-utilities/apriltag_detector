# -*- cmake -*-
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(ament_dependencies
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "apriltag_msgs")

foreach(pkg ${ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

if(${cv_bridge_VERSION} GREATER "3.3.0")
  add_definitions(-DUSE_CV_BRIDGE_HPP)
endif()

#
# --------- wrapper library (exported)
#
add_library(detector_wrapper SHARED
  src/detector_wrapper.cpp
  src/detector_wrapper_base.cpp
  src/convert_detections.cpp
  src/draw_tag.cpp)

ament_target_dependencies(detector_wrapper ${ament_dependencies})
target_link_libraries(detector_wrapper opencv_core opencv_imgproc apriltag)

target_include_directories(
    detector_wrapper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(apriltag OpenCV)

#
# --------- composable node library (private)
#
add_library(${PROJECT_NAME} SHARED
  src/apriltag_detector.cpp)

ament_target_dependencies(${PROJECT_NAME} ${ament_dependencies})
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc detector_wrapper)

target_include_directories(${PROJECT_NAME} PRIVATE include)

rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::ApriltagDetector")

#
# -------- node
#
add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# the node must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(
  TARGETS detector_wrapper
  EXPORT ${PROJECT_NAME}_export
  DESTINATION lib
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING
  PATTERN "detector_wrapper*.hpp"
  PATTERN "detector_wrapper_ros1.hpp" EXCLUDE
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
