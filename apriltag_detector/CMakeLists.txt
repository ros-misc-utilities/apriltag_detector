#
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

cmake_minimum_required(VERSION 3.16)
project(apriltag_detector)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#
# --------- header-only library
#
add_library(${PROJECT_NAME} INTERFACE
  include/apriltag_detector/detector.hpp)
  ament_target_dependencies(${PROJECT_NAME} INTERFACE sensor_msgs apriltag_msgs)

target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
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
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()


