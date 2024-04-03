# -*- cmake -*-
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

add_compile_options(-Wall -Wextra -pedantic -Werror)
add_definitions(-DUSING_ROS_1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(apriltag REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  image_transport
  cv_bridge
  sensor_msgs
  apriltag_detector_msgs)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
# --------- shared library
#
add_library(${PROJECT_NAME} SHARED
  src/apriltag_detector_ros1.cpp
  src/convert_detections.cpp
  src/detector_wrapper_base.cpp
  src/detector_wrapper.cpp
  src/draw_tag.cpp)

target_link_libraries(
  ${PROJECT_NAME} opencv_core opencv_imgproc apriltag ${catkin_LIBRARIES})
#
# --------- nodelet
#
add_library(${PROJECT_NAME}_nodelet src/nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES})

#
# -------- node
#
add_executable(${PROJECT_NAME}_node src/node_ros1.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch")
