# Copyright (c) 2017 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(object_map)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

# Messages
add_message_files(DIRECTORY msg FILES
    Object.msg
    Objects.msg
    ObjectInfo.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

# What other packages will need to use this package
catkin_package(
)

#############
## Install ##
#############

install(PROGRAMS
  nodes/object_map_node.py
  nodes/test_conf.py
  nodes/test_object_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## test    ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python()
  roslint_add_test()
endif()
