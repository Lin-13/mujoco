# Copyright 2023 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

option(MUJOCO_DATASERVER_USE_SYSTEM_FLATBUFFERS
  "Use a FlatBuffers installation provided by the host environment" OFF)

if(MUJOCO_DATASERVER_USE_SYSTEM_FLATBUFFERS)
  if(WIN32)
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};D:/vcpkg/installed/x64-windows")
  endif()

  find_package(flatbuffers CONFIG REQUIRED)
  set(MUJOCO_FLATBUFFERS_INCLUDE_DIR "")
else()
  add_subdirectory(_deps/flatbuffers)
  set(MUJOCO_FLATBUFFERS_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/_deps/flatbuffers/include)
endif()

set(MUJOCO_DATASERVER_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

set(MUJOCO_DATASERVER_SRCS
  data_server.cc
  data_server.h
  model_tree.cc
  model_tree.h
  register.cc
  data_type.h
  shm_manager.cc
  shm_manager.h
  shm_server.cc
  shm_server.h
)

add_library(dataserver SHARED)
target_sources(dataserver PRIVATE ${MUJOCO_DATASERVER_SRCS})
target_include_directories(dataserver PRIVATE
  ${MUJOCO_DATASERVER_INCLUDE}
  ${MUJOCO_FLATBUFFERS_INCLUDE_DIR}
)
target_link_libraries(dataserver PRIVATE mujoco)
target_compile_options(
  dataserver
  PRIVATE ${AVX_COMPILE_OPTIONS}
  ${MUJOCO_MACOS_COMPILE_OPTIONS}
  ${EXTRA_COMPILE_OPTIONS}
  ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  dataserver
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)
install(
  TARGETS
  dataserver
  DESTINATION
  "${CMAKE_INSTALL_BINDIR}/mujoco_plugin"
)
add_library(dataclient STATIC)
target_sources(dataclient PRIVATE
  shm_client.cc
  shm_client.h
  ${MUJOCO_DATASERVER_SRCS}
)
target_include_directories(dataclient PRIVATE
  ${MUJOCO_DATASERVER_INCLUDE}
  ${MUJOCO_FLATBUFFERS_INCLUDE_DIR}
)

# target_link_libraries(dataclient PRIVATE ${MUJOCO_FLATBUFFERS_LIBS})
add_executable(shm_client_example)
target_sources(shm_client_example PRIVATE
  shm_client_example.cc
)
target_include_directories(shm_client_example PRIVATE
  ${MUJOCO_DATASERVER_INCLUDE}
)
target_link_libraries(shm_client_example PRIVATE dataclient mujoco)