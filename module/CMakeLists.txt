# Copyright 2023 Viam Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

set(CMAKE_PROJECT_VERSION 0.0.1)

# Identify the project.
project(dmtxscanreadmodule
  VERSION ${CMAKE_PROJECT_VERSION}
  DESCRIPTION "Data Matrix Scan Read Module"
  HOMEPAGE_URL https://github.com/stevebriskin/dmtx-scanread
  LANGUAGES C CXX
)

# Instead of setting ENV variables, add these as compile definitions
add_definitions(-DMAGICKCORE_QUANTUM_DEPTH=16)
add_definitions(-DMAGICKCORE_HDRI_ENABLE=1)

set(viam-cpp-sdk_ROOT "/Users/stevebriskin/workspace/viam-cpp-sdk/build/install")
find_package(viam-cpp-sdk REQUIRED)

# Find ImageMagick dependency
set(ImageMagick_ROOT "/opt/homebrew/opt/imagemagick@7/")
find_package(ImageMagick 7.0 COMPONENTS MagickWand REQUIRED)
include_directories(${ImageMagick_INCLUDE_DIRS})

# Find libdmtx dependency
set(dmtx_ROOT "/opt/homebrew/opt/libdmtx/")
find_package(PkgConfig REQUIRED)
pkg_check_modules(DMTX REQUIRED libdmtx)
include_directories(${DMTX_INCLUDE_DIRS})
link_directories(${DMTX_LIBRARY_DIRS})

# Create a library from dmtxread.c
add_library(dmtxread STATIC
  ../dmtxread/dmtxread.c
  ../common/dmtxutil.c
)

# Explicitly set C as the language for dmtxread
set_target_properties(dmtxread PROPERTIES
  LINKER_LANGUAGE C
)

# Set include directories for dmtxread if needed
target_include_directories(dmtxread PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../dmtxread
)

add_executable(dmtxscanreadmodule
  main.cpp
)

target_link_libraries(dmtxscanreadmodule
  dmtxread
  viam-cpp-sdk::viamsdk
  ImageMagick::MagickWand
  ${DMTX_LIBRARIES}
)

install(
  TARGETS dmtxscanreadmodule
)
