cmake_minimum_required(VERSION 3.14)
project(hpipm_build)

include(FetchContent)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
 

# Required manually-set path to use the previously built blasfeo
set(BLASFEO_PATH ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-install)

set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/hpipm-src)
set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/hpipm-build)

# Fetch HPIPM source
FetchContent_Declare(hpipm
  GIT_REPOSITORY https://github.com/giaf/hpipm
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
  SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
  BINARY_DIR ${HPIPM_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install PREFIX=${CMAKE_CURRENT_BINARY_DIR}/hpipm-install"
)
FetchContent_MakeAvailable(hpipm)

# Import the built library manually
if(NOT TARGET hpipm)
  add_library(hpipm STATIC IMPORTED GLOBAL)
  set_target_properties(hpipm PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/hpipm-install/lib/libhpipm.so
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR}/hpipm-install/include
    INTERFACE_LINK_LIBRARIES blasfeo
  )
endif()

# Create a wrapper target for HpipmInterface code
add_library(hpipm_interface STATIC
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)

target_include_directories(hpipm_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hpipm_interface
  PRIVATE hpipm blasfeo Eigen3::Eigen Threads::Threads OpenMP::OpenMP_CXX ocs2_core ocs2_oc)

# install(TARGETS hpipm_interface DESTINATION lib)

# # Install and export the interface target
# install(TARGETS hpipm_interface
#   EXPORT hpipmTargets
#   DESTINATION lib
# )

# install(DIRECTORY include/ DESTINATION include)

# # Export CMake config (makes find_package(hpipm) work)
# install(EXPORT hpipmTargets
#   FILE hpipmTargets.cmake
#   NAMESPACE hpipm::
#   DESTINATION lib/cmake/hpipm)

# Optional: gtest test (replace with CMake version)
# add_executable(test_hpipm test/testHpipmInterface.cpp)
# target_link_libraries(test_hpipm PRIVATE hpipm_interface gtest_main)