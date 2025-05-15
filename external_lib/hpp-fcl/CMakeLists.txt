cmake_minimum_required(VERSION 3.14)
project(hppfcl_builder)

include(ExternalProject)

set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(DOXYGEN_GENERATE_DOC OFF CACHE BOOL "" FORCE)

set(HPPFCL_INSTALL_DIR ${CMAKE_BINARY_DIR}/hppfcl-install)
set(EIGEN_SOURCE_DIR ${CMAKE_BINARY_DIR}/eigen-src)  # must match eigen3 target

ExternalProject_Add(hppfcl_ep
  GIT_REPOSITORY https://github.com/leggedrobotics/hpp-fcl.git
  GIT_TAG master
  PREFIX ${CMAKE_BINARY_DIR}/hppfcl
  CMAKE_CACHE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${HPPFCL_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN_SOURCE_DIR}
    -DHPP_FCL_HAS_QHULL:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=OFF
    -DBUILD_TESTING:BOOL=OFF
    -DBUILD_DOCUMENTATION:BOOL=OFF
    -DDOXYGEN_GENERATE_DOC:BOOL=OFF
    UPDATE_COMMAND ${CMAKE_COMMAND} -E echo "Removing doc build line" &&
    ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/hppfcl/src/hppfcl_ep/CMakeLists.txt ${CMAKE_BINARY_DIR}/hppfcl/src/hppfcl_ep/CMakeLists.txt.bak &&
    sed -i.bak "/add_subdirectory(doc)/d" ${CMAKE_BINARY_DIR}/hppfcl/src/hppfcl_ep/CMakeLists.txt
  LOG_CONFIGURE OFF
  LOG_BUILD OFF
  LOG_INSTALL OFF
)

add_library(hppfcl::hppfcl STATIC IMPORTED GLOBAL)
add_dependencies(hppfcl::hppfcl hppfcl_ep)

set_target_properties(hppfcl::hppfcl PROPERTIES
  IMPORTED_LOCATION ${HPPFCL_INSTALL_DIR}/lib/libhpp-fcl.a
  INTERFACE_INCLUDE_DIRECTORIES ${HPPFCL_INSTALL_DIR}/include
  INTERFACE_LINK_LIBRARIES eigen3::eigen
)
