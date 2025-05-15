cmake_minimum_required(VERSION 3.14)
project(pinocchio_builder)

include(ExternalProject)

set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(DOXYGEN_GENERATE_DOC OFF CACHE BOOL "" FORCE)

set(PINOCCHIO_INSTALL_DIR ${CMAKE_BINARY_DIR}/pinocchio-install)
set(HPPFCL_INSTALL_DIR ${CMAKE_BINARY_DIR}/hppfcl-install)
set(EIGEN_SOURCE_DIR ${CMAKE_BINARY_DIR}/eigen-src)

ExternalProject_Add(pinocchio_ep
  GIT_REPOSITORY https://github.com/leggedrobotics/pinocchio.git
  GIT_TAG master
  PREFIX ${CMAKE_BINARY_DIR}/pinocchio
  DEPENDS hppfcl_ep
  CMAKE_CACHE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${PINOCCHIO_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN_SOURCE_DIR}
    -DHPP_FCL_INCLUDE_DIR:PATH=${HPPFCL_INSTALL_DIR}/include
    -DHPP_FCL_LIBRARY:PATH=${HPPFCL_INSTALL_DIR}/lib/libhpp-fcl.a
    -DCMAKE_PREFIX_PATH:PATH=${HPPFCL_INSTALL_DIR}
    -DBUILD_WITH_COLLISION_SUPPORT:BOOL=ON
    -DBUILD_WITH_URDF_SUPPORT:BOOL=ON
    -DBUILD_PYTHON_INTERFACE:BOOL=OFF
    -DBUILD_TESTING:BOOL=OFF
    -DDOXYGEN_GENERATE_DOC:BOOL=OFF
    -DBUILD_DOCUMENTATION:BOOL=OFF
    -DBUILD_DOC:BOOL=OFF
    UPDATE_COMMAND ${CMAKE_COMMAND} -E echo "Removing doc build line" &&
    ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/pinocchio/src/pinocchio_ep/CMakeLists.txt ${CMAKE_BINARY_DIR}/pinocchio/src/pinocchio_ep/CMakeLists.txt.bak &&
    sed -i.bak "/add_subdirectory(doc)/d" ${CMAKE_BINARY_DIR}/pinocchio/src/pinocchio_ep/CMakeLists.txt
  LOG_CONFIGURE OFF
  LOG_BUILD OFF
  LOG_INSTALL OFF
)

add_library(pinocchio::pinocchio STATIC IMPORTED GLOBAL)
add_dependencies(pinocchio::pinocchio pinocchio_ep)

set_target_properties(pinocchio::pinocchio PROPERTIES
  IMPORTED_LOCATION ${PINOCCHIO_INSTALL_DIR}/lib/libpinocchio.a
  INTERFACE_INCLUDE_DIRECTORIES ${PINOCCHIO_INSTALL_DIR}/include
  INTERFACE_LINK_LIBRARIES "hppfcl::hppfcl;eigen3::eigen"
)
