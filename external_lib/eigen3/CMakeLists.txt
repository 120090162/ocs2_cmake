cmake_minimum_required(VERSION 3.14)
project(eigen3_builder)

include(FetchContent)

set(EIGEN_SOURCE_DIR ${CMAKE_BINARY_DIR}/eigen-src)
set(EIGEN_INSTALL_DIR ${CMAKE_BINARY_DIR}/eigen-install)

FetchContent_Declare(eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 21ae2afd4edaa1b69782c67a54182d34efe43f9c  # 3.3.7
  SOURCE_DIR ${EIGEN_SOURCE_DIR}
)

FetchContent_MakeAvailable(eigen3)

add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
set_target_properties(Eigen3::Eigen PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${EIGEN_SOURCE_DIR}
)

