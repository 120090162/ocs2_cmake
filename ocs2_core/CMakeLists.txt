cmake_minimum_required(VERSION 3.14)
project(ocs2_core)

# ---- 依赖包 ----
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(Boost_NO_BOOST_CMAKE ON)  # 强制使用旧风格 MODULE 查找
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

 

# ---- 编译设置 ----
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/ocs2_cxx_flags.cmake)
message(STATUS "OCS2_CXX_FLAGS: ${OCS2_CXX_FLAGS}")

# ---- 头文件路径 ----
include_directories(
  include
  test/include
  # ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ---- 源文件 ----
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)

# ---- 库目标 ----
add_library(${PROJECT_NAME} ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME}
  PUBLIC ocs2_thirdparty
  PUBLIC Eigen3::Eigen
  PUBLIC ${Boost_LIBRARIES}
  PUBLIC OpenMP::OpenMP_CXX
  PUBLIC Threads::Threads
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- lintTarget 可执行程序 ----
add_executable(${PROJECT_NAME}_lintTarget src/lintTarget.cpp)
target_link_libraries(${PROJECT_NAME}_lintTarget PRIVATE ${PROJECT_NAME})

# ---- clang tooling ----
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif()

# ---- 安装 ----
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ocs2::
  DESTINATION lib/cmake/${PROJECT_NAME})