cmake_minimum_required(VERSION 3.14)
project(ocs2_qp_solver)

# 系统依赖
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# 编译选项
include(cmake/ocs2_cxx_flags.cmake OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 自动包含所有源文件
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp)

# 创建库
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
  PUBLIC ocs2_core
  PUBLIC ocs2_oc
  PUBLIC Eigen3::Eigen
  PUBLIC Threads::Threads
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  test/include
)

# Clang Tooling（可选）
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif()

# 测试
# enable_testing()
# add_executable(test_${PROJECT_NAME}
#   test/testDiscreteTranscription.cpp
#   test/testQpSolver.cpp
#   test/testOcs2QpSolver.cpp
# )
# target_link_libraries(test_${PROJECT_NAME}
#   PRIVATE ${PROJECT_NAME}
#   PRIVATE GTest::gtest_main
# )
# add_test(NAME test_${PROJECT_NAME} COMMAND test_${PROJECT_NAME})

# 安装（可选）
# install(TARGETS ${PROJECT_NAME} DESTINATION lib)
# install(DIRECTORY include/ DESTINATION include)
# install(DIRECTORY test/include/ DESTINATION include)
