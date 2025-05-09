cmake_minimum_required(VERSION 3.14)
project(ocs2_oc)

# 外部依赖
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem)
# find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# 编译选项（可选）
include(cmake/ocs2_cxx_flags.cmake OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 头文件路径
include_directories(
  include
  test/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 添加源文件
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp)

# 主库
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
  PUBLIC ocs2_core
  PUBLIC Threads::Threads
  PUBLIC Eigen3::Eigen
  PUBLIC ${Boost_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC include)


# lintTarget
add_executable(${PROJECT_NAME}_lintTarget src/lintTarget.cpp)
target_link_libraries(${PROJECT_NAME}_lintTarget PRIVATE ${PROJECT_NAME})

# # Clang tooling（可选）
# find_package(cmake_clang_tools QUIET)
# if(cmake_clang_tools_FOUND)
#   add_clang_tooling(
#     TARGETS ${PROJECT_NAME}_lintTarget
#     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
#     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
#     CF_WERROR
#   )
# endif()

# 单元测试
# enable_testing()

# macro(add_ocs2_test name)
#   add_executable(${name} ${ARGN})
#   target_link_libraries(${name} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
#   add_test(NAME ${name} COMMAND ${name})
# endmacro()

# add_ocs2_test(test_multiple_shooting
#   test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
#   test/multiple_shooting/testTranscriptionMetrics.cpp
#   test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
# )

# add_ocs2_test(test_oc_data
#   test/oc_data/testTimeDiscretization.cpp
# )

# add_ocs2_test(test_rollout
#   test/rollout/testTimeTriggeredRollout.cpp
#   test/rollout/testStateTriggeredRollout.cpp
# )

# add_ocs2_test(test_change_of_variables
#   test/testChangeOfInputVariables.cpp
# )

# add_ocs2_test(test_trajectory_spreading
#   test/trajectory_adjustment/TrajectorySpreadingTest.cpp
# )

# add_ocs2_test(test_ocp_to_kkt
#   test/oc_problem/testOcpToKkt.cpp
# )

# add_ocs2_test(test_precondition
#   test/precondition/testPrecondition.cpp
# )

# 安装（可选，仅对外发布时使用）
# install(TARGETS ${PROJECT_NAME} DESTINATION lib)
# install(DIRECTORY include/ DESTINATION include)
