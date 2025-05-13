cmake_minimum_required(VERSION 3.14)
project(ocs2_comm_wrapper)

# 构建头文件接口库
add_library(${PROJECT_NAME} INTERFACE)

# 指定头文件路径（INTERFACE）
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 安装 include 文件夹
install(DIRECTORY include/ DESTINATION include)

# 导出 CMake 包配置
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ocs2::
  DESTINATION lib/cmake/${PROJECT_NAME})
