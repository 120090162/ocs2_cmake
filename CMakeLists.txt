cmake_minimum_required(VERSION 3.14)
project(ocs2_all LANGUAGES CXX)

# 设置全局 C++ 编译标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 可选：导出 compile_commands.json 给 clangd 用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 所有模块（按依赖顺序）
set(OCS2_MODULES
  ocs2_thirdparty
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_qp_solver
  ocs2_sqp/blasfeo_cmake
  ocs2_sqp/hpipm_cmake
  ocs2_sqp/ocs2_sqp  # 可按需解注
  ocs2_pinocchio/ocs2_pinocchio_interface
  ocs2_pinocchio/ocs2_centroidal_model
)

# 自动添加所有模块
foreach(module ${OCS2_MODULES})
  message(STATUS "Adding subdirectory: ${module}")
  add_subdirectory(${module})
endforeach()
