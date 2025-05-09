cmake_minimum_required(VERSION 3.0.2)
project(ocs2_cmake)


add_subdirectory(ocs2_thirdparty)
add_subdirectory(ocs2_core)
add_subdirectory(ocs2_oc)
add_subdirectory(ocs2_mpc)
add_subdirectory(ocs2_robotic_tools)
add_subdirectory(ocs2_qp_solver)
add_subdirectory(ocs2_sqp/blasfeo_cmake)
add_subdirectory(ocs2_sqp/hpipm_cmake)
# add_subdirectory(ocs2_sqp/ocs2_sqp)