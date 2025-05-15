cmake_minimum_required(VERSION 3.0.2)
project(ocs2_legged_robot_test)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CATKIN_PACKAGE_DEPENDENCIES
#   roslib
#   cmake_modules
#   tf
#   urdf
#   kdl_parser
#   robot_state_publisher
#   ocs2_core
#   ocs2_oc
#   ocs2_ddp
#   ocs2_mpc
#   ocs2_sqp
#   ocs2_ipm
#   ocs2_robotic_tools
#   ocs2_pinocchio_interface
#   ocs2_centroidal_model
#   ocs2_robotic_assets
#   ocs2_msgs
#   ocs2_ros_interfaces
#   ocs2_legged_robot
# )

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(urdfdom REQUIRED)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(pinocchio REQUIRED pinocchio)

include_directories(include)

###########
## Build ##
###########

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
  src/synchronized_module/ReferenceReceiver.cpp
  src/gait/GaitReceiver.cpp
)
target_link_libraries(legged_robot_sqp_mpc
  PUBLIC pinocchio
  PUBLIC ocs2_robotic_tools
  PUBLIC ocs2_core
  PUBLIC ocs2_oc
  PUBLIC ocs2_mpc
  PUBLIC ocs2_qp_solver
  PUBLIC ocs2_sqp
  PUBLIC blasfeo
  PUBLIC hpipm
  PUBLIC hpipm_interface
  PUBLIC Eigen3::Eigen
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC ocs2_pinocchio_interface
  PUBLIC ocs2_centroidal_model
  PUBLIC ocs2_legged_robot
  PUBLIC ocs2_comm_wrapper
  PUBLIC urdfdom_model
  PUBLIC urdfdom_model_state
  PUBLIC urdfdom_world
  PUBLIC console_bridge
  # dl
)
target_compile_options(legged_robot_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

