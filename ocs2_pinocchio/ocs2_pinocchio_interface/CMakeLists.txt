cmake_minimum_required(VERSION 3.0.2)
project(ocs2_pinocchio_interface)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Threads REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CATKIN_PACKAGE_DEPENDENCIES
#   ocs2_core
#   ocs2_robotic_tools
# )

# find_package(catkin REQUIRED COMPONENTS
#   ${CATKIN_PACKAGE_DEPENDENCIES}
# )

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(pinocchio REQUIRED pinocchio)

# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add pinocchio configurations
include(cmake/pinocchio_config.cmake)

###################################
## catkin specific configuration ##
###################################

# catkin_package(
#   INCLUDE_DIRS
#     include
#     ${pinocchio_INCLUDE_DIRS}
#   LIBRARIES
#     ${PROJECT_NAME}
#   CATKIN_DEPENDS
#     ${CATKIN_PACKAGE_DEPENDENCIES}
#   DEPENDS
#     pinocchio
#   CFG_EXTRAS
#     pinocchio_config.cmake
# )

###########
## Build ##
###########

# include_directories(
#   include
#   # ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   ${pinocchio_INCLUDE_DIRS}
# )

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
  src/PinocchioInterface.cpp
  src/PinocchioInterfaceCppAd.cpp
  src/PinocchioEndEffectorKinematics.cpp
  src/PinocchioEndEffectorKinematicsCppAd.cpp
  src/urdf.cpp
)
# add_dependencies(${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )
target_link_libraries(${PROJECT_NAME}
  # ${catkin_LIBRARIES}
  # PUBLIC ${pinocchio_LIBRARIES}
  PUBLIC pinocchio
  PUBLIC ocs2_robotic_tools
  PUBLIC ocs2_core
  PUBLIC ocs2_oc
  PUBLIC ocs2_mpc
  PUBLIC ocs2_qp_solver
  PUBLIC blasfeo
  PUBLIC hpipm
  PUBLIC hpipm_interface
  PUBLIC Eigen3::Eigen
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  PUBLIC Threads::Threads
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

# ####################
# ## Clang tooling ###
# ####################

# find_package(cmake_clang_tools QUIET)
# if (cmake_clang_tools_FOUND)
#   message(STATUS "Run clang tooling")
#   add_clang_tooling(
#     TARGETS ${PROJECT_NAME}
#     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
#     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
#     CF_WERROR
#   )
# endif (cmake_clang_tools_FOUND)

# #############
# ## Install ##
# #############

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# catkin_add_gtest(testPinocchioInterface
#   test/testPinocchioInterface.cpp
#   test/testPinocchioEndEffectorKinematics.cpp
# )
# target_link_libraries(testPinocchioInterface
#   gtest_main
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
