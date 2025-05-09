cmake_minimum_required(VERSION 3.14)  # use 3.14+ for good export support
project(ocs2_thirdparty)

# Define the interface target
add_library(${PROJECT_NAME} INTERFACE)

# Include directory for both build and install
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Export the target to be usable via find_package(ocs2_thirdparty)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ocs2::
  DESTINATION lib/cmake/${PROJECT_NAME}
)