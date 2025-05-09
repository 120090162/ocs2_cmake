cmake_minimum_required(VERSION 3.14)
project(blasfeo_build)

include(FetchContent)

set(BLASFEO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-src)
set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-build)
set(BLASFEO_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-install)

# Fetch the source code
FetchContent_Declare(blasfeo
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
  SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
  BINARY_DIR ${BLASFEO_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install PREFIX=${BLASFEO_INSTALL_DIR}"
)

FetchContent_MakeAvailable(blasfeo)

# Import the built library manually
if(NOT TARGET blasfeo)
  add_library(blasfeo STATIC IMPORTED GLOBAL)
  set_target_properties(blasfeo PROPERTIES
    IMPORTED_LOCATION ${BLASFEO_INSTALL_DIR}/lib/libblasfeo.so
    INTERFACE_INCLUDE_DIRECTORIES ${BLASFEO_INSTALL_DIR}/include
  )
endif()

# Export to install location for external usage
install(FILES
  ${BLASFEO_INSTALL_DIR}/lib/libblasfeo.so
  DESTINATION lib)

install(DIRECTORY ${BLASFEO_INSTALL_DIR}/include/
  DESTINATION include)
