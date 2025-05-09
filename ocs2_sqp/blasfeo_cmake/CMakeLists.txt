cmake_minimum_required(VERSION 3.14)
project(blasfeo_build)

include(FetchContent)

# 设置 install 路径（可用于其他 target 导入）
set(BLASFEO_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-install)

set(BLASFEO_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)

# 下载并构建 blasfeo
FetchContent_Declare(blasfeo
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-src
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-build
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install PREFIX=${BLASFEO_INSTALL_DIR}"
)

FetchContent_MakeAvailable(blasfeo)

# 🔁 不要手动 add_library(blasfeo)，直接使用 upstream 提供的 target 即可
