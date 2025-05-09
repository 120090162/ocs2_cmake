#!/bin/bash
set -e  # stop on error

# You can later make this configurable
INSTALL_PREFIX=$(pwd)/install
BUILD_ROOT=$(pwd)/build_workspace

mkdir -p "$INSTALL_PREFIX"
mkdir -p "$BUILD_ROOT"

# Ordered list of packages (you maintain the dependency order)
PACKAGES=(
  ocs2_thirdparty
  ocs2_core
  # ocs2_oc
)

for pkg in "${PACKAGES[@]}"; do
  echo "===== Building $pkg ====="
  mkdir -p "$BUILD_ROOT/$pkg"
  cmake -S "$pkg" -B "$BUILD_ROOT/$pkg" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"
  cmake --build "$BUILD_ROOT/$pkg" -j$(nproc)
  cmake --install "$BUILD_ROOT/$pkg"
  echo "===== Successfully built $pkg ====="
done

# for pkg in "${PACKAGES[@]}"; do
#   echo "===== Building $pkg ====="
#   mkdir -p "$BUILD_ROOT/$pkg"

#   # Add -DCMAKE_PREFIX_PATH only for packages after the first
#   if [ "$pkg" == "ocs2_thirdparty" ]; then
#     cmake -S "$pkg" -B "$BUILD_ROOT/$pkg" \
#       -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"
#   else
#     cmake -S "$pkg" -B "$BUILD_ROOT/$pkg" \
#       -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
#       -DCMAKE_PREFIX_PATH="$INSTALL_PREFIX"
#   fi

#   cmake --build "$BUILD_ROOT/$pkg" -j$(nproc)
#   cmake --install "$BUILD_ROOT/$pkg"
#   echo "===== Successfully built $pkg ====="
# done


echo "✅ All packages built and installed to: $INSTALL_PREFIX"
