#!/bin/bash
set -e  # stop on error

# You can later make this configurable
INSTALL_PREFIX=$(pwd)/install
BUILD_ROOT=$(pwd)/build_workspace

rm -rf "$INSTALL_PREFIX"
rm -rf "$BUILD_ROOT"