#!/usr/bin/env bash

set -euxo pipefail

# From
# https://www.openrobots.org/morse/doc/stable/headless.html

MESA_VERSION=18.3.4
INSTALL_DIR=/opt/llvmpipe/lib

BUILD_DIR=/tmp/mesa-build
rm -rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR

wget https://mesa.freedesktop.org/archive/mesa-$MESA_VERSION.tar.xz
tar -xvf mesa-$MESA_VERSION.tar.xz
cd mesa-$MESA_VERSION
scons build=release llvm=yes libgl-xlib

rm -rf $INSTALL_DIR
mkdir -p $INSTALL_DIR

cp build/linux-x86_64/gallium/targets/libgl-xlib/* $INSTALL_DIR

rm -rf $BUILD_DIR
