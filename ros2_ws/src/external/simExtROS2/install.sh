#!/bin/sh

set -e

cd "`dirname "$0"`"

PLUGIN_NAME="$(basename "$(pwd)")"

if [ "x$COPPELIASIM_ROOT_DIR" = "x" ]; then
    # see if we can determine it automatically
    # (i.e. the plugin dir is in $COPPELIASIM_ROOT_DIR/programming/$dir)
    COPPELIASIM_ROOT_DIR="$(cd ../.. ; pwd)"
    if [ ! -d "$COPPELIASIM_ROOT_DIR/programming/include" ]; then
        echo "error: \$COPPELIASIM_ROOT_DIR is not set" 1>&2
        exit 1
    fi
fi

if [ "`uname`" = "Darwin" ]; then
    INSTALL_TARGET="$COPPELIASIM_ROOT_DIR/coppeliaSim.app/Contents/MacOS/"
    DLEXT=dylib
else
    INSTALL_TARGET="$COPPELIASIM_ROOT_DIR"
    DLEXT=so
fi

if [ "x$BUILD_TARGET" = "x" ]; then
    BUILD_TARGET=debug
fi

LIBRARY="lib$PLUGIN_NAME.$DLEXT"

if [ -f CMakeLists.txt ]; then
    # plugin uses cmake
    mkdir -p build
    cd build
    cmake ..
    cmake --build .
    cd ..
    LIBRARY="build/$LIBRARY"
elif [ -f $PLUGIN_NAME.pro ]; then
    # plugin uses qmake
    qmake $PLUGIN_NAME.pro
    make $BUILD_TARGET
elif [ -f makefile ]; then
    # plugin uses make
    make $BUILD_TARGET
else
    echo "Unable to figure out the build system of $PLUGIN_NAME"
    exit 1
fi

cp -v "$LIBRARY" "$INSTALL_TARGET"
if [ -f *.lua ]; then cp -v *.lua "$COPPELIASIM_ROOT_DIR/lua/"; fi

