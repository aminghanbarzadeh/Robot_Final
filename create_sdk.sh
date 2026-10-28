#!/bin/bash
# Script to create the IUST_SDK folder from the current codebase

SDK_DIR="IUST_SDK"

echo "Creating $SDK_DIR directory structure..."
mkdir -p $SDK_DIR/include
mkdir -p $SDK_DIR/lib
mkdir -p $SDK_DIR/user/CustomController

echo "Copying include headers..."
cp -r common/include/* $SDK_DIR/include/
cp -r robot/include/* $SDK_DIR/include/
cp -r third-party/* $SDK_DIR/include/
cp -r lcm-types/cpp/* $SDK_DIR/include/

# Clean up any non-header files from third-party includes
find $SDK_DIR/include/ -type f -not -name "*.h" -and -not -name "*.hpp" -and -not -name "*.hh" -delete
# Keep some CMake files if needed by libraries or just delete all non headers
# Wait, some third party might have internal cpp we don't want to distribute
# We only need the headers to build the custom controller.

echo "Copying compiled libraries..."
# The main libraries built are libbiomimetics.a, librobot.so, libinih.so, libdynacore_param_handler.so, etc.
# We will search for all .a and .so in the build directory and copy them
find build -name "*.a" -exec cp {} $SDK_DIR/lib/ \;
find build -name "*.so" -exec cp {} $SDK_DIR/lib/ \;
find build -name "*.so.*" -exec cp {} $SDK_DIR/lib/ \;

echo "Copying CustomController template..."
cp -r user/CustomController/* $SDK_DIR/user/CustomController/

echo "Copying SDK CMakeLists.txt template..."
cp scripts/sdk_template_CMakeLists.txt $SDK_DIR/CMakeLists.txt

echo "SDK creation complete in $SDK_DIR!"
