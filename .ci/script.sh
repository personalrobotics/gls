#!/bin/sh
set -ex

if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$COMPILER" = "CLANG" ]; then exit; fi

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DGLS_TREAT_WARNINGS_AS_ERRORS=$GLS_TREAT_WARNINGS_AS_ERRORS ..
make -j4 all
make -j4 test

# Make sure we can install with no issues
$SUDO make install
