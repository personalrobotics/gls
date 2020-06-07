#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
. "${TRAVIS_BUILD_DIR}/.ci/install_catkin.sh"
else
. "${TRAVIS_BUILD_DIR}/.ci/install_cmake.sh"
fi
