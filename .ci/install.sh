#!/bin/sh
set -ex

APT='
cmake
libeigen3-dev
libboost-all-dev
clang-format-6.0
'

sudo apt-get -qq --yes --force-yes install $APT
