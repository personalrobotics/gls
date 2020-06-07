#!/usr/bin/env bash

set -ex

$SUDO apt-get -qq update
$SUDO apt-get -y install lsb-release software-properties-common
$SUDO apt-get -qq update

# Build tools
$SUDO apt-get -y install \
  sudo \
  build-essential \
  cmake \
  pkg-config \
  curl \
  git

# Required dependencies
$SUDO apt-get -y install \
  libboost-all-dev \
  libompl-dev \
  libeigen3-dev
