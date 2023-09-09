FROM mcr.microsoft.com/devcontainers/base:ubuntu

# git already installed
# docker already installed
# python already installed
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get install -y cmake
RUN apt-get install -y pip
RUN pip install clang-format
RUN pip install pre-commit

# *** need to manually initialize pre-commit ***
# pre-commit install
# pre-commit run

# *** need to manually create build directory and initialize cross-compiler ***
# mkdir build
# cd build
# ../utils/toolchain.sh rpi3-armv8 --pull --export
# cmake -DCMAKE_TOOLCHAIN_FILE=../utils/armv8-rpi3-linux-gnueabihf.cmake -DCMAKE_BUILD_TYPE=Debug ..
# make -j`nproc`
