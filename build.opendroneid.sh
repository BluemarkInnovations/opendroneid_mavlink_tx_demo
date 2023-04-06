#!/bin/bash
# (c) Bluemark Innovations BV 
# MIT license

mkdir bin
cd opendroneid-core-c
rm -R build
#git submodule update --init
mkdir build && cd build

cmake ../.
make

