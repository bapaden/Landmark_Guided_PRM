#!/bin/bash

if [ ! -d 'build' ]; then
  mkdir build
  echo 'Creating a build directory'
fi

cd build && cmake .. && make
