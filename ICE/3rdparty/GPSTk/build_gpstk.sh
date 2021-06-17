#!/bin/bash

mkdir build
cd build
cmake ../
cd ..
make -s -j 4
sudo make install
sudo ldconfig
