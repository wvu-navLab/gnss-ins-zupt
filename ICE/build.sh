#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
mkdir -p "$DIR/bin" "$DIR/include" "$DIR/share" "$DIR/test"

NCORES="$(cat /proc/cpuinfo | awk '/^processor/{print $3}' | wc -l)"

# Get data
# if [ ! -d "$DIR/data" ]
# then
#   git clone https://github.com/wvu-navLab/GnssData.git "$DIR/data"
# fi


# Setup Eigen
# if [ -d "$DIR/3rdparty/Eigen/build" ]
# then
#   rm -rf "$DIR/3rdparty/Eigen/build"
# fi
# mkdir "$DIR/3rdparty/Eigen/build"
# cd "$DIR/3rdparty/Eigen/build"
# cmake -DCMAKE_INSTALL_PREFIX:PATH="$DIR" ../
# make install -s -j $NCORES
#
# ln -s "$DIR/include/eigen3" "$DIR/include/Eigen"
#
#
# # Setup LibCluster
# if [ -d "$DIR/3rdparty/LibCluster/build" ]
# then
#   rm -rf "$DIR/3rdparty/LibCluster/build"
# fi
# mkdir "$DIR/3rdparty/LibCluster/build"
# cd "$DIR/3rdparty/LibCluster/build"
# cmake -DCMAKE_INSTALL_PREFIX="$DIR/" -DEIGEN_INCLUDE_DIRS="$DIR/include/Eigen/" ..
# make -s -j $NCORES
# make install
#
# #
# #
# # Setup GTSAM
# if [ -d "$DIR/3rdparty/RobustGNSS/gtsam/build" ]
# then
#   rm -rf "$DIR/3rdparty/RobustGNSS/gtsam/build"
# fi
# mkdir "$DIR/3rdparty/RobustGNSS/gtsam/build"
# cd "$DIR/3rdparty/RobustGNSS/gtsam/build"
# cmake -DGTSAM_USE_SYSTEM_EIGEN="ON" -DGTSAM_EIGEN_INCLUDE_PREFIX="$DIR/include/Eigen/" -DCMAKE_INSTALL_PREFIX="$DIR" ..
# make -s -j $NCORES
# make install
#
#
# # Setup GPSTk
# if [ -d "$DIR/3rdparty/GPSTk/build" ]
# then
#   rm -rf "$DIR/3rdparty/GPSTk/build"
# fi
# # mkdir "$DIR/3rdparty/GPSTk/build"
# cd "$DIR/3rdparty/GPSTk"
# ./build.sh -c -x -e -i "$DIR"


# Setup Examples
if [ -d "$DIR/examples/build" ]
then
  rm -rf "$DIR/examples/build"
fi
cd "$DIR/examples"
./build_examples.sh



echo -e "\n\n\n\n ----------------------------------------------- \n"
echo -e " build done. :-) "
echo -e  "\n ----------------------------------------------- \n\n\n"
