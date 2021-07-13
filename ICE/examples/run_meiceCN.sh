#!/bin/bash

########################################################################################
#
# This is a simple scipt used to run all of the examples on the specified data-set
#
# i.e., if you want to run on data-set 1 with low-quality obs.
# ./run_me.sh 1_lq
#
#
# The list of possible inputs are: {1,2,3}_{lq,hq}   (e.g., 1_lq)
# * The corresponding data is housed in ~/ICE/data/gtsam
########################################################################################

CURRDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd && cd .. )"
DIR="$(dirname "$CURRDIR")"

TDIR="$DIR/test"
BDIR="$DIR/examples/build"
DDIR="$DIR/data"
# GDIR="$DIR/data/gtsam_ice"

# cd $DDIR/conf

## Set-up estimator config file for user's computer
# cp template.conf curr_run.conf
# sed -i "s|LOCATION|$GDIR|" curr_run.conf
# sed -i "s|DATAFILE|$1|" curr_run.conf


## Run all of the estimators
echo "Running ICE/L2/maxmix/DCS .. "
"$BDIR/test_gnss_iceCN"
# | grep "xyz" | awk '{print $2 " " $3 " " $4 " " $5}' > "$TDIR/ice_zupt.xyz"

# -c curr_run.conf
# echo "Running MM"
# "$BDIR/test_gnss_maxmix" -c curr_run.conf | grep "xyz" | awk '{print $2 " " $3 " " $4 " " $5}' > "$TDIR/mm.xyz"
#
# echo "Running DCS"
# "$BDIR/test_gnss_dcs" -c curr_run.conf | grep "xyz" | awk '{print $2 " " $3 " " $4 " " $5}' > "$TDIR/dcs.xyz"
#
# echo "Running L2"
# "$BDIR/test_gnss_l2"
# | grep "xyz" | awk '{print $2 " " $3 " " $4 " " $5}' > "$TDIR/l2.xyz"


## Get the stats. for the runs. NOTE: this requires that you have matlab installed
# export MATLABPATH="$DDIR/truth":"$CURRDIR/make_plots":"$CURRDIR/make_plots/utils"
#
# fileNum=$(echo $1 | cut -d '_' -f1)
# truth_dir="$DDIR/truth/drive_$fileNum.xyz"
# matlab  -nodisplay -nojvm -r "get_stats('"$TDIR"', '"$truth_dir"');exit"
# clear;
# cat "$TDIR/stats.txt"
