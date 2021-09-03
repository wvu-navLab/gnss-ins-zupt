#!/bin/bash

########################################################################################
#
# This is a simple scipt used to run all of the examples on the specified data-set
#
########################################################################################

CURRDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd && cd .. )"
DIR="$(dirname "$CURRDIR")"

TDIR="$DIR/test"
BDIR="$DIR/examples/build"
DDIR="$DIR/data"


## Run all of the estimators
echo "Running L2 "

"$BDIR/test_gnss_l2"
