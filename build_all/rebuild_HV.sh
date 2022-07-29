#!/bin/bash
set -e

FWPATH="../"
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

COPYDIR=ENNOID-HV
rm -f $COPYDIR/*

cd $FWPATH
make clean
make build_args='-DENNOID_HV=1'
cd $DIR
cp $FWPATH/main.bin $COPYDIR/ENNOID-BMS.bin

# Clean
cd $FWPATH
make clean
cd $DIR