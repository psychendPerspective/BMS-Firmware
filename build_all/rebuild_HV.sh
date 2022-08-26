#!/bin/bash
set -e

FWPATH="../"
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

COPYDIR=XANADU_HV_EV
rm -f $COPYDIR/*

cd $FWPATH
make clean
make build_args='-DXANADU_HV_EV=1'
cd $DIR
cp $FWPATH/main.bin $COPYDIR/XANADU_HV_EV-BMS.bin
cp $FWPATH/main.elf $COPYDIR/XANADU_HV_EV-BMS.elf

# Clean
cd $FWPATH
make clean
cd $DIR