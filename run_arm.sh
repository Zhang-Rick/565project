#!/bin/sh
if [ $# = 2 ]
then
	./build/ARM/gem5.opt -d $1 configs/spec2k6/run.py  -b $2 --maxinsts=1000000 --cpu-type=DerivO3CPU --l1d_size=64kB --l1i_size=16kB --caches --l2cache
	exit 1
fi

./build/ARM/gem5.opt configs/spec2k6/run.py -b bzip2 --maxinsts=1000000 --cpu-type=DerivO3CPU --l1d_size=64kB --l1i_size=16kB --caches --l2cache
