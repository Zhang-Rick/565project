# 565project
As for prophet-critic branch predictor, this repository has set the prophet-critic hybrid branch predictor as the default branch predictor. If you want to check the performance of prophet branch predictor by itself, you need to modify the following file.
```
vim src/cpu/o3/O3CPU.py  
```
find out the code blow and replace prophet_criticsBP with prophet_criticsoldBP
```python
branchPred = Param.BranchPredictor(prophet_criticsBP(numThreads =
Parent.numThreads),
"Branch Predictor")
```
and then compile with following code
```
scons-3 -j 4 ./build/ARM/gem5.opt
```
# 2.run for simulation
```
./build/ARM/gem5.opt -d my_outputs configs/spec2k6/run.py -b calculix --maxinsts=1000000000 --cpu-type=DerivO3CPU --l1d_size=64kB --l1i_size=16kB --caches --l2cache
```
The prophet and prophet-critic hybrid is available in src/cpu/pred directory.

check data
please fo to 565projectdata folder to check the data for prophet and prophet-critic hybrid
and datatable in excel