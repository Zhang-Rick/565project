# 565project
This branch of the repo has been set up with Piecewise Linear BP as the default BP.

Find Below the list of modifications done to get the implement the BP

Files Added:
src/cpu/pred/piecewise_linear.cc
src/cpu/pred/piecewise_linear.hh

Files Modified:
src/cpu/pred/BranchPredictor.py	#To declare the BP
src/cpu/o3/O3CPU.py		#To set the BP used by the O3CPU

# Building and Running Simulation

Execute the below shell script to Build Gem5 after modifiaction
```
/make_arm.sh
```

Execute the below shell script to Run the Simulation. For the below command Arg1 id the output location and Arg2 is the name of the benchmark to be run
```
./run_scripts.sh Arg1 Arg2
```

The Accuracy  and CPI information for the run can be found in the BP_stats.txt file in the output directory.
