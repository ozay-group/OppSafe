# Opportunistic Safety Outside the Maximal RCIS
Authors: Zexiang Liu, Hao Chen, Yulong Gao, Necmiye Ozay

## Dependency
### MATLAB
- [MPT3](https://www.mpt3.org/)
- [PCIS](https://github.com/pettni/pcis)
- [GUROBI](https://www.gurobi.com/)

### Python (only if you want to run drone experiments)
- Numpy
- Scipy
- Matplotlib
- CVXPY
- [cflib](https://github.com/bitcraze/crazyflie-lib-python)

## Reproduce the experimental results in the paper
### Environment Setup (required before running any examples)
Open MATLAB. Run the following prompts in the workspace of MATLAB:
```
# Set up GUROBI
cd 'directory_of_gurobi/linux64/matlab'
gurobi_setup

# Set up PCIS
cd 'directory_of_pcis'
addpath(genpath('pcis/lib/'));

# Initialize MPT3
cd 'directory_of_tbxmanager' # tbxmanager is installed with MPT3
startup
clc;
mpt_init

# Set up our code
cd 'directory_of_this_repo'
initial
```
### Car Following Example
Run the following prompts in the workspace of MATLAB:
```
cd 'directory_of_this_repo/experiments/car_following'
open run_experiments.m
```
Then, follow the instructions in `run_experiments.m` to reproduce the results.

### Lane Keeping Example
Run the following prompts in the workspace of MATLAB:
```
cd 'directory_of_this_repo/experiments/lk'
open run_experiments.m
```
Then, follow the instructions in `run_experiments.m` to reproduce the results.

### Safe Tracking for Aerial Vehicles
We use [Crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) for the experiments.

#### Preparation
To implement the safety supervisors, we need to identify a model of the drone from data, and then compute the maximal RCIS. 

For system identification, run the MATLAB script `drone_sys_id.m` under the directory `OppSafe/experiments/drone`. The result is saved to `OppSafe/experiments/data/drone_dyn_3d.mat`.

For RCIS computation, run the MATLAB script `compute_cis_drone.m` under the directory `OppSafe/experiments/drone`. The result is saved to `OppSafe/experiments/data/alpha_filter_py.mat`.

We included our precomputed results in the folder `OppSafe/experiments/drone/data`.


#### Run the experiment
Make sure you install all the Python dependencies and connect your laptop to the drone. 
Ensure that there are no obstacles within a 2-meter radius of the drone.

In a terminal (we only test the code in Ubuntu), navigate to the folder `OppSafe/experiments/drone` and run the Python script `drone_landmark_vel.py`. 
If everything works well, the drone should take off and start tracking an "M"-shape reference trajectory. Press `Ctrl-C` in the terminal for emergency stop.

#### Visualization
In a terminal, navigate to the folder `OppSafe/experiments/drone` and run the Python script `generate_plot.py`.
If you run your own experiments, make sure you modify the data file name in the script to yours.
