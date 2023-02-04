# RRSTAR
 RigidBodyPlanning_RRTStar.cpp

# Build code
cd build
make

# To run code
./planers_bench

# required files
allscenarios.txt -> file organized by rows. First column has num obstacles and then x_obs, y_obs, rad_obs

times.txt -> file with one column with the time it took SBMPO to solve each scenario

RRTStar will run each scenario for the runtime associated with the scenario

The code will produce the following files:

costs_results.txt -> file with one column containing the RRT* path cost of each scenario

result.txt -> resulting path for the last scenario

# Visualizing results
The planner generates a txt file with the results (results.txt)

cd build
python3 visualizeresult.py 

you should makesure that the visualizer is using the correct file name for the obstacle file

