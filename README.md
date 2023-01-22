# RRSTAR
 RigidBodyPlanning_RRTStar.cpp

# Build code
cd build
make

# To run code
./planner ostacles.txt

RRTStar will run for a set runtime

where the obstacle files are .txt files with the x y and radii of the circular objects ( 3 columns)

# Visualizing results
The planner generates a txt file with the results (results.txt)

cd build
python3 visualizeresult.py 

you should makesure that the visualizer is using the correct file name for the obstacle file

