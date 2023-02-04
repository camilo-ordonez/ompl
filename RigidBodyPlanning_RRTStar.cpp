/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */
// Modified by Camilo Ordonez, Jan 22-2023

#include <ompl/base/SpaceInformation.h>
//#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>


#include <fstream>
#include <array>
#include <vector>

#include<chrono>



namespace ob = ompl::base;
namespace og = ompl::geometric;


std::vector<std::vector<std::array<float,3>>> obstaclesList; // all scenarios
std::vector<std::array<float,3>> obstacles_; //x,y,rad

int num_obst_; // number of obstacles in the map
float robot_rad_ = 0.1;

// function to read CSV file with many obstacle scenarios
// file should be organized by rows. First col is the number of obstacles
// then it goes x_obst y_obst rad_obst

void readObstaclesFromFile(const std::string& filename, std::vector<std::vector<std::array<float,3>>> obstaclesList) {

        std::ifstream myFile(filename);
        if(!myFile.is_open()) 
            throw std::runtime_error("Could not open file");

        std::string line, value;
        while (std::getline(myFile, line)) {

            std::vector<std::array<float,3>> obstacles;
            std::stringstream ss(line);

            std::getline(ss, value, ',');
            int num_obstacles = std::stoi(value);

            for (int obs = 0; obs < num_obstacles; obs++) {

                float x,y,r;

                std::getline(ss, value, ',');
                x = std::stof(value);
                std::getline(ss, value, ',');
                y = std::stof(value);
                std::getline(ss, value, ',');
                r = std::stof(value);

                obstacles.push_back({x, y, r});
            }

            obstaclesList.push_back(obstacles);
        }
    }


void printObstacleList(std::vector<std::vector<std::array<float,3>>> obstaclesList){

    int no_scenarios = 2;
    int no_obst;

    for (int j = 0; j < no_scenarios; j++){
        no_obst = obstaclesList[j].size();
        std::cout<<"num obstacles"<< no_obst << std::endl;

/*
        for (int i = 0; i< num_obst_; i++){
        
            std::cout<<obstacles_[i][0]<<"\t";
            std::cout<<obstacles_[i][1]<<"\t";
            std::cout<<obstacles_[i][2]<<std::endl;
        }
  */  
    }
    
    //obstaclesList[0][0][0]

}


void LoadMap(const std::string& file_name){

    float x_obs, y_obs, rad_obs;
    int i = 0;

    //std::ifstream in(file_name,std::ios_base::in);

    std::ifstream in(file_name,std::ios_base::in);
    
    while(in >> x_obs >> y_obs >> rad_obs){
        obstacles_.push_back({x_obs, y_obs, rad_obs});
        i++;
    }

    num_obst_ = i;

    //obstacles_[0][0] = // x position of first object 
    in.close();

}

void PrintMap(){

    std::cout << "Num Obstacles:" << num_obst_ << std::endl;

    for (int i = 0; i<num_obst_; i++){
        //std::cout<<"test"<<std::endl;
        std::cout<<obstacles_[i][0]<<"\t";
        std::cout<<obstacles_[i][1]<<"\t";
        std::cout<<obstacles_[i][2]<<std::endl;
        //<<std::cout<<obstacles_[i][2]<<std::endl;
        //'\t'<<std::cout<<obstacles_[i][1]<<'\t'<<std::cout<<obstacles_[i][2]<<std::endl; 
    }


}


//bool CheckCollision(const Robot& robot){
bool CheckCollision(const float X, const float Y){
        
        float dx,dy,rad_obst,d;
        bool in_collision;
        
        for(int i = 0; i < num_obst_;i++){
            dx = (X - obstacles_[i][0]);
            dy = (Y - obstacles_[i][1]);
            d = sqrt(dx*dx + dy*dy);
            rad_obst = obstacles_[i][2];

            in_collision = (d < (rad_obst + robot_rad_)) ? true : false;

            if(in_collision) break;

        }

        //std::cout<<robot.rad_<<std::endl;
        //std::cout<<"d\n"<<d<<std::endl;
        //std::cout<<rad_obst + robot.rad_<<std::endl;
        

        return in_collision;


}



ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);


bool isStateValid(const ob::State *state)
{

    float X,Y;
    float heading;
    bool flag = true;

    // cast the abstract state type to the type we expect
    //const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    //const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    //const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot
    X = pos->values[0];
    Y = pos->values[1];
    heading = rot->value;
    //Z = pos->values[2];

    //std::cout << "X: " << X << " Y: " << Y << " Heading: " << heading << std::endl;
    
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings

    flag = CheckCollision(X,Y);
    
    if (flag == true) { // if there is a collision, state is not valid
        return false;
    }
    else{
        return true;
    }
    //return (~CheckCollision(X,Y));

    //return(flag);
    //return (const void*)rot != (const void*)pos;


}






void plan()
{
    // construct the state space we are planning in
    //auto space(std::make_shared<ob::SE3StateSpace>());
    auto space(std::make_shared<ob::SE2StateSpace>());


    // set the bounds for the R^3 part of SE(3)
    //ob::RealVectorBounds bounds(3);
    ob::RealVectorBounds bounds(2);
    
    //bounds.setLow(-1);
    //bounds.setHigh(1);

    bounds.setLow(-10);
    bounds.setHigh(10);



    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    //start.random();

    start[0] = -7.5;
    start[1] = -2.5;
    start[2] = 0.0;


    // create a random goal state
    ob::ScopedState<> goal(space);
    //goal.random();

    goal[0] = 7.5;
    goal[1] = 7.5;
    goal[2] = 0.0;


    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);


    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(getPathLengthObjective(si));


    // create a planner for the defined space
    //auto planner(std::make_shared<og::RRTConnect>(si));
    //auto planner(std::make_shared<og::RRT>(si));
    auto planner(std::make_shared<og::RRTstar>(si));
   

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within the specified runtime

    // start timer
    auto start_time = std::chrono::high_resolution_clock::now();

    ob::PlannerStatus solved = planner->ob::Planner::solve(0.009);

    auto stop_time = std::chrono::high_resolution_clock::now();



    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);


        //Added stuff
        std::cout << "Path cost" << std::endl;

         std::cout  << pdef->getSolutionPath()->length();
         // print result to screen
         //std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(std::cout);
         
         // print result to file
          std::filebuf fb;
          fb.open ("result.txt",std::ios::out);
          std::ostream os(&fb);
          std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(os);
         
  
          fb.close();

          // time duration
          auto time_duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_time-start_time);

          // Print duration
          std::cout<< "Time taken by planner:" << time_duration.count() << " microseconds" << std::endl;


    }
    else
        std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetup()
{
    // construct the state space we are planning in
    //auto space(std::make_shared<ob::SE3StateSpace>());
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    //ob::RealVectorBounds bounds(3);
    ob::RealVectorBounds bounds(2);
    
    bounds.setLow(-10.0);
    bounds.setHigh(10.0);

    //bounds.setLow(-6.0);
    //bounds.setHigh(6.0);


    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    // create a random start state
    ob::ScopedState<> start(space);
    //start.random();
    
    
    start[0] = 1.0;
    start[1] = 1.0;
    start[2] = 0.0;
    
    //std::cout << " start collision" <<  CheckCollision(start[0], start[1]) << std::endl;


    
    // create a random goal state
    ob::ScopedState<> goal(space);
    //goal.random();
    
    
    goal[0] = 6.0;
    goal[1] = 6.0;
    goal[2] = 0.0;
    
    //goal.setY(5.0);
    //goal.setYaw(0.0);
    



    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}


int main(int argc, char *argv[])
{

   std::string scenarios_filename = "/home/camilo/Documents/omplplanning/myplanners_rand/allscenarios.txt";
   readObstaclesFromFile(scenarios_filename,obstaclesList);
   printObstacleList(obstaclesList);

  
/* 
   if(argc <2){
    std::cout << "provide obstacle filename" << std::endl;
    return 1;
   }


   std::string obstacle_filename = argv[1];
   std::string path = "/home/camilo/Documents/omplplanning/myplanners/";
   std::string filename = path + obstacle_filename;

   

   std::cout<<obstacle_filename;
    // load map
    //LoadMap("/home/camilo/Documents/omplplanning/myplanners/obstacles_large.txt");
    //LoadMap("/home/camilo/Documents/omplplanning/myplanners/obstacles_large.txt");
    LoadMap(filename);

    PrintMap();

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    
    plan();
*/
    return 0;
}


/** Returns a structure representing the optimization objective to use
    for optimal motion planning. This method returns an objective
    which attempts to minimize the length in configuration space of
    computed paths. */
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}


