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

#include <ompl/base/SpaceInformation.h>
//#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>


#include <fstream>
#include <array>
#include <vector>



namespace ob = ompl::base;
namespace og = ompl::geometric;

std::vector<std::array<float,3>> obstacles_; //x,y,rad
int num_obst_; // number of obstacles in the map
float robot_rad_ = 0.001;


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







/*
double getX() const
                 {
                     return as<RealVectorStateSpace::StateType>(0)->values[0];
                 }
  
                 double getY() const
                 {
                     return as<RealVectorStateSpace::StateType>(0)->values[1];
                 }
  
                 double getZ() const
                 {
                     return as<RealVectorStateSpace::StateType>(0)->values[2];

*/

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
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    //auto planner(std::make_shared<og::RRTConnect>(si));
    //auto planner(std::make_shared<og::RRT>(si));
    auto planner(std::make_shared<og::RRTSTAR>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within the specified runtime
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);


        //Added stuff
         //std::cout  << pdef->getSolutionPath()->length();
         std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(std::cout);
         //pdef->getSolutionPath()->printAsMatrix(std::cout);

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

int main(int /*argc*/, char ** /*argv*/)
{

    // load map
    //LoadMap("/home/camilo/Documents/modern_cpp/myMap/obstacles.txt"); // create map with the obstacle file
    LoadMap("/home/camilo/Documents/omplplanning/myplanners/obstacles.txt");

    PrintMap();

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    //plan();

    //std::cout << std::endl << std::endl;
    //std::cout << " start collision" <<  CheckCollision(1.0, 1.0) << std::endl;

    //planWithSimpleSetup();
    plan();

    return 0;
}
