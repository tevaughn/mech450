#include <cmath>
#include <boost/bind.hpp>

// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/base/samplers/UniformValidStateSampler.h>

// Except for the state space definitions and any planners

#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/control/planners/rrt/RRT.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <valarray>
#include <limits>

// The collision checker produced in project 2
#include "CollisionChecking.h"

using namespace ompl;

const int BENCHMARK  = 1;
const int PLANNER = 2;

const int PENDULUM  = 1;
const int CAR = 2;

const int TWISTY  = 1;
const int CUBICLES = 2;

void planWithSimpleSetupCar(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY);

void planWithSimpleSetupPendulum(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY);

void runBenchmarks(int twistycoolORcubicles);

base::ValidStateSamplerPtr allocUniformStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::UniformValidStateSampler(si));
}


int main(int, char **)
{
	// Empty environment
	std::vector<Rectangle> none;

	//Street environment
    std::vector<Rectangle> obstacles;
    Rectangle obstacle;

	//1
    obstacle.x = -6.0;
    obstacle.y = -10.0;
    obstacle.width = 12.0;
    obstacle.height = 4.0;
    obstacles.push_back(obstacle);

	//2
	obstacle.x = -6;
	obstacle.y = -4;
	obstacle.width = 6;
	obstacle.height = 8;
	obstacles.push_back(obstacle);

	//3
	obstacle.x = 2;
	obstacle.y = -4;
	obstacle.width = 4;
	obstacle.height = 8;
	obstacles.push_back(obstacle);

	//4
	obstacle.x = -6;
	obstacle.y = 6;
	obstacle.width = 12;
	obstacle.height = 2;
	obstacles.push_back(obstacle);


	int benchmarkOrPlan;
	do 
	{
        std::cout << "Benchmark (1) or Plan (2) "<< std::endl;
		std::cin >> benchmarkOrPlan;

	} while (benchmarkOrPlan < 1 || benchmarkOrPlan > 2);

	switch (benchmarkOrPlan) 
	{
		case BENCHMARK:
			int benchmarkChoice;
			do
			{
				std::cout << "Benchmark for: "<< std::endl;
				std::cout << " (1) Pendulum" << std::endl;
				std::cout << " (2) Car" << std::endl;

				std::cin >> benchmarkChoice;
			} while (benchmarkChoice < 1 || benchmarkChoice > 2);
				//runBenchmarks(benchmarkChoice);
				break;
		case PLANNER:
			int plannerChoice;
			do
			{
				std::cout << "Plan for: "<< std::endl;
				std::cout << " (1) Pendulum" << std::endl;
				std::cout << " (2) Car" << std::endl;

				std::cin >> plannerChoice;
			} while (plannerChoice < 1 || plannerChoice > 2);

			switch(plannerChoice)
			{
				case PENDULUM:
					std::cout << "Running in empty environment \n";
				    planWithSimpleSetupPendulum(none, -10, 10, -10, 10, -5, -5, 5, 5);
				    break;
				case CAR:
					std::cout << "Running in street like environment\n";
				    planWithSimpleSetupCar(obstacles, -10, 10, -10, 10, -5, -5, 5, 5);
				    break;
			}
	}
    return 0;
}

