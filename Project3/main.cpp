#include <iostream>
#include <fstream>
#include <cmath>

#include <boost/bind.hpp>

// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

// Except for the state space definitions and any planners
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"
#include "RandomTree.h"

using namespace ompl;

const int BENCHMARK  = 1;
const int PLANNER = 2;

const int TWISTY  = 1;
const int CUBICLES = 2;

const int R2  = 1;
const int SE2 = 2;

// This is our state validitiy checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    // Extract x, y
    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}


// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State* state, double sideLength, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::CompoundState* cstate = state->as<ompl::base::CompoundState>();

    // Extract x, y, theta
    const ompl::base::RealVectorStateSpace::StateType* r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2state->values[0];
    double y = r2state->values[1];

    const ompl::base::SO2StateSpace::StateType* so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    double theta = so2State->value;

    return isValidSquare(x, y, theta, sideLength, obstacles);
}


// This is our state validity checker.  It says every state is valid.
bool stateAlwaysValid(const ompl::base::State* /*state*/)
{
    return true;
}


void planWithSimpleSetup(const std::vector<Rectangle>& obstacles,  int low, int high, double startX, double startY, double goalX, double goalY, int SE2orR2)
{
    // Step 1) Create the state (configuration) space for your system
    // In this instance, we will plan for a square of side length 0.3
    // that both translates and rotates in the plane.
    // The state space can be easily composed of simpler state spaces
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(low); // x and y have a minimum of -2
    bounds.setHigh(high); // x and y have a maximum of 2

    // Cast the r2 pointer to the derived type, then set the bounds
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    
    if( SE2orR2 == SE2 ) {
        ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
        space = space + so2;
    }

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem using OMPL.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(space);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker

    // Note, we are "binding" the side length, 0.3, and the obstacles to the
    // state validity checker.  The _1 notation is from boost::bind and indicates
    // "the first argument" to the function pointer.

      if ( SE2orR2 == SE2 ) {
        ss.setStateValidityChecker(boost::bind(isValidStateSquare, _1, 0.3, obstacles));
    } else if ( SE2orR2 == R2) {
        ss.setStateValidityChecker(boost::bind(isValidStatePoint, _1, obstacles));
    }  

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    // You can index into the components of the state easily with ScopedState
    // The indexes correspond to the order that the StateSpace components were
    // added into the StateSpace
    ompl::base::ScopedState<> start(space);
    start[0] = startX;
    start[1] = startY;

    ompl::base::ScopedState<> goal(space);
    goal[0] = goalX;
    goal[1] = goalY;

     if ( SE2orR2 == SE2 ) {
         start[2] = 0.0;
         goal[2] = 0.0;
    }

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    ompl::base::PlannerPtr planner(new ompl::geometric::RandomTree(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);


        // print path to file
        std::ofstream fout("path.txt");
         
        if ( SE2orR2 == SE2 ) { 
            fout << "SE2" << std::endl;
        } else if ( SE2orR2 == R2 ){
            fout << "R2" << std::endl;
        }
        path.printAsMatrix(fout);
        fout.close();
    } else {
        std::cout << "No solution found" << std::endl;
    }
}

base::ValidStateSamplerPtr allocUniformStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::UniformValidStateSampler(si));
}

void benchmark0(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("cubicles");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    setup.setup();

    std::vector<double> cs(3);
    cs[0] = 35; cs[1] = 35; cs[2] = 35;
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 25;

}

void benchmark1(std::string& benchmark_name, app::SE3RigidBodyPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0,350.);
    bounds.setHigh(1,250.);
    bounds.setHigh(2,-150.);
    bounds.setLow(0,200.);
    bounds.setLow(1,75.);
    bounds.setLow(2,-450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 25;
}

void runBenchmarks(int twistycoolORcubicles) {

    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    if (twistycoolORcubicles == TWISTY)
        benchmark0(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (twistycoolORcubicles == CUBICLES)
        benchmark1(benchmark_name, setup, runtime_limit, memory_limit, run_count);

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    b.addPlanner(base::PlannerPtr(new geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::EST(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::PRM(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new geometric::RandomTree(setup.getSpaceInformation())));

    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    b.setExperimentName(benchmark_name + "_uniform_sampler");
    b.benchmark(request);
	if (twistycoolORcubicles == TWISTY) {
		b.saveResultsToFile("twisty.log");

	} else {
		b.saveResultsToFile("cubicles.log");
	}



}


int main(int, char **)
{

	//First environment
    std::vector<Rectangle> obstacles1;
    Rectangle obstacle;

	//1
    obstacle.x = -0.5;
    obstacle.y = -0.5;
    obstacle.width = 1.0;
    obstacle.height = 1.0;
    obstacles1.push_back(obstacle);

	//2
	obstacle.x = -5;
	obstacle.y = -5;
	obstacle.width = 1;
	obstacle.height = 5;
	obstacles1.push_back(obstacle);

	//3
	obstacle.x = -3.9;
	obstacle.y = -4;
	obstacle.width = 2;
	obstacle.height = 2;
	obstacles1.push_back(obstacle);

	//4
	obstacle.x = -1.5;
	obstacle.y = -5;
	obstacle.width = 4;
	obstacle.height = 1.5;
	obstacles1.push_back(obstacle);

	//5
	obstacle.x = -1;
	obstacle.y = -2;
	obstacle.width = 5;
	obstacle.height = 1;
	obstacles1.push_back(obstacle);

	//6
	obstacle.x = -3;
	obstacle.y = -2;
	obstacle.width = 2.2;
	obstacle.height = 3;
	obstacles1.push_back(obstacle);

	//7
	obstacle.x = 0;
	obstacle.y = 1.5;
	obstacle.width = 3;
	obstacle.height = 1.5;
	obstacles1.push_back(obstacle);

	//8
	obstacle.x = 3;
	obstacle.y = 0;
	obstacle.width = 1;
	obstacle.height = 5;
	obstacles1.push_back(obstacle);

	
	//Second environment
	std::vector<Rectangle> obstacles2;

	//1
    obstacle.x = -0.5;
    obstacle.y = .25;
    obstacle.width = 1.0;
    obstacle.height = .25;
    obstacles2.push_back(obstacle);

	//2
	obstacle.x = .49;
	obstacle.y = -1;
	obstacle.width = .25;
	obstacle.height = 1.27;
	obstacles2.push_back(obstacle);

	//3
	obstacle.x = -.5;
	obstacle.y = -.75;
	obstacle.width = .25;
	obstacle.height = 1;
	obstacles2.push_back(obstacle);

	//4
	obstacle.x = -.25;
	obstacle.y = -.75;
	obstacle.width = .4;
	obstacle.height = .25;
	obstacles2.push_back(obstacle);

	//5
	obstacle.x = -.75;
	obstacle.y = -.5;
	obstacle.width = .375;
	obstacle.height = .25;
	obstacles2.push_back(obstacle);

	//6
	obstacle.x = -.5;
	obstacle.y = .5;
	obstacle.width = .25;
	obstacle.height = .25;
	obstacles2.push_back(obstacle);

	//7
	obstacle.x = .6;
	obstacle.y = .4;
	obstacle.width = .2;
	obstacle.height = .6;
	obstacles2.push_back(obstacle);

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
				std::cout << " (1) Twistycool" << std::endl;
				std::cout << " (2) Cubicles" << std::endl;

				std::cin >> benchmarkChoice;
			} while (benchmarkChoice < 1 || benchmarkChoice > 2);
				runBenchmarks(benchmarkChoice);
				break;
		case PLANNER:
			int plannerChoice;
			do
			{
				std::cout << "Plan for: "<< std::endl;
				std::cout << " (1) A point in 2D" << std::endl;
				std::cout << " (2) A rigid box in 2D" << std::endl;

				std::cin >> plannerChoice;
			} while (plannerChoice < 1 || plannerChoice > 2);

			switch(plannerChoice)
			{
				case R2:
					std::cout << "Running in first environment (5x5 with 8 obstacles)\n";
				    planWithSimpleSetup(obstacles1, -5, 5, -3.5, -4.5, 4.5, 4.5, plannerChoice);
					std::cout << "Running in second environment (1x1 with 7 obstacles)\n";
					planWithSimpleSetup(obstacles2, -1, 1, 0, 0, .9, .9, plannerChoice);
				    break;
				case SE2:
					std::cout << "Running in first environment (5x5 with 8 obstacles)\n";
				    planWithSimpleSetup(obstacles1, -5, 5, -3.5, -4.5, 4.5, 4.5, plannerChoice);
				    break;
			}
	}
    return 0;
}

