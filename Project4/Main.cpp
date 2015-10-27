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

using namespace ompl;

const int BENCHMARK  = 1;
const int PLANNER = 2;

const int PENDELUM  = 1;
const int CAR = 2;

// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    // Extract x, y
    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}


// This is our state validity checker.  It says every state is valid.
bool stateAlwaysValid(const ompl::base::State* /*state*/)
{
    return true;
}


void pendulumODE(const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot) {

	const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
	const double thera = q[2];
	
	qdot.resize(q.size(),0);
	
	qdot[0] = q[1]
	qdpt[1] = cos(q[0])*-9.81+tau;


}

void CarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot) {
     	
	const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
	const double theta = q[2];
	double carLength = 0.5;

	// Zero out qdot
	qdot.resize (q.size (), 0);

	qdot[0] = u[0] * cos(theta);
	qdot[1] = u[0] * sin(theta);
	qdot[2] = u[0] * tan(u[1]) / carLength;

}

void planWithSimpleSetup(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY, int SE2orR2)
{
    // Create the state (configuration) space for your system
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(low);
    bounds.setHigh(high);

    // Cast the r2 pointer to the derived type, then set the bounds
    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    
 	// create a control space
	ompl::control::ControlSpacePtr cspace(new RealVectorControlSpace(space, 2));
 
	// set the bounds for the control space
	ompl::base::RealVectorBounds cbounds(2);
	cbounds.setLow(clow);
	cbounds.setHigh(chigh);

	cspace->as<DemoControlSpace>()->setBounds(cbounds);

	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
    ss.setStateValidityChecker(boost::bind(isValidStatePoint, _1, obstacles)); 

	// Set propagationg routine
	ompl::control::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &KinematicCarODE));
	ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration))

    // Specify the start and goal states
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
	start->setX(startX);
	start->setY(startY);
	start->setYaw(0.0)    

    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
	goal->setX(goalX);
	goal->setY(goalY);
	goal->setYaw(0.0)   

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
	ss.setup()

    // Specify a planning algorithm to use
    ompl::control::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        ompl::geometric::PathGeometric& path = ss.getSolutionPath().asGeometric();
        path.interpolate(50);
        path.printAsMatrix(std::cout);


        // print path to file
        std::ofstream fout("path.txt");

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
				runBenchmarks(benchmarkChoice);
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
				    planWithSimpleSetup(none, -5, 5, -3.5, -4.5, 4.5, 4.5, plannerChoice);
				    break;
				case CAR:
					std::cout << "Running in street like environment\n";
				    planWithSimpleSetup(obstacles, -5, 5, -3.5, -4.5, 4.5, 4.5, plannerChoice);
				    break;
			}
	}
    return 0;
}

