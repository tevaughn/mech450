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

void CarODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot) {
     	
	const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
	const double theta = q[2];
	double carLength = 0.5;

	// Zero out qdot
	qdot.resize (q.size (), 0);

	qdot[0] = u[0] * cos(theta);
	qdot[1] = u[0] * sin(theta);
	qdot[2] = u[0] * tan(u[1]) / carLength;

}

// This is a callback method invoked after numerical integration.
void CarPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
 {
 	// Normalize orientation between 0 and 2*pi
 	ompl::base::SO2StateSpace SO2;
	SO2.enforceBounds (result->as<ompl::base::SE2StateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

void planWithSimpleSetupCar(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY)
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
	ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, 2));
 
	// set the bounds for the control space
	ompl::base::RealVectorBounds cbounds(2);
	cbounds.setLow(clow);
	cbounds.setHigh(chigh);

	cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);

	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
    ss.setStateValidityChecker(boost::bind(isValidStatePoint, _1, obstacles)); 

	// Set propagationg routine
	ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &CarODE));
	ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &CarPostIntegration));

    // Specify the start and goal states
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
	start->setX(startX);
	start->setY(startY);
	start->setYaw(0.0);    

    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
	goal->setX(goalX);
	goal->setY(goalY);
	goal->setYaw(0.0);  

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
	ss.setup();

    // Specify a planning algorithm to use
    ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {

        // print the path to screen
        ompl::geometric::PathGeometric path = ss.getSolutionPath().asGeometric();
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
