#include "Main.h"




void PendulumODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot) {
     	
	const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
	const double theta = q[0];


	// Zero out qdot
	qdot.resize (q.size (), 0);

	qdot[0] = q[1];
	qdot[1] = -9.81 * cos(theta) + u[0];

}

// This is a callback method invoked after numerical integration.
void PendulumPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
 {
 	// Normalize orientation between 0 and 2*pi
 	ompl::base::SO2StateSpace SO2;
	SO2.enforceBounds (result->as<ompl::base::SE2StateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

void planWithSimpleSetupPendulum(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY)
{
    // Create the state (configuration) space for your system
    ompl::base::StateSpacePtr space(new ompl::base::SO2StateSpace());

    // We need to set bounds on R^2
    //ompl::base::RealVectorBounds bounds(2);
    //bounds.setLow(low);
    //bounds.setHigh(high);

    // Cast the r2 pointer to the derived type, then set the bounds
    //space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    
 	// create a control space
	ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, 1));
 
	// set the bounds for the control space
	ompl::base::RealVectorBounds cbounds(1);
	cbounds.setLow(clow);
	cbounds.setHigh(chigh);

	cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);

	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
    ss.setStateValidityChecker(boost::bind(isValidStatePoint, _1, obstacles)); 

	// Set propagationg routine
	ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &PendulumODE));
	ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &PendulumPostIntegration));

    // Specify the start and goal states
    ompl::base::ScopedState<ompl::base::SO2StateSpace> start(space);
	start->value = -3.14/2; 

    ompl::base::ScopedState<ompl::base::SO2StateSpace> goal(space);
	goal->value = 3.14/2;

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
