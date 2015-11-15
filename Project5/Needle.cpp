#include "Main.h"
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>

#define RIGHT 1
#define LEFT 1

void NeedleODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot) {
	const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;	

    const double r = u[0];
    const double b = u[1];
    
    const double theta = q[2];
    double w = 1/r;

    if (b == LEFT) {
        w = -w;
    }

	// Zero out qdot
	qdot.resize (q.size(), 0);
    
	qdot[0] = cos(theta);
	qdot[1] = sin(theta);
	qdot[2] = w;
    qdot[3] = b - q[3];

}

// This is a callback method invoked after numerical integration.
void NeedlePostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
 {
 	// Normalize orientation between 0 and 2*pi
 	ompl::base::SO2StateSpace SO2;
	SO2.enforceBounds (result->as<ompl::base::SE2StateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles)
{

    const ompl::base::CompoundState *compoundState = state->as<ompl::base::CompoundState>();
    const ompl::base::SE2StateSpace::StateType *se2state = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
	const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

	return si->satisfiesBounds(state) && isValidStatePoint(pos, obstacles);
}

void planWithSimpleSetupNeedle(const std::vector<Rectangle>& obstacles,  int low, int high, int rlow, int rhigh, double startX, double startY, double goalX, double goalY)
{
    // Create the state (configuration) space for your system
    ompl::base::StateSpacePtr space(new ompl::base::CompoundStateSpace());
    ompl::base::StateSpacePtr pspace(new ompl::base::SE2StateSpace());
    ompl::base::StateSpacePtr dspace(new ompl::base::DiscreteStateSpace(0,1));


    // Set bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(low);
    bounds.setHigh(high);

    // Cast the r2 pointer to the derived type, then set the bounds
    pspace->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    space = pspace + dspace;

    std::cout << "set space \n";
    std::cout << space->as<ompl::base::CompoundStateSpace>()->getDimension() << "\n";

 	// create a control space
    ompl::control::ControlSpacePtr cspace(new ompl::control::CompoundControlSpace(space));
	ompl::control::ControlSpacePtr rspace(new ompl::control::RealVectorControlSpace(space, 1));
    ompl::control::ControlSpacePtr bspace(new ompl::control::DiscreteControlSpace(space, 0, 1));

	// set the bounds for the control space
	ompl::base::RealVectorBounds rbounds(1);
	rbounds.setLow(rlow);
	rbounds.setHigh(rhigh);
	rspace->as<ompl::control::RealVectorControlSpace>()->setBounds(rbounds);


    cspace->as<ompl::control::CompoundControlSpace>()->addSubspace(rspace);
    cspace->as<ompl::control::CompoundControlSpace>()->addSubspace(bspace);

	std::vector<ompl::control::Control*> controls;	
   std::cout << "set controls \n";
	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
	ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1, obstacles));

	// Set propagation routine
	ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &NeedleODE));
	ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &NeedlePostIntegration));
   std::cout << "set ODE stuff \n";

    // Specify the start and goal states
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);

    start->as<ompl::base::SE2StateSpace::StateType>(0)->setX(startX);
    start->as<ompl::base::SE2StateSpace::StateType>(0)->setY(startY);
    start->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    //start->as<ompl::base::DiscreteStateSpace::StateType>(1)->value = 0;



    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);

    goal->as<ompl::base::SE2StateSpace::StateType>(0)->setX(goalX);
    goal->as<ompl::base::SE2StateSpace::StateType>(0)->setY(goalY);
    //goal->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    //goal->as<ompl::base::DiscreteStateSpace::StateType>(1)->value = 0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
	ss.setup();

    // Specify a planning algorithm to use
	//ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
	//ss.setPlanner(planner);


    // Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(20.0);

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


