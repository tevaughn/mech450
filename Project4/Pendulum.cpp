#include "Main.h"
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Goal.h>



void PendulumODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot) {
     	
	const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
	const double theta = q[0];


	// Zero out qdot
	qdot.resize (q.size(), 0);

	qdot[0] = q[1];
	qdot[1] = -9.81 * cos(theta) + u[0];

}

// This is a callback method invoked after numerical integration.
void PendulumPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
 {
 	// Normalize orientation between 0 and 2*pi
 	ompl::base::SO2StateSpace SO2;
	SO2.enforceBounds(result);
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state)
{
	return si->satisfiesBounds(state);
}

void planWithSimpleSetupPendulum(int low, int high, int clow, int chigh, double startT, double goalT, int plannerChoice)
{
    // Create the state (configuration) space for your system
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(1));

	ompl::base::RealVectorBounds bounds(1);
    bounds.setLow(clow); 
    bounds.setHigh(chigh);
	space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);	

	ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());    
	space = so2 + space;

 	// create a control space
	ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, 1));

	// set the bounds for the control space
	ompl::base::RealVectorBounds cbounds(1);
	cbounds.setLow(low);
	cbounds.setHigh(high);

	cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);

	std::vector<ompl::control::Control*> controls;	

	double interval = (chigh - clow)/10;
	for (double low = clow; low <= chigh; low += interval) {

        ompl::control::Control* control = cspace->allocControl();

        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = low;
        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = 0;

        controls.push_back(control);
	}

	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
	ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));
	std::cout << "validated\n";
	// Set propagation routine
	ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &PendulumODE));

	ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &PendulumPostIntegration));
    std::cout << "propped\n";
    // Specify the start and goal states

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
    start->as<ompl::base::SO2StateSpace::StateType>(0)->setIdentity();

    start[0] = startT;
    start[1] = 0.0;


    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
    goal->as<ompl::base::SO2StateSpace::StateType>(0)->setIdentity();

    goal[0] = goalT;
    goal[1] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
	ss.setup();

    // Specify a planning algorithm to use

    if (plannerChoice == RRT) {
        ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));
        ss.setPlanner(planner);
    } else if (plannerChoice == KPIECE) {
        ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss.getSpaceInformation()));
        space->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new myProjection(space)));
        ss.setPlanner(planner);
    }
    else if (plannerChoice == RGRRT) {
        ompl::base::PlannerPtr planner(new ompl::control::RGRRT(ss.getSpaceInformation(), controls));
        ss.setPlanner(planner);
    }

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
