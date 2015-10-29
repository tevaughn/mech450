#include "Main.h"

void CarODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot) {

	const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;	
    const double theta = q[2];
    const double velocity = q[3];

	// Zero out qdot
	qdot.resize (q.size (), 0);
    
	qdot[0] = velocity * cos(theta);
	qdot[1] = velocity * sin(theta);
	qdot[2] = u[0];
    qdot[3] = u[1];

}

// This is a callback method invoked after numerical integration.
void CarPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
 {
 	// Normalize orientation between 0 and 2*pi
 	ompl::base::SO2StateSpace SO2;
	SO2.enforceBounds (result->as<ompl::base::SE2StateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles)
{

    const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

	return si->satisfiesBounds(state) && isValidStatePoint(pos, obstacles);
}

void planWithSimpleSetupCar(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY, int plannerChoice)
{
    // Create the state (configuration) space for your system
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
    ompl::base::StateSpacePtr vspace(new ompl::base::RealVectorStateSpace(1));
 

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(low);
    bounds.setHigh(high);

    // Cast the r2 pointer to the derived type, then set the bounds
    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // We need to set bounds on velocity
    ompl::base::RealVectorBounds vbounds(1);
    vbounds.setLow(low);
    vbounds.setHigh(high);

    vspace->as<ompl::base::RealVectorStateSpace>()->setBounds(vbounds);

    space = space + vspace;


 	// create a control space
	ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, 2));
 
	// set the bounds for the control space
	ompl::base::RealVectorBounds cbounds(2);
	cbounds.setLow(clow);
	cbounds.setHigh(chigh);

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
	ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1, obstacles));

	// Set propagation routine
	ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &CarODE));
	ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &CarPostIntegration));

    // Specify the start and goal states
    ompl::base::ScopedState<> start(space);
    start[0] = startX;
    start[1] = startY;
    start[2] = 0.0;
    start[3] = 0.0;

    ompl::base::ScopedState<> goal(space);
    goal[0] = goalX;
    goal[1] = goalY;
    goal[2] = 0.0;
    goal[3] = 0.0;

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


