#include "Main.h"

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

// custom projection for KPIECE1
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
	CarProjection(const ompl::base::StateSpacePtr &space) : ompl::base::ProjectionEvaluator(space)
	{
	}
	virtual unsigned int getDimension(void) const
	{
		return 2;
	}
	virtual void defaultCellSizes(void)
	{
		cellSizes_.resize(2);
		cellSizes_[0] = 0.1;
		cellSizes_[1] = 0.25;
	}
	virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
	{
		const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
		projection(0) = (values[0] + values[1]) / 2.0;
		projection(1) = (values[2] + values[3]) / 2.0;
	}
};

// This is a callback method invoked after numerical integration.
void CarPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
 {
 	// Normalize orientation between 0 and 2*pi
 	ompl::base::SO2StateSpace SO2;
	SO2.enforceBounds (result->as<ompl::base::SE2StateSpace::StateType>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles)
{
     //    ob::ScopedState<ob::SE2StateSpace>
     const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();

     const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

     //const ompl::base::SO2StateSpace::StateType *rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1);
 
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
	return si->satisfiesBounds(state) && isValidStatePoint(pos, obstacles);
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
    //ss.setStateValidityChecker(boost::bind(isValidStatePoint, _1, obstacles)); 
	ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1, obstacles));

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

    // set rrt as planning algorithm
    //ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));

	// register the kpiece projection
	//space->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new CarProjection(space)));

	//set kpiece as planning algorithm
	ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss.getSpaceInformation()));

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
