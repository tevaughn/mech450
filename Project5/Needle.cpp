#include "Main.h"
#include "SMR.h"

#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>

#define RIGHT 0
#define LEFT 1

void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, int uncertaintyRange)
{
    const ompl::base::CompoundState *state = start->as<ompl::base::CompoundState>();
    const ompl::base::SE2StateSpace::StateType* se2 = state->as<ompl::base::SE2StateSpace::StateType>(0);
    //const ompl::base::DiscreteStateSpace::StateType* d = state->as<ompl::base::DiscreteStateSpace::StateType>(1);
    
    const ompl::control::RealVectorControlSpace::ControlType* rctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>();

	// +- 10% uncertainty
	const double uncertainty = (rand()%(uncertaintyRange*2) + (100 - uncertaintyRange))/100.0; 
	const double r = rctrl->values[0] * uncertainty;
    const double b = rctrl->values[1];
    
    double theta = se2->getYaw();
    double w = duration/r;


    if (b == LEFT) {
        w = -w;
    }

    double yawout = (se2->getYaw() + w);
    double xout = (se2->getX() + r*cos(yawout));
    double yout = (se2->getY() + r*sin(yawout));
    double bout = b;

    ompl::base::CompoundStateSpace::StateType* out = result->as<ompl::base::CompoundStateSpace::StateType>();
    ompl::base::SE2StateSpace::StateType* se2out = out->as<ompl::base::SE2StateSpace::StateType>(0);
    se2out->setXY(xout, yout);
    se2out->setYaw(yawout);
	
	

    ompl::base::SO2StateSpace::StateType* so2out = se2out->as<ompl::base::SO2StateSpace::StateType>(1);
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds (so2out);

    ompl::base::DiscreteStateSpace::StateType* dout = out->as<ompl::base::DiscreteStateSpace::StateType>(1);
    dout->value = bout;
}
 

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::CompoundState *compoundState = state->as<ompl::base::CompoundState>();
    const ompl::base::SE2StateSpace::StateType *se2state = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
	const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

	if ((abs(se2state->getX()) > 10 || abs(se2state->getY()) > 10) && si->satisfiesBounds(state) ) {
		std::cout << se2state->getX() << " " << se2state->getY() << " "  << si->satisfiesBounds(state) << "\n";
	}

	return si->satisfiesBounds(state) && isValidStatePoint(pos, obstacles);
}

void planWithSimpleSetupNeedle(const std::vector<Rectangle>& obstacles, int uncertainty,  int low, int high, int rlow, int rhigh, double startX, double startY, double goalX, double goalY)
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

 	// create a control space
    ompl::control::ControlSpacePtr cspace(new ompl::control::CompoundControlSpace(space));
	ompl::control::ControlSpacePtr rspace(new ompl::control::RealVectorControlSpace(space, 1));
    ompl::control::ControlSpacePtr bspace(new ompl::control::DiscreteControlSpace(space, 0, 1));

	ompl::base::RealVectorBounds rbounds(1);
	rbounds.setLow(rlow);
	rbounds.setHigh(rhigh);
	rspace->as<ompl::control::RealVectorControlSpace>()->setBounds(rbounds);


    cspace->as<ompl::control::CompoundControlSpace>()->addSubspace(rspace);
    cspace->as<ompl::control::CompoundControlSpace>()->addSubspace(bspace);

	std::vector<ompl::control::Control*> controls;	

	double interval = (rhigh - rlow)/5.0;
	for (double low = rlow; low <= rhigh; low += interval) {
        ompl::control::Control* rcontrol = cspace->allocControl();

        rcontrol->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = low;
        rcontrol->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = 0;

        controls.push_back(rcontrol);

		ompl::control::Control* lcontrol = cspace->allocControl();

        lcontrol->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = low;
        lcontrol->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = 1;

        controls.push_back(lcontrol);
	}

	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
	ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1, obstacles));

	// Set propagation routine
    ss.setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4, uncertainty));


    // Specify the start and goal states
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);

    start->as<ompl::base::SE2StateSpace::StateType>(0)->setX(startX);
    start->as<ompl::base::SE2StateSpace::StateType>(0)->setY(startY);
    start->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    start->as<ompl::base::DiscreteStateSpace::StateType>(1)->value = 0;



    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);

    goal->as<ompl::base::SE2StateSpace::StateType>(0)->setX(goalX);
    goal->as<ompl::base::SE2StateSpace::StateType>(0)->setY(goalY);
    goal->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(0.0);
    goal->as<ompl::base::DiscreteStateSpace::StateType>(1)->value = 0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
	ss.setup();

    // Specify a planning algorithm to use
	ompl::base::PlannerPtr planner(new ompl::control::SMR(ss.getSpaceInformation(), controls, 1000, 5));

	ss.setPlanner(planner);


    // Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(2.0);

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


