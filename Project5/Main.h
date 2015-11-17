#include <cmath>
#include <boost/bind.hpp>



// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/base/samplers/UniformValidStateSampler.h>

#include "SMR.h"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <valarray>
#include <limits>



// The collision checker produced in project 2
#include "CollisionChecking.h"


void planWithSimpleSetupNeedle(const std::vector<Rectangle>& obstacles,  int low, int high, int rlow, int rhigh, double startX, double startY, double goalX, double goalY);

void NeedleODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot);

void NeedlePostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result);

ompl::base::ValidStateSamplerPtr allocUniformStateSampler(const ompl::base::SpaceInformation *si);


//bool stateAlwaysValid(const ompl::base::State* /*state*/);


bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles);
//bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles);


class NeedleControl : public ompl::control::Control {

	public:
		NeedleControl() {}
};

