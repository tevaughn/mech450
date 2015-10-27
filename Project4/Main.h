#include <cmath>
#include <boost/bind.hpp>

// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/base/samplers/UniformValidStateSampler.h>

// Except for the state space definitions and any planners
#include <omplapp/apps/SE3RigidBodyPlanning.h>

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

bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles);

bool stateAlwaysValid(const ompl::base::State* /*state*/);

void planWithSimpleSetupCar(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY);

void planWithSimpleSetupPendulum(int clow, int chigh, double startT, double goalT);
