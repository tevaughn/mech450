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


const int BENCHMARK  = 1;
const int PLANNER = 2;

const int PENDULUM  = 1;
const int CAR = 2;


void planWithSimpleSetupCar(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY, int plannerChoice);

void CarODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot);

void CarPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result);

ompl::base::ValidStateSamplerPtr allocUniformStateSampler(const ompl::base::SpaceInformation *si);


void planWithSimpleSetupPendulum(int low, int high, int clow, int chigh, double startT, double goalT, int plannerChoice);

void PendulumODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot);

void PendulumPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result);


void runCarBenchmark(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY);

void runPendulumBenchmark(int low, int high, int clow, int chigh,  double startT, double goalT);



bool stateAlwaysValid(const ompl::base::State* /*state*/);

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles);
bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles);



