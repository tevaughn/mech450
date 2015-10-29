#include <cmath>
#include <boost/bind.hpp>

// Including SimpleSetup.h will pull in MOST of what you need to plan
#include <ompl/base/samplers/UniformValidStateSampler.h>

// Except for the state space definitions and any planners
#include <omplapp/apps/SE3RigidBodyPlanning.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

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

const int RRT = 1;
const int KPIECE = 2;
const int RGRRT = 3;



void planWithSimpleSetupCar(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY, int plannerChoice);

void CarODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot);

void CarPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result);

ompl::base::ValidStateSamplerPtr allocUniformStateSampler(const ompl::base::SpaceInformation *si);

<<<<<<< HEAD
// custom pendulum projection for KPIECE1
class myProjection : public ompl::base::ProjectionEvaluator
{
public:
    myProjection(const ompl::base::StateSpacePtr &space) : ompl::base::ProjectionEvaluator(space)
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
=======

void planWithSimpleSetupPendulum(int low, int high, int clow, int chigh, double startT, double goalT, int plannerChoice);

void PendulumODE (const ompl::control::ODESolver::StateType& q, const ompl::control::Control* control, ompl::control::ODESolver::StateType& qdot);

void PendulumPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result);


void runCarBenchmark(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY);

void runPendulumBenchmark(int low, int high, int clow, int chigh,  double startT, double goalT);



bool stateAlwaysValid(const ompl::base::State* /*state*/);

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle>& obstacles);
bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles);



>>>>>>> a960255adb7899f9e066820bfc09f2543e56966f
