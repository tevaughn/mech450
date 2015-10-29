#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"
#include "Main.h"

using namespace ompl;


void runPendulumBenchmark(int low, int high, int clow, int chigh,  double startT, double goalT) {
	
	std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

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

	// Define a simple setup class
	ompl::control::SimpleSetup ss(cspace);

    // Setup the StateValidityChecker
	ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));

	// Set propagationg routine
	ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<> (ss.getSpaceInformation(), &PendulumODE));

	ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &PendulumPostIntegration));

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

    std::vector<double> cs(2);
    cs[0] = 35;
    cs[1] = 35;

    space->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new myProjection(space)));
    ss.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 3;

    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(ss, benchmark_name);

    b.addPlanner(base::PlannerPtr(new control::RRT(ss.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(ss.getSpaceInformation())));
    //b.addPlanner(base::PlannerPtr(new base::RG-RRT(ss.getSpaceInformation())));

    ss.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    b.setExperimentName(benchmark_name + "_uniform_sampler");
    b.benchmark(request);

	b.saveResultsToFile("benchmark.log");

}

void runCarBenchmark(const std::vector<Rectangle>& obstacles,  int low, int high, int clow, int chigh, double startX, double startY, double goalX, double goalY) {
	
	std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

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

    std::vector<double> cs(2);
    cs[0] = 35;
    cs[1] = 35;
    ss.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 3;

    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(ss, benchmark_name);

    b.addPlanner(base::PlannerPtr(new control::RRT(ss.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(ss.getSpaceInformation())));
    //b.addPlanner(base::PlannerPtr(new base::RG-RRT(ss.getSpaceInformation())));

    ss.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    b.setExperimentName(benchmark_name + "_uniform_sampler");
    b.benchmark(request);

	b.saveResultsToFile("benchmark.log");

}
