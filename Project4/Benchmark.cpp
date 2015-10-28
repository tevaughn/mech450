#include <omplapp/apps/BlimpPlanning.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>


// The collision checker produced in project 2
#include "CollisionChecking.h"
#include "Main.h"

using namespace ompl;

const int TWISTY  = 1;
const int CUBICLES = 2;

void benchmark0(std::string& benchmark_name, app::BlimpPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("blimp");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/blimp.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    setup.setup();

    std::vector<double> cs(3);
    cs[0] = 35; cs[1] = 35; cs[2] = 35;
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 25;

}

void benchmark1(std::string& benchmark_name, app::BlimpPlanning& setup,
                double& runtime_limit, double& memory_limit, int& run_count)
{
    benchmark_name = std::string("Twistycool");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0,350.);
    bounds.setHigh(1,250.);
    bounds.setHigh(2,-150.);
    bounds.setLow(0,200.);
    bounds.setLow(1,75.);
    bounds.setLow(2,-450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 10.0;
    memory_limit  = 10000.0; // set high because memory usage is not always estimated correctly
    run_count     = 25;
}

void runBenchmarks(int twistycoolORcubicles) {
	std::cout << "lets try to do things";
    app::BlimpPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;
	std::cout << "lets try to call bnech";
    if (twistycoolORcubicles == TWISTY)
        benchmark0(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else if (twistycoolORcubicles == CUBICLES)
        benchmark1(benchmark_name, setup, runtime_limit, memory_limit, run_count);
	std::cout << "called bench";
    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
    //b.addPlanner(base::PlannerPtr(new base::RG-RRT(setup.getSpaceInformation())));

    setup.getSpaceInformation()->setValidStateSamplerAllocator(&allocUniformStateSampler);
    b.setExperimentName(benchmark_name + "_uniform_sampler");
    b.benchmark(request);
	if (twistycoolORcubicles == TWISTY) {
		b.saveResultsToFile("twisty.log");

	} else {
		b.saveResultsToFile("cubicles.log");
	}
}
