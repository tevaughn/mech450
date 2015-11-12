#include "Main.h"

using namespace ompl;

base::ValidStateSamplerPtr allocUniformStateSampler(const base::SpaceInformation *si)
{
    return base::ValidStateSamplerPtr(new base::UniformValidStateSampler(si));
}



// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles)
{
    const ompl::base::RealVectorStateSpace::StateType* r2state;
    r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    // Extract x, y
    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}

// This is our state validity checker.  It says every state is valid.
bool stateAlwaysValid(const ompl::base::State* /*state*/)
{
	//std::cout << "HERE!\n" ;   
	return true;
}


int main(int, char **)
{
	// Empty environment
	std::vector<Rectangle> none;

	//Street environment
    std::vector<Rectangle> obstacles;
    Rectangle obstacle;

	//1
    obstacle.x = -10.0;
    obstacle.y = -10.0;
    obstacle.width = 20.0;
    obstacle.height = 4.0;
    obstacles.push_back(obstacle);

	//2
	obstacle.x = -10;
	obstacle.y = -4;
	obstacle.width = 10;
	obstacle.height = 8;
	obstacles.push_back(obstacle);

	//3
	obstacle.x = 2;
	obstacle.y = -4;
	obstacle.width = 8;
	obstacle.height = 8;
	obstacles.push_back(obstacle);

	//4
	obstacle.x = -10;
	obstacle.y = 6;
	obstacle.width = 20;
	obstacle.height = 2;
	obstacles.push_back(obstacle);


	int benchmarkOrPlan;
	do 
	{
        std::cout << "Benchmark (1) or Plan (2) "<< std::endl;
		std::cin >> benchmarkOrPlan;

	} while (benchmarkOrPlan < 1 || benchmarkOrPlan > 2);

	switch (benchmarkOrPlan) 
	{
		case BENCHMARK:
			int benchmarkChoice;
			do
			{
				std::cout << "Benchmark for: "<< std::endl;
				std::cout << " (1) Pendulum" << std::endl;
				std::cout << " (2) Car" << std::endl;
				
				std::cin >> benchmarkChoice;
			} while (benchmarkChoice < 1 || benchmarkChoice > 2);

			switch(benchmarkChoice)
			{
				case PENDULUM:
					runPendulumBenchmark(-10, 10, -10, 10, -3.14/2, 3.14/2);
					break;
				case CAR:
					runCarBenchmark(obstacles, -15, 15, -10, 10, -5, -5, 5, 5);
					break;
			}
	    
			break;

		case PLANNER:
			int envChoice;
			do
			{
				std::cout << "Plan for: "<< std::endl;
				std::cout << " (1) Pendulum" << std::endl;
				std::cout << " (2) Car" << std::endl;

				std::cin >> envChoice;
			} while (envChoice < 1 || envChoice > 2);

            int plannerChoice;
            do
            {
                std::cout << "Plan using: "<< std::endl;
                std::cout << " (1) RRT" << std::endl;
                std::cout << " (2) KPIECE" << std::endl;
                std::cout << " (3) RGRRT" << std::endl;

                std::cin >> plannerChoice;
            } while (plannerChoice < 1 || plannerChoice > 3);

			switch(envChoice)
			{
				case PENDULUM:
					std::cout << "Running in empty environment \n";
                    //changed bounds to 10 because 9.81 is not an int
				    planWithSimpleSetupPendulum(-2, 2, -10, 10, -3.14/2, 3.14/2, plannerChoice);
				    break;
				case CAR:
					std::cout << "Running in street like environment\n";
				    planWithSimpleSetupCar(obstacles, -10, 10, -10, 10, -5, -5, 5, 5, plannerChoice);
				    break;
			}
	}
    return 0;
}

