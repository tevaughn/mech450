
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
	return true;
}


int main(int, char **)
{

	// Empty environment
	std::vector<Rectangle> none;

	//Street environment
    std::vector<Rectangle> obstacles1;
    
    Rectangle obstacle;

	//1
    obstacle.x = -10.0;
    obstacle.y = -10.0;
    obstacle.width = 20.0;
    obstacle.height = 4.0;
    obstacles1.push_back(obstacle);

	//2
	obstacle.x = -10;
	obstacle.y = -4;
	obstacle.width = 10;
	obstacle.height = 8;
	obstacles1.push_back(obstacle);

	//3
	obstacle.x = 2;
	obstacle.y = -4;
	obstacle.width = 8;
	obstacle.height = 8;
	obstacles1.push_back(obstacle);

	//4
	obstacle.x = -10;
	obstacle.y = 6;
	obstacle.width = 20;
	obstacle.height = 2;
	obstacles1.push_back(obstacle);

	//Block environment
    std::vector<Rectangle> obstacles2;

	//1
    obstacle.x = -7.0;
    obstacle.y = -2.0;
    obstacle.width = 14.0;
    obstacle.height = 4.0;
    obstacles2.push_back(obstacle);

    //Small obstacles environment
    std::vector<Rectangle> obstacles3;

	//1
	obstacle.x = -6;
	obstacle.y = -4;
	obstacle.width = 1;
	obstacle.height = 1;
	obstacles3.push_back(obstacle);

	//2
	obstacle.x = -1.5;
	obstacle.y = -5;
	obstacle.width = 3;
	obstacle.height = 3;
	obstacles3.push_back(obstacle);

	//3
	obstacle.x = -5;
	obstacle.y = 2.5;
	obstacle.width = 4.5;
	obstacle.height = .5;
	obstacles3.push_back(obstacle);

	
	//4
	obstacle.x = -3.25;
	obstacle.y = -1;
	obstacle.width = 6;
	obstacle.height = .5;
	obstacles3.push_back(obstacle);

	//5
	obstacle.x = 3;
	obstacle.y = 6;
	obstacle.width = 2;
	obstacle.height = 3;
	obstacles3.push_back(obstacle);

	//6
	obstacle.x = -3;
	obstacle.y = -4;
	obstacle.width = 1;
	obstacle.height = 2;
	obstacles3.push_back(obstacle);

	
	//7
	obstacle.x = 2;
	obstacle.y = -6;
	obstacle.width = 1;
	obstacle.height = 2;
	obstacles3.push_back(obstacle);

	//8
	obstacle.x = -1.25;
	obstacle.y = -8.5;
	obstacle.width = 1;
	obstacle.height = 2;
	obstacles3.push_back(obstacle);

	//9
	obstacle.x = -4.75;
	obstacle.y = 4;
	obstacle.width = 1.5;
	obstacle.height = 5;
	obstacles3.push_back(obstacle);


	//10
	obstacle.x = -4.25;
	obstacle.y = -2;
	obstacle.width = .5;
	obstacle.height = 2;
	obstacles3.push_back(obstacle);

	//11
	obstacle.x = -1;
	obstacle.y = 4;
	obstacle.width = 3;
	obstacle.height = 4;
	obstacles3.push_back(obstacle);
	

    int obstaclesNo;
	int uncertainty;

    do 
	{
        std::cout << "Plan with: "<< std::endl;
        std::cout << " (1) no Obstacles" << std::endl;
		std::cout << " (2) H-obstacles" << std::endl;
        std::cout << " (3) Middle block" << std::endl;
        std::cout << " (4) Many small obstacles" << std::endl;


		std::cin >> obstaclesNo;

	} while (obstaclesNo < 1 || obstaclesNo > 4);
	
	do
	{
        std::cout << "Use uncertainty: "<< std::endl;
        std::cout << " (1) 1%" << std::endl;
		std::cout << " (2) 5%" << std::endl;
        std::cout << " (3) 10%" << std::endl;
        std::cout << " (4) 50%" << std::endl;


		std::cin >> uncertainty;

	} while (uncertainty < 1 || uncertainty > 4);

	switch(uncertainty)
	{
    	case 1: uncertainty = 1;
		break;
    	case 2: uncertainty = 5;
		break;
    	case 3: uncertainty = 10;
		break;
    	case 4: uncertainty = 50;
		break;

	}
    switch(obstaclesNo)
    {
        case 1:
            std::cout << "Running in empty environment\n";
	        planWithSimpleSetupNeedle(none, uncertainty, -10, 10, 1, 5, -5, -5, 5, 5);
        break;
        case 2:
            std::cout << "Running in H environment\n";
	        planWithSimpleSetupNeedle(obstacles1, uncertainty,-10, 10, 1, 5, -5, -5, 5, 5);
            break;
        case 3:
            std::cout << "Running in BLOCK environment\n";
	        planWithSimpleSetupNeedle(obstacles2, uncertainty,-10, 10, 1, 5, -5, -5, 5, 5);
            break;
        case 4:
            std::cout << "Running in small obstacles environment\n";
	        planWithSimpleSetupNeedle(obstacles3, uncertainty,-10, 10, 1, 5, -5, -5, 5, 5);
        break;
        }

    return 0;
}

