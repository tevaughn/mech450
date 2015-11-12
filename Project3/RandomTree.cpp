
/* Author: Team Sean */

#include "RandomTree.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>


ompl::geometric::RandomTree::RandomTree(const base::SpaceInformationPtr &si) : base::Planner(si, "RandomTree") {
    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range", this, &RandomTree::setRange, &RandomTree::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RandomTree::setGoalBias, &RandomTree::getGoalBias, "0.:.05:1.");
}

ompl::geometric::RandomTree::~RandomTree() {
    freeMemory();
}


void ompl::geometric::RandomTree::clear() {
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nodes_.size() > 0) {
        nodes_.clear();
	}
    lastGoalMotion_ = NULL;
}


void ompl::geometric::RandomTree::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
}

void ompl::geometric::RandomTree::freeMemory()
{
    if (nodes_.size() > 0)
    {

        for (unsigned int i = 0 ; i < nodes_.size() ; ++i)
        {
            if (nodes_[i]->state)
                si_->freeState(nodes_[i]->state);
            delete nodes_[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RandomTree::solve(const base::PlannerTerminationCondition &terminationCondition)
{
    checkValidity();
	// get our goal
    base::Goal                 *goal   = pdef_->getGoal().get();
	// Can we sample the goal?    
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Node *node = new Node(si_);
        node->parent = NULL;
        si_->copyState(node->state, st);
        nodes_.push_back(node);
    }

    if (nodes_.size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

	// If we haven't created our sampler, do it
    if (!sampler_) {
        sampler_ = si_->allocStateSampler();
	}

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nodes_.size());

    Node *solution  = NULL;
    Node *approxsol = nodes_[0];
    double approxDistToGoal = std::numeric_limits<double>::infinity();
    //Node *nodeB = new Node(si_);
    Node *nodeA = nodes_[0];
    base::State *newState = si_->allocState();
    
    while (terminationCondition == false) {
	    /* sample random node */

	    int randomIndex = rand() % nodes_.size();
	    Node* nodeA = nodes_[randomIndex]; 

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
            goal_s->sampleGoal(newState);
		} else {
            sampler_->sampleUniform(newState);
		}

        // if no collisions
        if (si_->checkMotion(nodeA->state, newState)) {
            // Set nodeB parent to nodeA and add to nodes vector
            Node *newNode = new Node(si_);
 
            newNode->parent = nodeA;
            si_->copyState(newNode->state, newState);

            nodes_.push_back(newNode);

            // If we made it to the goal, we're done
            double distToGoal = 0.0;
            bool satisfied = goal->isSatisfied(newNode->state, &distToGoal);
            if (satisfied) {
                approxDistToGoal = distToGoal;
                solution = newNode;
                break;
            }
            if (distToGoal < approxDistToGoal)
            {
                approxDistToGoal = distToGoal;
                approxsol = newNode;
            }
        }
    }

    bool solved = false;
    bool approximate = false;

    // If we didn't find a solution, return our closest attempt
    if (solution == NULL )
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Node*> nodePath;
        while (solution != NULL)
        {
            nodePath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = nodePath.size() - 1 ; i >= 0 ; --i) {
            path->append(nodePath[i]->state);
        }
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxDistToGoal, getName());
        solved = true;
    }
    
    if (newState)
        si_->freeState(newState);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nodes_.size());
    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RandomTree::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < nodes_.size() ; ++i)
    {
        if (nodes_[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(nodes_[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(nodes_[i]->parent->state),
                         base::PlannerDataVertex(nodes_[i]->state));
    }
}

