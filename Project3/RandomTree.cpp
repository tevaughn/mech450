
/* Author: Team Sean */

#include "RandomTree.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::RandomTree::RandomTree(const base::SpaceInformationPtr &spaceInfo) : base::Planner(spaceInfo, "RandomTree") {
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
    if (nodes_) {
        nodes_->clear();
	}
    lastGoalMotion_ = NULL;
}


void ompl::geometric::RandomTree::setup() {
    Planner::setup();
    tools::SelfConfig sc(spaceInfo_, getName());
    sc.configurePlannerRange(maxDistance_);

	if (!nodes_) {
		nodes_.reset();
	}

	nodes_->push_back(Node(spaceInfo_->getStateSpace(), NULL));
}

void ompl::geometric::RandomTree::freeMemory()
{
    if (nodes_)
    {

        for (unsigned int i = 0 ; i < nodes_->size() ; ++i)
        {
            if ((*nodes_)[i]->state)
                spaceInfo_->freeState((*nodes_)[i]->state);
            delete (*nodes_)[i];
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

//    while (const base::State *st = pis_.nextStart())
//    {
//        Motion *motion = new Motion(si_);
//        si_->copyState(motion->state, st);
//        nn_->add(motion);
//    }

    if (nodes_->size() == 0) {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

	// If we haven't created our sampler, do it
    if (!sampler_) {
        sampler_ = spaceInfo_->allocStateSampler();
	}

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Node *solution  = NULL;
    Node *approxsol = NULL;
    Node *nodeB = new Node(spaceInfo_, NULL);
	Node *nodeA;

    while (terminationCondition == false)
    {
		/* sample random node */
		int randomIndex = rand() % nodes_.size();
		boost::weak_ptr < Node > nodeA = nodes_[randomIndex]; 

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
            goal_s->sampleGoal(nodeB->state);
		} else {
            sampler_->sampleUniform(nodeB->state);
		}


		// if no collisions
        if (true) {
			// Set nodeB parent to nodeA and add to nodes vector
			nodeB->parent = nodeA;
            nodes_->push_back(nodeB);

            bool satisfied = goal->isSatisfied(nodeB->state, 0.0);
            if (satisfied) {
                solution = nodeB;
                break;
            }

            approxsol = nodeB;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
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
        PathGeometric *path = new PathGeometric(spaceInfo_);
        for (int i = nodePath.size() - 1 ; i >= 0 ; --i)
            path->append(nodePath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
        solved = true;
    }

    spaceInfo_->freeState(nodeA);
    if (nodeB->state)
        spaceInfo_->freeState(nodeB->state);
    delete nodeB;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RandomTree::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < nodes_->size() ; ++i)
    {
        if ((*nodes_)[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex((*nodes_)[i]->state));
        else
            data.addEdge(base::PlannerDataVertex((*nodes_)[i]->parent->state),
                         base::PlannerDataVertex((*nodes_)[i]->state));
    }
}

