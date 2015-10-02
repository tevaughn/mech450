/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef OMPL_GEOMETRIC_PLANNERS_RandomTree
#define OMPL_GEOMETRIC_PLANNERS_RandomTree

#include "ompl/geometric/planners/PlannerIncludes.h"
//#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
namespace geometric
{

	/*
		Random Tree

		Finds a path from start point to goal with following method:
		Pick random configuration A on tree, sample new random 
		configuration B in the space, with goal_bias probability
		of choosing the goal space. Determine if path AB is
		collision free. If so, add path AB to tree. Else, discard B.
		Repeat until goal has been added to tree	
	*/

	/* Random Tree */
	class RandomTree : public base::Planner {
		public:

		/* Constructor */
		RandomTree(const base::SpaceInformationPtr &spaceInfo);

		virtual ~RandomTree();
		virtual void getPlannerData(base::PlannerData &data) const;
		virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &terminationCondition);
		virtual void clear();

        /*
			Set the goal bias

		    In the process of randomly selecting states in
		    the state space to attempt to go towards, the
		    algorithm may in fact choose the actual goal state, if
		    it knows it, with some probability. This probability
		    is a real number between 0.0 and 1.0. 0.05 is default 
		*/

        void setGoalBias(double goalBias) {
            goalBias_ = goalBias;
        }

        double getGoalBias() const {
            return goalBias_;
        }

        /*
			Set the range the planner is supposed to use.

            This parameter greatly influences the runtime of the
            algorithm. It represents the maximum length of a
            motion to be added in the tree of motions. 
		*/
        void setRange(double distance) {
            maxDistance_ = distance;
        }
        double getRange() const {
            return maxDistance_;
        }

        virtual void setup();

    protected:


        /* 
			Nodes on Random Tree

            Contains state (position) and pointer to parent motion
		*/
        class Node
        {
        public:

            Node() : state(NULL), parent(NULL)
            {
            }

            /* Constructor that allocates memory for the state */
            Node(const base::SpaceInformationPtr &spaceInfo) : state(spaceInfo->allocState()), parent(NULL) {}
            ~Node() {}

            /* The state (position) contained by the node */
            base::State	*state;

            /* The parent node in the exploration tree */
            Node	*parent;

        };

        /* Free the memory allocated by this planner */
        void freeMemory();


        /* State sampler */
        base::StateSamplerPtr	sampler_;

        /* The probability goal state is picked over random sample for configuration B */
        double				goalBias_;

        /* The maximum length of a motion to be added to a tree */
        double				maxDistance_;

        /* The random number generator */
        RNG				rng_;

        /* Most recent goal motion.  Used for PlannerData computation */
        Node				*lastGoalMotion_;

	/* Vector storing pointers to all nodes in the tree for choosing random node A */
	boost::shared_ptr< std::vector<Node*> > 	nodes_;
	
    	const base::SpaceInformationPtr &spaceInfo_;
		
	};

}
}

#endif
