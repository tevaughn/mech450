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

/* Author: Ioan Sucan */

#include "SMR.h"
#include "Main.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/goals/GoalState.h>
#include <limits>

ompl::control::SMR::SMR(const SpaceInformationPtr &si, const std::vector<Control*> c, int n, int m) : base::Planner(si, "SMR")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
	std::cout << siC_ << " WE GOT A SIC\n";
    addIntermediateStates_ = false;
    lastGoalMotion_ = NULL;
	controls = c;
    n_ = n;
    m_ = m;

    goalBias_ = 0.05;

    Planner::declareParam<double>("goal_bias", this, &SMR::setGoalBias, &SMR::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &SMR::setIntermediateStates, &SMR::getIntermediateStates);
}

ompl::control::SMR::~SMR()
{
    freeMemory();
}

void ompl::control::SMR::setup()
{
    base::Planner::setup();
    if (!nn_) {
        nn_.reset(new NearestNeighborsLinear<Motion*>());
    	nn_->setDistanceFunction(boost::bind(&SMR::distanceFunction, this, _1, _2));
	}
}

void ompl::control::SMR::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::control::SMR::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            if (motions[i]->control)
                siC_->freeControl(motions[i]->control);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::control::SMR::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    std::vector<base::State *> sampledStates;

    while (const base::State *st = pis_.nextStart())
    {
		base::State *addstate;
		addstate = si_->allocState();
        si_->copyState(addstate, st);
		sampledStates.push_back(addstate);

    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    //OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion      *rmotion = new Motion(siC_);
    base::State  *rstate = rmotion->state;
    Control       *rctrl = rmotion->control;
    base::State  *xstate = si_->allocState();

 

    while (ptc == false)
    {

	    /* Learning phase */
        std::cout << "starting to learn\n";

		base::State *addstate;
        sampledStates.push_back(rstate);

		addstate = si_->allocState();
		goal_s->sampleGoal(addstate);
		sampledStates.push_back(addstate);

        for (int i = 1; i < n_; i++) {
			addstate = si_->allocState();
            sampler_->sampleUniform(addstate);
            if (si_->getStateValidityChecker()->isValid(addstate)) {
                sampledStates.push_back(addstate);
            } else {
                i--;
            }
        }

        double total = 0;
        for (base::State *state : sampledStates) {
            for (Control *control : controls) {
                for (int j = 0; j < m_; j++) {
			        addstate = si_->allocState();

                    siC_->propagate(state, control, 1, addstate);
                	if (si_->getStateValidityChecker()->isValid(addstate)) {
		                bool foundState = false;
		                for (base::State *checkstate :  sampledStates) {
		                    if (checkstate != state) {
		                        ompl::base::CompoundState *addc = addstate->as<ompl::base::CompoundState>();
		                        ompl::base::SE2StateSpace::StateType* addSE2 = addc->as<ompl::base::SE2StateSpace::StateType>(0);
		                        ompl::base::CompoundState *checkc = checkstate->as<ompl::base::CompoundState>();
		                        ompl::base::SE2StateSpace::StateType* checkSE2 = checkc->as<ompl::base::SE2StateSpace::StateType>(0);


		                        if (std::abs(checkSE2->getX() - addSE2->getX()) < 1 && 
		                            std::abs(checkSE2->getY() - addSE2->getY()) < 1 && 
		                            std::abs(checkSE2->getYaw() - addSE2->getYaw()) < 30) {

		                            tprobs[state][control][checkstate] += 1;
		                            foundState = true;
		                            break;
		                        } 
		                    }
		                }	

		                if (!foundState) {
		                    //std::cout << "found nothing close\n";
		                }
					}			
                }
            }
        }
        std::cout <<"PROBS\n";
        for (base::State *state : sampledStates) {
            for (std::pair<Control*, std::map<base::State*, double>> control : tprobs[state]) {
                for (std::pair<base::State*, double> otherState : control.second) {
                    std::cout << tprobs[state][control.first][otherState.first] << "\n";
                    tprobs[state][control.first][otherState.first] /= m_;
                    std::cout << tprobs[state][control.first][otherState.first] << "\n";
                }
            }
        }
        std::cout << "NO MO PROBS\n";




        /* Query phase */	

        ompl::base::State *goals = (goal->as<ompl::base::GoalState>())->getState();
        ompl::base::CompoundStateSpace::StateType *goalc = goals->as<ompl::base::CompoundStateSpace::StateType>();
        ompl::base::SE2StateSpace::StateType* goalSE2 = goalc->as<ompl::base::SE2StateSpace::StateType>(0);

        std::map<base::State*, double> v;
        bool itsAMatch = false;
        std::map<base::State*, Control*> pi;

        for (base::State *state: sampledStates) {
            v[state] = 0.0;
            pi[state] = NULL;
            //std::cout << state << "\n";
        }

        for (std::pair<base::State*, Control*> s : pi) {
            //std::cout << s.first << " " << s.second << "\n";
        }

        while (!itsAMatch) {
            itsAMatch = true;
            //computeOptimalPolicy(v, &pi, goal);
            std::cout << "policy\n";
            std::map<base::State*, double> newV;
            // for (std::pair<base::State*, double> state : v) {
            //     std::cout << "state: " << state.first << "\n";
            //     newV[state.first] = computeQ(v, state.first, pi[state.first], goal);
            //     std::cout << "q\n";
            //     if (v[state.first] != newV[state.first]) {
            //         itsAMatch = false;
            //     }
            // }
            // v = newV;

            std::cout << v.size() << "\n";
            for (base::State *state: sampledStates) {
                //std::cout << state << "\n";
                double maxQ = -std::numeric_limits<double>::infinity();
                Control *bestAction;
                for (auto action : tprobs[state]) {
                    double q = 0.0;
                    for (auto nextstate: tprobs[state][action.first]) {
                        //std::cout << "q " << nextstate.first << "\n";
                        //std::cout << "p=" << nextstate.second << " v=" << v[nextstate.first] << " product=" << nextstate.second*v[nextstate.first] << "\n";
                        //std::cout << (std::find(v.begin(), v.end(), nextstate) == v.end()) << "\n";
                        if (nextstate.second*v[nextstate.first] > 1 ) {//[nextstate.first] > 0) {
                            std::cout << "p=" << nextstate.second << " v=" << v[nextstate.first] << " product=" << nextstate.second*v[nextstate.first] << "\n"; 
                        }
                        q += (nextstate.second * v[nextstate.first]);
                    }
                    if (q > maxQ) {
                        maxQ = q;
                        bestAction = action.first;
                    }
                }
                //std::cout << "max q: " << maxQ << "\n";
                double reward = 0.0;

                ompl::base::CompoundState *statec = state->as<ompl::base::CompoundState>();
                ompl::base::SE2StateSpace::StateType* stateSE2 = statec->as<ompl::base::SE2StateSpace::StateType>(0);
            
                //std::cout << stateSE2->getX() << " " << goalSE2->getX() << " " << stateSE2->getY() << " " << goalSE2->getY() << " " << stateSE2->getYaw() << " " << goalSE2->getYaw() << "\n";
                if (std::abs(stateSE2->getX() - goalSE2->getX()) == 0 && 
                    std::abs(stateSE2->getY() - goalSE2->getY()) == 0 && 
                    std::abs(stateSE2->getYaw() - goalSE2->getYaw()) == 0) {
//                    std::cout << "GOAL STATE\n";
                    reward = 1.0;
                    maxQ = 0;
 //                   std::cout << reward + maxQ << "\n";
                }

                newV[state] = reward + maxQ;
                if (newV[state] > 0) {
                    //std::cout << "newv " << newV[state] << "\n"; 
                }
                pi[state] = bestAction;
                //std::cout << bestAction->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] << " " << bestAction->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1]<< " action \n";
                if (v[state] != newV[state]) {
                    itsAMatch = false;
                }
                //std::cout << v[state] << " " << newV[state] << "\n";
               

            }  
            v = newV;
            std::cout << "pi done " << itsAMatch << "\n";
        }






        // std::map<base::State*, Control*> pi;

        // for (base::State *state: sampledStates) {
        //     pi[state] = NULL;
        // }


        // computeOptimalPolicy(&pi, goal);


        /* Simulate running the robot according to the policy. Record the path it actually travels */
        bool finished = false;
        base::State *curstate;
		base::State *approxstate;
        base::State *nextstate = si_->allocState();
        double dist = 0.0;
        std::vector<Motion*> mpath;
        Motion *lastmotion = NULL;
        bool solved = false;

        std::cout << "V\n";
        for (auto state: sampledStates) {
            std::cout << v[state] << "\n";
        }
		
        curstate = rmotion->state;
		approxstate = curstate;
        while (!finished) {
            std::cout << "not finished\n";
            //std::cout << curstate << "\n";
			
			double mindist = std::numeric_limits<double>::infinity();

			// for (base::State *state : sampledStates) {
			// 	dist =si_->distance(state, curstate);
			// 	if (dist < mindist) {
			// 		mindist = dist;
			// 		curstate = state;
			// 	} 
			// }

			Control *ctrl = siC_->allocControl();
			ctrl = pi[curstate];
            siC_->propagate(curstate, ctrl, 1, nextstate);
            for (base::State *checkstate :  sampledStates) {
                if (checkstate != curstate) {
                    ompl::base::CompoundState *addc = nextstate->as<ompl::base::CompoundState>();
                    ompl::base::SE2StateSpace::StateType* addSE2 = addc->as<ompl::base::SE2StateSpace::StateType>(0);
                    ompl::base::CompoundState *checkc = checkstate->as<ompl::base::CompoundState>();
                    ompl::base::SE2StateSpace::StateType* checkSE2 = checkc->as<ompl::base::SE2StateSpace::StateType>(0);

                    //std::cout << checkSE2->getX() << " " << addSE2->getX() << " " << checkSE2->getY() << " " << addSE2->getY() << " " << checkSE2->getYaw() << " " << addSE2->getYaw() << "\n";
                    if (std::abs(checkSE2->getX() - addSE2->getX()) < 1 && 
                        std::abs(checkSE2->getY() - addSE2->getY()) < 1 && 
                        std::abs(checkSE2->getYaw() - addSE2->getYaw()) < 30) {


                        //std::cout << checkstate << "\n";
                        nextstate = checkstate;
                        break;
                    } 
                }
            }   

    		const ompl::base::CompoundState *state = nextstate->as<ompl::base::CompoundState>();
    		const ompl::base::SE2StateSpace::StateType* se2 = state->as<ompl::base::SE2StateSpace::StateType>(0);

			std::cout << se2->getX() << " " << se2->getY() << " " << se2->getYaw() << "\n";
			
			const ompl::control::RealVectorControlSpace::ControlType* nctrl = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>();
			std::cout << nctrl->values[0] << " " << nctrl->values[1] << " \n";
			

            //std::cout << "propagated\n";

            /* create a motion */
            Motion *motion = new Motion(siC_);
            si_->copyState(motion->state, curstate);

			//Control* control = siC_->allocControl();
			motion->control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
			motion->control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1];

			//std::cout << "copied\n";
            motion->steps = 1;
            motion->parent = lastmotion;
            nn_->add(motion);
            mpath.push_back(motion);
			//std::cout << "pushed \n";
            lastmotion = motion;

            solved = goal->isSatisfied(nextstate, &dist);

            std::cout << "check solved " << dist << "\n";
            if (solved) {
                finished = true;
                approxdif = dist;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
            }

            curstate = nextstate;
        }
        std::cout << "we made it boys\n";
        PathControl *path = new PathControl(si_);
        for (int i = 0; i < mpath.size() ; ++i) {
            std::cout << "i " << i << "\n";
            if (mpath[i]->parent) {
                std::cout << "parent exists\n";
                std::cout << mpath[i]->state << "\n";
                std::cout << mpath[i]->control << "\n";
                std::cout << mpath[i]->steps << "\n";
                Control *control = siC_->allocControl();
                path->append(mpath[i]->state, control, mpath[i]->steps * siC_->getPropagationStepSize());
            }
            else {
                std::cout << "orphan\n";
                path->append(mpath[i]->state);
            }
        }
        std::cout << "so close i can almost feel it\n";
        pdef_->addSolutionPath(base::PathPtr(path), true, approxdif, getName());
        std::cout << "mama we made it\n";      
		return base::PlannerStatus(solved, true);
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
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathControl *path = new PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::SMR::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const Motion *m = motions[i];
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                             base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}

void ompl::control::SMR::computeOptimalPolicy( std::map<base::State*, Control*> *pi, base::Goal *goal) { 

	bool matched = 0;
	while(!matched) {
		matched = 1;
    	for (auto const &state :tprobs) {
			if ( computeQ(state.first, goal) == true) {
				matched = 0;			
			}
		}
	}

    double maxQ = -std::numeric_limits<double>::infinity();
	double q = 0;
    Control *bestAction;

	for (auto const &state :tprobs) {
		for (auto const &control: state.second) {
			for (auto const &newState : control.second) {
				q = tprobs[state.first][control.first][newState.first];
				if ( q > maxQ) {
               		maxQ = q;
                	bestAction = control.first;
				}

			}
		}
		(*pi)[state.first] = bestAction;
	}


}

bool ompl::control::SMR::computeQ(base::State *state, base::Goal *goal) { 
	
	bool changed = false;

	ompl::base::State *goals = (goal->as<ompl::base::GoalState>())->getState();
    ompl::base::CompoundStateSpace::StateType *goalc = goals->as<ompl::base::CompoundStateSpace::StateType>();
    ompl::base::SE2StateSpace::StateType* goalSE2 = goalc->as<ompl::base::SE2StateSpace::StateType>(0);
    
	double max_prob = 0;
	for (auto const &control : tprobs[state]) {
		for (auto const &newstate : control.second) {
			// find the max probability of the children

        	double reward = 0.0;
			double selfprob = tprobs[state][control.first][newstate.first];

			ompl::base::CompoundState *statec = state->as<ompl::base::CompoundState>();
    		ompl::base::SE2StateSpace::StateType* stateSE2 = statec->as<ompl::base::SE2StateSpace::StateType>(0);
		
			//std::cout << stateSE2->getX() << " " << goalSE2->getX() << " " << stateSE2->getY() << " " << goalSE2->getY() << " " << stateSE2->getYaw() << " " << goalSE2->getYaw() << "\n";
        	if (std::abs(stateSE2->getX() - goalSE2->getX()) < .2 && 
            	std::abs(stateSE2->getY() - goalSE2->getY()) < .2 && 
        		std::abs(stateSE2->getYaw() - goalSE2->getYaw()) < .2) {

            	reward = 1.0;
        	}

			for (auto const &newcontrol : tprobs[newstate.first]) {
				for (auto const &finalstate : newcontrol.second) {
					double prob = tprobs[newstate.first][newcontrol.first][finalstate.first];
					if (prob > max_prob) {
						max_prob = prob;
						tprobs[state][control.first][newstate.first] = prob*selfprob + reward;	
						changed = true;				
					}				
				}
			}
		}
	}
	return changed;
}



