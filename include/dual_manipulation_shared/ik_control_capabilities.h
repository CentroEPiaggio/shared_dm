/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Alessandro Settimi, Hamal Marino, Mirko Ferrati, Centro di Ricerca "E. Piaggio", University of Pisa
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
*   * Neither the name of the ISR University of Coimbra nor the names of its
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

#ifndef IK_CONTROL_CAPABILITIES_H
#define IK_CONTROL_CAPABILITIES_H

#include <map>
#include <string>

#define SET_TARGET_CAPABILITY "set_target"
#define SET_HOME_TARGET_CAPABILITY "set_home_target"
#define IK_CHECK_CAPABILITY "ik_check"
#define PLAN_CAPABILITY "plan"
#define PLAN_NO_COLLISION_CAPABILITY "plan_no_collision"
#define PLAN_BEST_EFFORT_CAPABILITY "plan_best_effort"
#define PLAN_CLOSE_BEST_EFFORT_CAPABILITY "plan_close_best_effort"
#define MOVE_CAPABILITY "execute"
#define GRASP_CAPABILITY "grasp"
#define UNGRASP_CAPABILITY "ungrasp"
#define HOME_CAPABILITY "home"

#define SET_TARGET_MSG "set_target_done"
#define SET_HOME_TARGET_MSG "set_target_done"
#define IK_CHECK_MSG "check_done"
#define PLAN_MSG "planning_done"
#define PLAN_NO_COLLISION_MSG "planning_done"
#define PLAN_BEST_EFFORT_MSG "planning_done"
#define PLAN_CLOSE_BEST_EFFORT_MSG "planning_done"
#define MOVE_MSG "action_done"
#define GRASP_MSG "grasp_done"
#define UNGRASP_MSG "ungrasp_done"
#define HOME_MSG "action_done"

enum class ik_control_capability_types
{
  SET_TARGET,
  IK_CHECK,
  PLAN,
  MOVE,
  GRASP
};

enum class ik_control_capabilities
{
  SET_TARGET,
  SET_HOME_TARGET,
  IK_CHECK,
  PLAN,
  PLAN_NO_COLLISION,
  PLAN_BEST_EFFORT,
  PLAN_CLOSE_BEST_EFFORT,
  MOVE,
  GRASP,
  UNGRASP,
  HOME
};

class ik_control_capability
{
public:
  ik_control_capability()
  {
    name[ik_control_capabilities::SET_TARGET] = SET_TARGET_CAPABILITY;
    name[ik_control_capabilities::SET_HOME_TARGET] = SET_HOME_TARGET_CAPABILITY;
    name[ik_control_capabilities::IK_CHECK] = IK_CHECK_CAPABILITY;
    name[ik_control_capabilities::PLAN] = PLAN_CAPABILITY;
    name[ik_control_capabilities::PLAN_NO_COLLISION] = PLAN_NO_COLLISION_CAPABILITY;
    name[ik_control_capabilities::PLAN_BEST_EFFORT] = PLAN_BEST_EFFORT_CAPABILITY;
    name[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT] = PLAN_CLOSE_BEST_EFFORT_CAPABILITY;
    name[ik_control_capabilities::MOVE] = MOVE_CAPABILITY;
    name[ik_control_capabilities::GRASP] = GRASP_CAPABILITY;
    name[ik_control_capabilities::UNGRASP] = UNGRASP_CAPABILITY;
    name[ik_control_capabilities::HOME] = HOME_CAPABILITY;
    
    from_name[SET_TARGET_CAPABILITY] = ik_control_capabilities::SET_TARGET;
    from_name[SET_HOME_TARGET_CAPABILITY] = ik_control_capabilities::SET_HOME_TARGET;
    from_name[IK_CHECK_CAPABILITY] = ik_control_capabilities::IK_CHECK;
    from_name[PLAN_CAPABILITY] = ik_control_capabilities::PLAN;
    from_name[PLAN_NO_COLLISION_CAPABILITY] = ik_control_capabilities::PLAN_NO_COLLISION;
    from_name[PLAN_BEST_EFFORT_CAPABILITY] = ik_control_capabilities::PLAN_BEST_EFFORT;
    from_name[PLAN_CLOSE_BEST_EFFORT_CAPABILITY] = ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT;
    from_name[MOVE_CAPABILITY] = ik_control_capabilities::MOVE;
    from_name[GRASP_CAPABILITY] = ik_control_capabilities::GRASP;
    from_name[UNGRASP_CAPABILITY] = ik_control_capabilities::UNGRASP;
    from_name[HOME_CAPABILITY] = ik_control_capabilities::HOME;
    
    msg[ik_control_capabilities::SET_TARGET] = SET_TARGET_MSG;
    msg[ik_control_capabilities::SET_HOME_TARGET] = SET_HOME_TARGET_MSG;
    msg[ik_control_capabilities::IK_CHECK] = IK_CHECK_MSG;
    msg[ik_control_capabilities::PLAN] = PLAN_MSG;
    msg[ik_control_capabilities::PLAN_NO_COLLISION] = PLAN_NO_COLLISION_MSG;
    msg[ik_control_capabilities::PLAN_BEST_EFFORT] = PLAN_BEST_EFFORT_MSG;
    msg[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT] = PLAN_CLOSE_BEST_EFFORT_MSG;
    msg[ik_control_capabilities::MOVE] = MOVE_MSG;
    msg[ik_control_capabilities::GRASP] = GRASP_MSG;
    msg[ik_control_capabilities::UNGRASP] = UNGRASP_MSG;
    msg[ik_control_capabilities::HOME] = HOME_MSG;
    
    type[ik_control_capabilities::SET_TARGET] = ik_control_capability_types::SET_TARGET;
    type[ik_control_capabilities::SET_HOME_TARGET] = ik_control_capability_types::SET_TARGET;
    type[ik_control_capabilities::IK_CHECK] = ik_control_capability_types::IK_CHECK;
    type[ik_control_capabilities::PLAN] = ik_control_capability_types::PLAN;
    type[ik_control_capabilities::PLAN_NO_COLLISION] = ik_control_capability_types::PLAN;
    type[ik_control_capabilities::PLAN_BEST_EFFORT] = ik_control_capability_types::PLAN;
    type[ik_control_capabilities::PLAN_CLOSE_BEST_EFFORT] = ik_control_capability_types::PLAN;
    type[ik_control_capabilities::MOVE] = ik_control_capability_types::MOVE;
    type[ik_control_capabilities::GRASP] = ik_control_capability_types::GRASP;
    type[ik_control_capabilities::UNGRASP] = ik_control_capability_types::GRASP;
    type[ik_control_capabilities::HOME] = ik_control_capability_types::MOVE;
    
    implemented_for_trees[ik_control_capability_types::SET_TARGET] = true;
    implemented_for_trees[ik_control_capability_types::IK_CHECK] = false;
    implemented_for_trees[ik_control_capability_types::PLAN] = true;
    implemented_for_trees[ik_control_capability_types::MOVE] = true;
    implemented_for_trees[ik_control_capability_types::GRASP] = false;
  }
  
  ~ik_control_capability(){};
  
  std::map<ik_control_capabilities,std::string> name;
  std::map<std::string,ik_control_capabilities> from_name;
  std::map<ik_control_capabilities,std::string> msg;
  
  std::map<ik_control_capabilities,ik_control_capability_types> type;
  std::map<ik_control_capability_types,bool> implemented_for_trees;
};

#endif // IK_CONTROL_CAPABILITIES_H
