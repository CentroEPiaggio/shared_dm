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

#include "ros/ros.h"
// #include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/ik_service_legacy.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/grasp_trajectory.h"

void copy_from_legacy(dual_manipulation_shared::grasp_trajectory& req, const dual_manipulation_shared::ik_service_legacy::Request& req_legacy)
{
  req.ee_pose.clear();
  req.ee_pose.insert(req.ee_pose.end(),req_legacy.ee_pose.begin(),req_legacy.ee_pose.end());
  req.grasp_trajectory = req_legacy.grasp_trajectory;
  req.attObject = req_legacy.attObject;
  
  // NOTE: add here any new field which should be managed in future versions (take them from above)
  
  // 2015-04-07 : using grasp_trajectory.msg instead of ik_service::Request anywhere from now on
  // this hopefully will avoid the need to change this in future versions
  req.object_db_id = req_legacy.object_db_id;
}

bool reserialize_grasp(int obj_id, int grasp_id)
{
  dual_manipulation_shared::grasp_trajectory req;
  dual_manipulation_shared::ik_service_legacy::Request req_legacy;

  if(deserialize_ik(req_legacy,"object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id)))
    std::cout << "Deserialization of legacy object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!" << std::endl;
  else
  {
    std::cout << "Error in deserialization of legacy object" + std::to_string(obj_id) << "/grasp" << grasp_id << "!" << std::endl;
    return false;
  }
  
  copy_from_legacy(req,req_legacy);
  
  if(serialize_ik(req,"object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id)))
    std::cout << "Serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!" << std::endl;
  else
  {
    std::cout << "Error in serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << "!" << std::endl;
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> convert_ik_from_legacy "<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "convert_ik_from_legacy");

    databaseMapper dbMapper;

//     ros::NodeHandle n;
//     ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    for(auto item:dbMapper.Grasps)
    {
      reserialize_grasp(item.second.obj_id,item.first);
    }
    
    return 0;
}