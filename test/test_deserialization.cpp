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
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include "dual_manipulation_shared/serialization_utils.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <visualization_msgs/Marker.h>

#define OBJ_ID 3.0

void publish_marker_utility(ros::Publisher& vis_pub, geometry_msgs::Pose& pose, int id, std::string name, double scale=0.05)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time(0);
    marker.ns = "my_namespace";
    marker.id = id;
    if(name=="ee")
    {
// 	marker.type = visualization_msgs::Marker::CUBE;
// 	marker.scale.x = scale/2.0;
// 	marker.scale.y = scale;
// 	marker.scale.z = scale*2;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://soft_hand_description/meshes/palm_right.stl";
	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.scale.z = 0.001;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	if(id==0)
	{
	  marker.color.g = 1.0;
	}
	else if (id==100)
	{
	  marker.color.r = 0.5;
	  marker.color.b = 0.5;
	}
	else
	{
	  marker.color.r = 1.0;
	}
    }
    if(name=="obj")
    {
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.scale.x = 0.053;
	marker.scale.y = 0.053;
	marker.scale.z = 0.245;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
    }
    if(name=="remove")
    {
	marker.action = visualization_msgs::Marker::DELETE;
    }
    
    vis_pub.publish( marker );
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_deserialization "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_deserialization");

    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    dual_manipulation_shared::ik_service srv;
    dual_manipulation_shared::scene_object_service srv_obj;
    
    std::vector<std::string> grasp_links;
    std::vector<geometry_msgs::Pose> grasp_poses;
    geometry_msgs::Pose obj_pose;
    obj_pose.orientation.w = 1.0;
    
    bool ok = true;
    std::vector<int> grasp_ids = {37,37,38}; //{16,16,19,24,25}; //,26,27,28,29};
    
    int object_id = (int)OBJ_ID;
    int id;
    
    char a;
    for (auto i:grasp_ids)
    {
      
      ok = deserialize_ik(srv.request,"object" + std::to_string(object_id) + "/grasp" + std::to_string(i));
      
      if (ok)
      {
	std::cout << "Deserialization object" + std::to_string(object_id) << "/grasp" << i << " OK!" << std::endl;
	
	publish_marker_utility(vis_pub,obj_pose,-1,"obj");
	
	id = 0;
	for(auto item:srv.request.ee_pose)
	{
	    publish_marker_utility(vis_pub,item,id++,"ee");
	    // std::cout << "id:" << id << " | distance=" << std::sqrt(item.position.x*item.position.x+item.position.y*item.position.y+item.position.z*item.position.z) << std::endl;;
	}
	// std::cout << std::endl;
	
	KDL::Frame post_grasp_frame;
	geometry_msgs::Pose post_grasp_pose;
	tf::poseMsgToKDL(srv.request.attObject.object.mesh_poses.front(),post_grasp_frame);
	tf::poseKDLToMsg(post_grasp_frame.Inverse(),post_grasp_pose);
	publish_marker_utility(vis_pub,post_grasp_pose,100,"ee");

	ros::spinOnce();
	
	std::cout << "Waiting for input..." << std::endl;
	std::cin >> a;
	
      }
      else
      {
	std::cout << "[Deserialization error] Unable to deserialize : object" + std::to_string(object_id) << "/grasp" << i << std::endl;
	break;
      }

      for(auto item:srv.request.ee_pose)
      {
	  publish_marker_utility(vis_pub,item,--id,"remove");
      }
      geometry_msgs::Pose tmp;
      publish_marker_utility(vis_pub,tmp,100,"remove");
      ros::spinOnce();
      
    }
    
    id = srv.request.ee_pose.size();
    for(auto item:srv.request.ee_pose)
    {
	publish_marker_utility(vis_pub,item,--id,"remove");
    }
    
    ros::spinOnce();
    return 0;
}