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
#include "dual_manipulation_shared/databasemapper.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <visualization_msgs/Marker.h>

// distance threshold to consider a transition valid
#define DIST_THRESHOLD 0.18 // distance for movable end-effectors
#define DOTPROD_THRESHOLD 0.87 // cos(30 deg), allowed angular distance for x-axis of movable end-effector and z-axis of non-movable ones
#define DIST_THRESHOLD_FIXED 0.14 // distance for movable and non-movable end-effectors

void publish_marker_utility(ros::Publisher& vis_pub, geometry_msgs::Pose& pose, double scale=0.05, int marker_id=0)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = scale/2.0;
  marker.scale.y = scale;
  marker.scale.z = scale*2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_create_grasp_transitions "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_create_grasp_transitions");

    ros::NodeHandle n;
    ros::ServiceClient client_obj = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    dual_manipulation_shared::ik_service srv;
    dual_manipulation_shared::scene_object_service srv_obj;
    
    std::vector<std::string> grasp_links;
    std::vector<geometry_msgs::Pose> grasp_poses;
    
    databaseMapper db_mapper;
    
    std::map<std::string,int> ee_map;
    ee_map["left_hand_palm_link"] = 1;
    ee_map["right_hand_palm_link"] = 2;
    ee_map["world"] = 3;
    
    for (auto item:db_mapper.EndEffectors)
      std::cout << "Ee ID " << item.first << " : " << item.second << std::endl;
    
    srv_obj.request.command = "add";
    
    bool ok = true;
    int i = 0;
    
    int object_id = 1;
    
    while (ok)
    {
      
      ok = deserialize_ik(srv.request,"object" + std::to_string(object_id) + "/grasp" + std::to_string(i++));
      
      if (ok)
      {
	std::cout << "Deserialization object" + std::to_string(object_id) << "/grasp" << i-1 << " OK!" << std::endl;

	grasp_links.push_back(srv.request.attObject.object.header.frame_id);
	grasp_poses.push_back(srv.request.ee_pose.back());
	
	// usleep(200000);
      }
      else
      {
	std::cout << "Stopped at deserialization object" + std::to_string(object_id) << "/grasp" << i-1 << std::endl;
      }
    }
    
    srv_obj.request.attObject = srv.request.attObject;
    srv_obj.request.attObject.object.mesh_poses.clear();
    geometry_msgs::Pose neutral,marker_pose;
    neutral.position.x = -0.5;
    neutral.position.y = 0.0;
    neutral.position.z = 0.5;
    neutral.orientation.w = 1.0;
    srv_obj.request.attObject.object.mesh_poses.push_back(neutral);
    
    // client_obj.call(srv_obj);
    
    KDL::Frame cyl_kdl;
    tf::poseMsgToKDL(neutral,cyl_kdl);
    
    std::cout << "Allowed transitions map (auto-generated for different end-effectors and distance > " << DIST_THRESHOLD << ": " << std::endl;
    std::cout << "(you will probably need some manual correction to this!)" << std::endl;
    
    for (int i=0; i<grasp_poses.size(); ++i)
    {
      KDL::Frame fi;
      tf::poseMsgToKDL(grasp_poses.at(i),fi);
      
      // // if I want to show a marker related to the grasp pose
      // tf::poseKDLToMsg(cyl_kdl*(fi),marker_pose);
      // 
      // publish_marker_utility(vis_pub,marker_pose);
      // 
      // if (grasp_links.at(i) != "world")
      //   usleep(200000);
      // else
      //   usleep(200000);
      //   // sleep(2);
      
      for (int j=0; j<grasp_poses.size(); ++j)
      {
	// if the end-effector is the same, go ahead
	if (grasp_links.at(i) == grasp_links.at(j))
	  continue;

	// if no end-effector is movable, go ahead
	if (!((db_mapper.EndEffectors.at(ee_map.at(grasp_links.at(i))).movable) || (db_mapper.EndEffectors.at(ee_map.at(grasp_links.at(j))).movable)))
	  continue;
	
	KDL::Frame fj;
	tf::poseMsgToKDL(grasp_poses.at(j),fj);
	
	double dist = (fi.p - fj.p).Norm();
	
	KDL::Vector sel_x(1,0,0);
	KDL::Vector sel_z(0,0,1);
	
	bool condition_dot_prod = false;
	bool condition_distance = dist > DIST_THRESHOLD;
	
	// if the i-th end-effector is not movable
	if (!(db_mapper.EndEffectors.at(ee_map.at(grasp_links.at(i))).movable))
	{
	  // dot product of z-axis of fi and x-axis of fj + small distance
 	  condition_dot_prod = ( dot( fi.M*sel_z, fj.M*sel_x ) < -1*DOTPROD_THRESHOLD) && (dist > DIST_THRESHOLD_FIXED);
	}
	// else, if the j-th end-effector is not movable
	else if (!(db_mapper.EndEffectors.at(ee_map.at(grasp_links.at(j))).movable))
	{
	  // dot product of z-axis of fj and x-axis of fi + small distance
	  condition_dot_prod = ( dot( fj.M*sel_z, fi.M*sel_x ) < -1*DOTPROD_THRESHOLD) && (dist > DIST_THRESHOLD_FIXED);
	}
	// both movable, distance is enough
	else
	{
	}
	
	if (condition_distance || condition_dot_prod)
	{
	  //std::cout << "OK: ";
	  std::cout << i << " " << j;
	  //std::cout << " distance is: " << dist;
	  std::cout << std::endl;
	}

// 	std::cout << i << " " << j;
// 	std::cout << " distance is: " << dist;
// 	std::cout << " |cos(theta)| is: " << std::abs( dot( fi.M*sel_z, fj.M*sel_x ) );
// 	std::cout << std::endl;
      }
    }
    
    ros::spinOnce();
    return 0;
}