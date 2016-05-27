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
#include <std_msgs/String.h>
#include <iostream>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"
#include "dual_manipulation_shared/databasemapper.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <ros/package.h>
#include <fstream>

#include <visualization_msgs/Marker.h>

void publish_marker_utility(ros::Publisher& vis_pub, visualization_msgs::Marker& marker, geometry_msgs::Pose& pose, double scale=0.05)
{
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::DURATION_MAX;
  marker.pose = pose;
  marker.scale.x = scale/2.0;
  marker.scale.y = scale;
  marker.scale.z = scale*2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );
  ros::spinOnce();
}

std::string object_selection_utility(int obj_id)
{
    std::string path = "package://dual_manipulation_grasp_db/object_meshes/";
    for(auto item:db_mapper.Objects)
    {
	if(item.first == obj_id)
	{
	    path.append(std::get<1>(item.second));
	    break;
	}
    }
    
    ROS_DEBUG_STREAM("object path: "<<path<<std::endl);

    return path;
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|dual_manipulation_shared| -> create_grasp_from_file "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_grasping");
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
    dual_manipulation_shared::ik_service srv;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1,true );

    // create an object for grasping
    moveit_msgs::AttachedCollisionObject& attached_object = srv.request.attObject;
    attached_object.link_name = "";
    // the frame where the object position is considered (only when inserted, then it's *probably* fixed in the environment)
    attached_object.object.header.frame_id = "world";
    attached_object.object.id = "cylinder";
    
    // relative translations from kuka to the hands
    KDL::Frame kuka_left_hand,kuka_right_hand;
    kuka_left_hand.p = KDL::Vector(0.002, -0.007, 0.090);
    kuka_right_hand.p = KDL::Vector(0.002, 0.007, 0.090);
    
    ROS_WARN_STREAM("Currently using left/right hand positioned (w.r.t. kuka link 7) in 0.002, -/+0.007, 0.09");

    // create markers to use in place of the object
    visualization_msgs::Marker hand_marker,obj_marker;
    databaseMapper db_mapper;

    hand_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obj_marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    hand_marker.header.frame_id = "world";
    hand_marker.header.stamp = ros::Time(0);
    hand_marker.ns = "my_namespace";
    hand_marker.id = 0;
    hand_marker.action = visualization_msgs::Marker::ADD;
    hand_marker.lifetime = ros::DURATION_MAX;
    double scale = 0.05;
    hand_marker.scale.x = scale/2.0;
    hand_marker.scale.y = scale;
    hand_marker.scale.z = scale*2;
    hand_marker.color.a = 0.5;
    hand_marker.color.r = 1.0;
    hand_marker.color.g = 0.0;
    hand_marker.color.b = 0.0;
    obj_marker.header.frame_id = "world";
    obj_marker.header.stamp = ros::Time(0);
    obj_marker.ns = "my_namespace";
    obj_marker.id = 0;
    obj_marker.action = visualization_msgs::Marker::ADD;
    obj_marker.lifetime = ros::DURATION_MAX;
    obj_marker.color.a = 1.0;
    obj_marker.color.r = 0.0;
    obj_marker.color.g = 1.0;
    obj_marker.color.b = 0.0;
    
    /////////////////////// get grasp data from file ///////////////////////
    std::string path = ros::package::getPath("dual_manipulation_shared");
    path.append("/test/grasp_data.txt");
    
    std::fstream fs;
    // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app*/);
    fs.open (path, std::fstream::in);
    if (fs.fail())
        return -1;
    
    double obj_id,obj_grasp_data[6],obj_postgrasp_data[6],hand_grasp_data[6];
    fs >> obj_id;
    for (int i=0; i<6; ++i)
        fs >> obj_grasp_data[i];
    for (int i=0; i<6; ++i)
        fs >> hand_grasp_data[i];
    
    fs.close();
    /////////////////////// get grasp data from file ///////////////////////

    std::cout << "obj_id: " << obj_id << std::endl;
    std::cout << "obj_pregrasp_data: ";
    for (int i=0; i<6; ++i)
        std::cout << obj_grasp_data[i] << " | ";
    std::cout << std::endl;
    std::cout << "hand_grasp_data: ";
    for (int i=0; i<6; ++i)
        std::cout << hand_grasp_data[i] << " | ";
    std::cout << std::endl;
    
    KDL::Frame obj_hand,world_obj,world_kuka,world_hand;
    for (int i=0; i<3; i++)
    {
      world_obj.p.data[i] = obj_grasp_data[i];
      world_kuka.p.data[i] = hand_grasp_data[i];
    }
    
    world_obj.M = KDL::Rotation::RPY(obj_grasp_data[3],obj_grasp_data[4],obj_grasp_data[5]);
    world_kuka.M = KDL::Rotation::RPY(hand_grasp_data[3],hand_grasp_data[4],hand_grasp_data[5]);
    
    // compute actual hand (which was left)
    world_hand = world_kuka*kuka_left_hand;
    
    // this is for the DB
    obj_hand = world_obj.Inverse()*world_hand;
    
    // conver to geometry_msgs for visualization
    geometry_msgs::Pose obj_pose,hand_pose;
    tf::poseKDLToMsg(world_obj,obj_pose);
    tf::poseKDLToMsg(world_hand,hand_pose);
    
    // complete visualization of object
    obj_marker.mesh_resource = object_selection_utility((int)obj_id);
    obj_marker.pose = obj_pose;
    publish_marker_utility(vis_pub,obj_pose);

    // start to push hand poses (towards approach direction)
    geometry_msgs::Point pt;
    
    hand_marker.points.push_back();
    
    ros::spinOnce();



    // first, show hand_pose using a box
    publish_marker_utility(vis_pub,hand_pose);
// //     std::cout << "waiting to continue..." << std::endl;
// //     char tmp;
// //     std::cin >> tmp;
// // 
// //     action_count++;
// //     if (client.call(srv))
// //     {
// // 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
// //     }
// //     else
// //     {
// // 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
// //     }
// //     
// //     while(action_count > 0)
// //     {
// //       ros::spinOnce();
// //       usleep(200000);
// //     }
// // 
// //     /////////////////////// actually move the arm ///////////////////////
// //     srv.request.command = "execute";
// //     
// //     action_count++;
// //     if (client.call(srv))
// //     {
// // 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
// //     }
// //     else
// //     {
// // 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
// //     }
// //     
// //     while(action_count > 0)
// //     {
// //       ros::spinOnce();
// //       usleep(200000);
// //     }

    
    /////////////////////// perform grasp ///////////////////////

//     // object to be grasped
//     attached_object.link_name = "left_hand_palm_link";
//     attached_object.object.header.frame_id = "left_hand_palm_link";
//     attached_object.object.header.stamp = ros::Time::now();
//     // object pose relative to the palm: post-grasp pose from DB!
//     obj_pose.position.x = 0.05; // primitive.dimensions.at(1); //+0.02;
//     obj_pose.position.y = 0.0;
//     obj_pose.position.z = 0.05;
//     obj_pose.orientation.x = -0.707;
//     obj_pose.orientation.y = 0.0;
//     obj_pose.orientation.z = 0.0;
//     obj_pose.orientation.w = 0.707;
// 
//     // // this does not work very well with the mesh (it gets oriented strangely)
//     // rot.EulerZYX(0.0,0.0,M_PI/2.0).GetQuaternion(obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z,obj_pose.orientation.w);
//     
//     // attached_object.object.primitive_poses.clear();
//     // attached_object.object.primitive_poses.push_back(obj_pose);
//     attached_object.object.mesh_poses.clear();
//     attached_object.object.mesh_poses.push_back(obj_pose);
//     
//     // hand joint trajectory
//     trajectory_msgs::JointTrajectory grasp_traj;
//     grasp_traj.header.stamp = ros::Time::now();
//     grasp_traj.joint_names.push_back(srv.request.ee_name + "_synergy_joint");
    q.clear();
    q.assign({0.0,1.0});
    t.clear();
    t.assign({0.0,1.0});
    
    if (!moveHand(hand,q,t))
      std::cout << "Moving the hand didn't work..." << std::endl;
    
//     trajectory_msgs::JointTrajectoryPoint tmp_traj;
//     tmp_traj.positions.reserve(1);
//     for (int i=0; i<q.size(); ++i)
//     {
//       tmp_traj.positions.clear();
//       tmp_traj.positions.push_back(q.at(i));
//       grasp_traj.points.push_back(tmp_traj);
//     }
//     
//     // complete request
//     srv.request.command = "grasp";
//     srv.request.attObject = attached_object;
//     srv.request.grasp_trajectory = grasp_traj;
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(5);
//     
//     
//     /////////////////////// move the hand somewhere ///////////////////////
//     srv.request.command = "plan";
//     srv.request.ee_name = "left_hand";
//     hand_pose.position.y -= 0.35;
//     srv.request.ee_pose.clear();
//     srv.request.ee_pose.push_back(hand_pose);
//     
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(1);
//     
//     srv.request.command = "execute";
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s request for %s accepted", srv.request.command.c_str(), srv.request.ee_name.c_str());
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(5);
//     
//     /////////////////////// perform ungrasp ///////////////////////
//     srv.request.command = "ungrasp";
//     // leave the object where it is (w.r.t. the hand, right now), but detach it from the robot
// 
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
//     
//     sleep(5);
//     
//     /////////////////////// go back home ///////////////////////
//     srv.request.command = "home";
//     if (client.call(srv))
//     {
// 	ROS_INFO("IK_control:test_grasping : %s object request accepted: %d", srv_obj.request.command.c_str(), (int)srv_obj.response.ack);
//     }
//     else
//     {
// 	ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::ik_service: %s %s",srv.request.ee_name.c_str(),srv.request.command.c_str());
//     }
    
    // // get the final pose of the object
    // get the object from the scene, once
//     gotobject_count = 1;
//     while(gotobject_count > 0)
//     {
//       ros::spinOnce();
//       usleep(200000);
//     }
    
    for (int i=0; i<gazebo_mdl.name.size(); i++)
    {
      // std::cout << gazebo_mdl.name.at(i) << ":" << std::endl << gazebo_mdl.pose.at(i) << std::endl;
      if(gazebo_mdl.name.at(i)=="cylinder")
      {
	obj_pose = gazebo_mdl.pose.at(i);
	obj_pose.position.z -= 1.0;
      }
    }
    
    std::cout << "obj_pose from gazebo:" << std::endl;
    std::cout << obj_pose << std::endl;

    // clear stuff at the end
    for(int i=0; i<3; i++)
    {
      usleep(200000);
      ros::spinOnce();
    }

    return 0;
}