#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"

#include "dual_manipulation_shared/serialization_utils.h"

// to generate transformations
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

// definitions used to generate grasps for a cylinder
#define H 0.2
#define R 0.05
#define mid_palm_height 0.0 //0.05
#define grasp_distance 0.05

std::vector<std::string> ee_vector{"left_hand","right_hand","table"};
std::vector<double> x_rot{/*-M_PI/2.0,*/M_PI/2.0};
std::vector<double> z_transl{-H/4.0,0.0,H/4.0};
std::vector<double> z_rot{-M_PI/2.0,/*0.0,*/M_PI/2.0/*,M_PI*/};
std::vector<double> y_rot{-M_PI/2.0,M_PI/2.0};

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_serialization "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_serialization");

    dual_manipulation_shared::ik_service srv;
    
    srv.request.command = "grasp";
    
    // create an object for grasping
    moveit_msgs::AttachedCollisionObject& attached_object = srv.request.attObject;
    trajectory_msgs::JointTrajectory& grasp_trajectory = srv.request.grasp_trajectory;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    
    // hand: only open to closed
    traj_point.positions.push_back(0.0);
    grasp_trajectory.points.push_back(traj_point);
    traj_point.positions.clear();
    traj_point.positions.push_back(1.0);
    grasp_trajectory.points.push_back(traj_point);
    
    geometry_msgs::Pose obj_pose, ee_pose;
    KDL::Frame common_frame, obj_frame;
    
    common_frame.M.RotX(0.0);
    common_frame.p.data[0] = 0.0;
    common_frame.p.data[1] = 0.0;
    common_frame.p.data[2] = 0.0;
    
    attached_object.object.id = "Cylinder";
    // this will be interpreted as the object ID (to read in the DB)
    attached_object.weight = 1.0;

    // grasp counter!
    int counter = 0;
    
    for(auto ee:ee_vector)
    {
      if(ee == "left_hand" || ee == "right_hand")
      {
	attached_object.link_name = ee + "_palm_link";
	grasp_trajectory.joint_names.clear();
	grasp_trajectory.joint_names.push_back(ee + "_synergy_joint");
      }
      else
      {
	attached_object.link_name = "world";
	grasp_trajectory.joint_names.clear();
      }
      
      // the frame where the object position is considered (only when inserted)
      attached_object.object.header.frame_id = attached_object.link_name; //"world";
      
      //side grasps
      if(ee == "left_hand" || ee == "right_hand")
      {
	common_frame.p.data[0] = R + grasp_distance;
	common_frame.p.data[2] = mid_palm_height;
      }
      else
      {
	common_frame.p.data[0] = R;
	common_frame.p.data[2] = 0.0;
      }

      for (auto x_rot_i:x_rot)
      {
	for (auto z_transl_i:z_transl)
	{
	  // if hands, do not consider mid-height
	  if ((ee == "left_hand" || ee == "right_hand") && (z_transl_i==0.0))
	    continue;
	  // if ee table: do not translate
	  else if (!(ee == "left_hand" || ee == "right_hand") && (z_transl_i!=0.0))
	    continue;
	    
	  for (auto z_rot_i:z_rot)
	  {
	    obj_frame = common_frame;
	    
	    // for the table, align x with z (rotation around "fixed" y of -pi/2)
	    if(!(ee == "left_hand" || ee == "right_hand"))
	    {
	      obj_frame = KDL::Frame( KDL::Rotation::RotY(-M_PI/2.0) )*obj_frame;
	    }
	    
	    obj_frame.M.DoRotX(x_rot_i);
	    obj_frame.p += obj_frame.M*KDL::Vector(0,0,z_transl_i);
	    obj_frame.M.DoRotZ(z_rot_i);
	    
	    // convert from KDL to geometry_msgs
	    tf::poseKDLToMsg(obj_frame,obj_pose);
	    tf::poseKDLToMsg(obj_frame.Inverse(),ee_pose);
	    
	    attached_object.object.mesh_poses.clear();
	    attached_object.object.mesh_poses.push_back(obj_pose);
	    
	    srv.request.ee_pose.clear();
	    srv.request.ee_pose.push_back(ee_pose);
	    
	    // save the obtained grasp
	  if(serialize_ik(srv.request,"object" + std::to_string((int)attached_object.weight) + "/grasp" + std::to_string(counter++)))
	    std::cout << "Serialization object" + std::to_string((int)attached_object.weight) << "/grasp" << counter-1 << " OK!" << std::endl;
// 	  if(serialize_ik(srv.request,"grasp" + std::to_string(counter++)))
// 	      std::cout << "Serialization " << counter-1 << " OK!" << std::endl;
	    else
	    {
	      std::cout << "Error in serialization object" + std::to_string((int)attached_object.weight) << "/grasp" << counter-1 << "!" << std::endl;
	      return -1;
	    }
	  }
	}
      }
      
      //top/bottom grasps
      if(ee == "left_hand" || ee == "right_hand")
      {
	common_frame.p.data[0] = H/2.0 + grasp_distance;
	common_frame.p.data[2] = mid_palm_height;
      }
      else
      {
	common_frame.p.data[0] = H/2.0;
	common_frame.p.data[2] = 0.0;
      }

      for (auto y_rot_i:y_rot)
      {
// 	for (auto z_rot_i:z_rot)
	{
	  obj_frame = common_frame;
	  
	  // for the table, align x with z (rotation around "fixed" y of -pi/2)
	  if(!(ee == "left_hand" || ee == "right_hand"))
	  {
	    obj_frame = KDL::Frame( KDL::Rotation::RotY(-M_PI/2.0) )*obj_frame;
	  }
	
	  obj_frame.M.DoRotY(y_rot_i);
// 	  obj_frame.M.DoRotZ(z_rot_i);
	  
	  // convert from KDL to geometry_msgs
	  tf::poseKDLToMsg(obj_frame,obj_pose);
	  tf::poseKDLToMsg(obj_frame.Inverse(),ee_pose);
	  
	  attached_object.object.mesh_poses.clear();
	  attached_object.object.mesh_poses.push_back(obj_pose);
	  
	  srv.request.ee_pose.clear();
	  srv.request.ee_pose.push_back(ee_pose);
	  
	  // save the obtained grasp
	  if(serialize_ik(srv.request,"object" + std::to_string((int)attached_object.weight) + "/grasp" + std::to_string(counter++)))
	    std::cout << "Serialization object" + std::to_string((int)attached_object.weight) << "/grasp" << counter-1 << " OK!" << std::endl;
	  else
	  {
	    std::cout << "Error in serialization object" + std::to_string((int)attached_object.weight) << "/grasp" << counter-1 << "!" << std::endl;
	    return -1;
	  }
	}
      }
      
    }
    
    // std::cout << "srv serialization length: " << ros::serialization::serializationLength(srv.request) << std::endl;
    // srv.request.attObject = attached_object;
    // 
    // if(serialize_ik(srv.request))
    //   std::cout << "Serialization OK!" << std::endl;
    // else
    //   return -1;
    // 
    // dual_manipulation_shared::ik_service req2;
    // 
    // if (deserialize_ik(req2.request))
    //   std::cout << "Deserialization OK!" << std::endl;
    // else
    //   return -1;

    // std::cout << req2.request << std::endl;
    
    ros::spinOnce();
    
    return 0;
    
}