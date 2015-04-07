#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include "dual_manipulation_shared/grasp_trajectory.h"

#define END_EFFECTOR_ID 3		// the ID of the end-effector (table) to consider
#define END_EFFECTOR_FRAME "world"	// the frame to be used in the request
#define YAW_STEPS 8 			// how many steps to use when rotating the grasp
#define WAYPOINT_HEIGHT 0.1		// height of the waypoint used for pre-ungrasp and post-grasp

bool read_data_from_file(std::string& obj_name, std::string& grasp_name, KDL::Frame& obj_ee_final, std::string filename = "/grasp_storage/table_grasp_data.txt")
{
  std::string path = ros::package::getPath("dual_manipulation_shared");
  path.append(filename);
  
  std::fstream fs;
  // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app*/);
  fs.open (path, std::fstream::in);
  if (fs.fail())
    return false;
  
  double obj_grasp_data[6];
  
  fs >> obj_name;
  fs >> grasp_name;
  for (int i=0; i<6; ++i)
    fs >> obj_grasp_data[i];
  
  fs.close();
  
  for (int i=0; i<3; i++)
    obj_ee_final.p.data[i] = obj_grasp_data[i];
  obj_ee_final.M = KDL::Rotation::RPY(obj_grasp_data[3],obj_grasp_data[4],obj_grasp_data[5]);

  return true;
}

bool serialize_data(const dual_manipulation_shared::grasp_trajectory& grasp_msg, int object_id, int grasp_id)
{
  // save the obtained grasp
  if(serialize_ik(grasp_msg,"object" + std::to_string(object_id) + "/grasp" + std::to_string(grasp_id)))
  {
    ROS_INFO_STREAM("Serialization object" + std::to_string(object_id) << "/grasp" + std::to_string(grasp_id) << " OK!");
  }
  else
  {
    ROS_ERROR_STREAM("In serialization object" + std::to_string(object_id) << "/grasp" + std::to_string(grasp_id));
    return false;
  }
  
  return true;
}

void build_grasp_msg(dual_manipulation_shared::grasp_trajectory& grasp_msg, const KDL::Frame& obj_ee_frame, std::string obj_name, int obj_id, std::string ee_frame_name = END_EFFECTOR_FRAME)
{
  // create an object for grasping
  moveit_msgs::AttachedCollisionObject& attached_object = grasp_msg.attObject;
  u_int64_t& object_db_id = grasp_msg.object_db_id;
  std::vector<geometry_msgs::Pose>& ee_pose = grasp_msg.ee_pose;
  
  // // NOTE: this commented part could be useful when building grasps for hands
  // trajectory_msgs::JointTrajectory& grasp_trajectory = grasp_msg.grasp_trajectory;
  // trajectory_msgs::JointTrajectoryPoint traj_point;
  // // hand: only open to closed
  // traj_point.positions.push_back(0.0);
  // grasp_trajectory.points.push_back(traj_point);
  // traj_point.positions.clear();
  // traj_point.positions.push_back(1.0);
  // grasp_trajectory.points.push_back(traj_point);
  // std::string ee = ee_name_map.at(end_effector_id);
  // if(ee == "left_hand" || ee == "right_hand")
  // {
  //   attached_object.link_name = ee + "_palm_link";
  //   grasp_trajectory.joint_names.clear();
  //   grasp_trajectory.joint_names.push_back(ee + "_synergy_joint");
  // }
  // else
  
  attached_object.link_name = ee_frame_name;

  attached_object.object.id = obj_name;
  // this will be used to read the DB
  object_db_id = obj_id;

  geometry_msgs::Pose obj_ee_final,obj_ee_wp,ee_obj;
  tf::poseKDLToMsg(obj_ee_frame,obj_ee_final);
  tf::poseKDLToMsg(obj_ee_frame.Inverse(),ee_obj);
  
  // add a higher waypoint on the table
  obj_ee_wp = obj_ee_final;
  obj_ee_wp.position.z += WAYPOINT_HEIGHT;
  
  // the frame where the object position is considered (only when inserted)
  attached_object.object.header.frame_id = attached_object.link_name;
  attached_object.object.mesh_poses.clear();
  attached_object.object.mesh_poses.push_back(ee_obj);
  
  // add the waypoint and the final pose as end-effector poses
  ee_pose.clear();
  ee_pose.push_back(obj_ee_wp);
  ee_pose.push_back(obj_ee_final);
}

int main()
{
  std::string db_name = "test.db";
  databaseMapper db_mapper(db_name);
  databaseWriter db_writer(db_name);
  
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  
  std::string obj_name;
  std::string grasp_name;
  KDL::Frame obj_ee_final;
  
  // read grasp data from file
  if(!read_data_from_file(obj_name, grasp_name, obj_ee_final))
  {
    ROS_ERROR_STREAM("Unable to read grasp file - returning");
    return -1;
  }
  
  int obj_id = -1;
  
  for(auto& obj:db_mapper.Objects)
    if(std::get<0>(obj.second) == obj_name)
    {
      obj_id = obj.first;
      break;
    }
  
  if(obj_id == -1)
  {
    ROS_ERROR_STREAM("Object " << obj_name << " is not present in the DB - returning");
    return -1;
  }
  
  // for the desired amount of steps, rotate around global z axis
  for(int i=0; i<YAW_STEPS; i++)
  {
    KDL::Frame rotz(KDL::Rotation::RotZ(M_PI*2*i/YAW_STEPS));
    KDL::Frame obj_ee_rotated(obj_ee_final);
    
    std::string grasp_name_rotated(grasp_name);
    grasp_name_rotated += "_" + std::to_string(180*i/YAW_STEPS);
    
    obj_ee_rotated.M = rotz.M*obj_ee_rotated.M;
    
    // build grasp msg for this orientation
    build_grasp_msg(grasp_msg, obj_ee_rotated, obj_name, obj_id, END_EFFECTOR_FRAME);

    // write its entry in the database
    int grasp_id = db_writer.writeNewGrasp(obj_id,END_EFFECTOR_ID,grasp_name_rotated);
    if(grasp_id == -1)
    {
      ROS_ERROR_STREAM("Unable to write entry in the grasp DB - returning");
      return -1;
    }

    // serialize it (using the id just obtained)
    if(!serialize_data(grasp_msg, obj_id, grasp_id))
    {
      ROS_ERROR_STREAM("Unable to serialize grasp message - returning...");
      if(!db_writer.deleteGrasp(grasp_id))
      {
	ROS_ERROR_STREAM("Unable to delete entry from DB: consider deleting it by hand!");
      }
      return -1;
    }
  }
  
  ROS_INFO_STREAM("Everything done! Change grasp file and restart!");
  return 0;
}
