#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include "dual_manipulation_shared/serialization_utils.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <visualization_msgs/Marker.h>

#define OBJ_ID 3

void transform_specular_y(geometry_msgs::Pose& pose)
{
    KDL::Frame pose_tmp;
    tf::poseMsgToKDL(pose,pose_tmp);
    pose_tmp.p.data[1] = -pose_tmp.p.data[1];
    pose_tmp.M.data[1] = -pose_tmp.M.data[1];
    pose_tmp.M.data[3] = -pose_tmp.M.data[3];
    pose_tmp.M.data[5] = -pose_tmp.M.data[5];
    pose_tmp.M.data[7] = -pose_tmp.M.data[7];
    tf::poseKDLToMsg(pose_tmp,pose);
}

void transform_specular_y(std::vector<geometry_msgs::Pose>& poses)
{
  for(int i=0; i<poses.size(); i++)
  {
    transform_specular_y(poses.at(i));
  }
}

void transform_premultiply(std::vector<geometry_msgs::Pose>& poses, KDL::Frame frame)
{
  KDL::Frame pose_tmp;
  for(int i=0; i<poses.size(); i++)
  {
    tf::poseMsgToKDL(poses.at(i),pose_tmp);
    tf::poseKDLToMsg(frame*pose_tmp,poses.at(i));
  }
}

bool reserialize_grasp(dual_manipulation_shared::ik_serviceRequest& req, int left_grasp_id)
{
    
    // 16 is a RH bottom grasp : generate the corresponding top grasp with symmetry on x|y plane
    transform_specular_y(req.ee_pose);
    // transform top-bottom and side_low to side_high
    transform_premultiply(req.ee_pose,KDL::Frame(KDL::Rotation::RotX(M_PI)));
    
    KDL::Frame obj_frame;
    geometry_msgs::Pose obj_pose;
    tf::poseMsgToKDL(req.attObject.object.mesh_poses.front(),obj_frame);
    obj_frame = obj_frame.Inverse();
    tf::poseKDLToMsg(obj_frame,obj_pose);
    transform_specular_y(obj_pose);
    std::vector<geometry_msgs::Pose> vec_gianma = {obj_pose};
    transform_premultiply(vec_gianma,KDL::Frame(KDL::Rotation::RotX(M_PI)));
    obj_pose = vec_gianma.front();
    tf::poseMsgToKDL(obj_pose,obj_frame);
    obj_frame = obj_frame.Inverse();
    tf::poseKDLToMsg(obj_frame,obj_pose);
    req.attObject.object.mesh_poses.clear();
    req.attObject.object.mesh_poses.push_back(obj_pose);
    req.attObject.link_name = "left_hand_palm_link";
    req.attObject.object.header.frame_id = req.attObject.link_name;
    req.object_db_id = OBJ_ID;
    req.grasp_trajectory.joint_names.clear();
    req.grasp_trajectory.joint_names.push_back("left_hand_synergy_joint");
    
    if(serialize_ik(req,"object" + std::to_string(req.object_db_id) + "/grasp" + std::to_string(left_grasp_id)))
      std::cout << "Serialization object" + std::to_string(req.object_db_id) << "/grasp" << left_grasp_id << " OK!" << std::endl;
    else
    {
      std::cout << "Error in serialization object" + std::to_string(req.object_db_id) << "/grasp" << left_grasp_id << "!" << std::endl;
      return false;
    }
    return true;

}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> convert_RH_to_LH_grasps "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_deserialization");

//     ros::NodeHandle n;
//     ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    dual_manipulation_shared::ik_service srv;
    
    std::vector<std::string> grasp_links;
    std::vector<geometry_msgs::Pose> grasp_poses;
        
    bool ok = true;
    std::vector<int> right_grasp_ids = {37};
    std::vector<int> left_grasp_ids = {38};
    std::vector<std::string> g_names = {"LH_bottom"};
    
    for(int i=0; i<right_grasp_ids.size(); i++)
    {
      ok = deserialize_ik(srv.request,"object" + std::to_string(OBJ_ID) + "/grasp" + std::to_string(right_grasp_ids.at(i)));
      
      if (!reserialize_grasp(srv.request,left_grasp_ids.at(i)))
      {
	std::cout << "Error!" << std::endl;
	return -1;
      }
      else
      {
	std::cout << "ok " << g_names.at(i) << "!!!" << std::endl;
      }
    }
    
    return 0;
}