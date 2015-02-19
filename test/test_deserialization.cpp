#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include "dual_manipulation_shared/serialization_utils.h"

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <visualization_msgs/Marker.h>

void publish_marker_utility(ros::Publisher& vis_pub, geometry_msgs::Pose& pose, double scale=0.05)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
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
    std::cout<<"|Dual manipulation| -> test_deserialization "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_deserialization");

    ros::NodeHandle n;
    ros::ServiceClient client_obj = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    
    dual_manipulation_shared::ik_service srv;
    dual_manipulation_shared::scene_object_service srv_obj;
    
    std::vector<std::string> grasp_links;
    std::vector<geometry_msgs::Pose> grasp_poses;
    
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
	
	srv_obj.request.attObject = srv.request.attObject;
	
	if (client_obj.call(srv_obj))
	{
	    ROS_INFO("IK_control:test_grasping : %s object %s request accepted: %d", srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str(), (int)srv_obj.response.ack);
	}
	else
	{
	    ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service: %s %s",srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str());
	}
	
	grasp_links.push_back(srv.request.attObject.object.header.frame_id);
	grasp_poses.push_back(srv.request.ee_pose.back());
	
	// usleep(200000);
      }
      else
      {
	std::cout << "Stopped at deserialization object" + std::to_string(object_id) << "/grasp" << i-1 << std::endl;
      }
    }
    
    srv_obj.request.attObject.object.mesh_poses.clear();
    geometry_msgs::Pose neutral,marker_pose;
    neutral.position.x = -0.5;
    neutral.position.y = 0.0;
    neutral.position.z = 0.5;
    neutral.orientation.w = 1.0;
    srv_obj.request.attObject.object.mesh_poses.push_back(neutral);
    
    client_obj.call(srv_obj);
    
    KDL::Frame cyl_kdl;
    tf::poseMsgToKDL(neutral,cyl_kdl);
    
    for (int i=0; i<grasp_poses.size(); ++i)
    {
      KDL::Frame fi;
      tf::poseMsgToKDL(grasp_poses.at(i),fi);
      
      tf::poseKDLToMsg(cyl_kdl*(fi),marker_pose);
      
      publish_marker_utility(vis_pub,marker_pose);
      
      if (grasp_links.at(i) != "world")
	usleep(200000);
      else
	usleep(200000);
	// sleep(2);
    }
    
    ros::spinOnce();
    return 0;
}