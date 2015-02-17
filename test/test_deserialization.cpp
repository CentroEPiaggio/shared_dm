#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/scene_object_service.h"

#include "dual_manipulation_shared/serialization_utils.h"

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_deserialization "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "test_deserialization");

    ros::NodeHandle n;
    ros::ServiceClient client_obj = n.serviceClient<dual_manipulation_shared::scene_object_service>("scene_object_ros_service");
    
    dual_manipulation_shared::ik_service srv;
    dual_manipulation_shared::scene_object_service srv_obj;
    
    srv_obj.request.command = "add";
    
    bool ok = true;
    int i = 0;
    
    while (ok)
    {
      
      ok = deserialize_ik(srv.request,"grasp" + std::to_string(i++));
      
      if (ok)
	std::cout << "Deserialization " << i-1 << " OK!" << std::endl;
      else
      {
	std::cout << "Stopped at deserialization " << i-1 << " OK!" << std::endl;
	return -1;
      }
      
      srv_obj.request.attObject = srv.request.attObject;
      
      if (client_obj.call(srv_obj))
      {
	  ROS_INFO("IK_control:test_grasping : %s object %s request accepted: %d", srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str(), (int)srv_obj.response.ack);
      }
      else
      {
	  ROS_ERROR("IK_control:test_grasping : Failed to call service dual_manipulation_shared::scene_object_service: %s %s",srv_obj.request.command.c_str(),srv_obj.request.attObject.object.id.c_str());
      }
      
      usleep(200000);
      
    }
    
    ros::spinOnce();
    return 0;
}