#include "grasp_storage.h"
#include "ros/ros.h"

#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

void thread_body()
{
    while(1)
    {
	ros::spinOnce(); 
	usleep(20000);
    }
}

int main()
{
  dual_manipulation_shared::ik_serviceRequest req;
  
  deserialize_ik(req,"object3/grasp37");
  
  geometry_msgs::Pose obj_pose;
  KDL::Frame obj_frame;
  
//   // invert attached object frame
//   obj_pose = req.ee_pose.back();
//   req.ee_pose.pop_back();
//   
//   tf::poseMsgToKDL(obj_pose,obj_frame);
//   tf::poseKDLToMsg(obj_frame.Inverse(),obj_pose);
//   
//   req.attObject.object.mesh_poses.clear();
//   req.attObject.object.mesh_poses.push_back(obj_pose);
  
  // add a pre-grasp pose
//   obj_pose = req.ee_pose.front();
//   obj_pose.position.x = obj_pose.position.x * 1.5;
//   obj_pose.position.y = obj_pose.position.y * 1.5;
//   obj_pose.position.z = obj_pose.position.z * 1.5;
//   req.ee_pose.insert(req.ee_pose.begin(),obj_pose);
  
//   serialize_ik(req,"object3/grasp37");
  
//   deserialize_ik(req,"object3/grasp38_old");
//   // add a pre-grasp pose
//   obj_pose = req.ee_pose.front();
//   obj_pose.position.x = obj_pose.position.x * 1.5;
//   obj_pose.position.y = obj_pose.position.y * 1.5;
//   obj_pose.position.z = obj_pose.position.z * 1.5;
//   req.ee_pose.insert(req.ee_pose.begin(),obj_pose);

  KDL::Frame rotx(KDL::Rotation::RotX(M_PI));
  KDL::Frame obj_hand;
  
  for(auto& pose:req.ee_pose)
  {
    tf::poseMsgToKDL(pose,obj_hand);
    tf::poseKDLToMsg(rotx*obj_hand,pose);
  }
  
  tf::poseMsgToKDL(req.attObject.object.mesh_poses.at(0),obj_frame);
  obj_hand = obj_frame.Inverse();
  obj_frame = (rotx*obj_hand).Inverse();
  tf::poseKDLToMsg(obj_frame,req.attObject.object.mesh_poses.at(0));
  
  serialize_ik(req,"object3/grasp38");
  
  return 0;
}

int main2(int argc, char** argv)
{
    if(argc!=4)
    {
	std::cout<<std::endl;
	std::cout<<" - Usage:   ./grasp_storage    object_id    end_effector_id    grasp_name"<<std::endl<<std::endl;
	std::cout<<"\t - object_id: identifies the object type (1=FakeCylinder, 2=Pot, 3=Cylinder) "<<std::endl;
	std::cout<<"\t - end_effector_id: identifies the e-e (1=LH, 2=RH, 3=Table)"<<std::endl;
	std::cout<<"\t - grasp_name: custom name for grasp (e.g. \"RH_top\")"<<std::endl<<std::endl;
      
	return -1;
    }

    int object_id, end_effector_id;
    std::string grasp_name;
    
    object_id = atoi(argv[1]);
    end_effector_id = atoi(argv[2]);
    grasp_name = argv[3];
    
    std::cout<<std::endl;
    std::cout<<" - Ready to store grasp for:"<<std::endl<<std::endl;
    std::cout<<"\t - object_id = "<<object_id<<std::endl;
    std::cout<<"\t - end_effector_id = "<<end_effector_id<<std::endl;
    std::cout<<"\t - grasp_name = "<<grasp_name<<std::endl<<std::endl;
    std::cout<<"No data will be saved on disk until the \"!! ATTENTION !!\" messages"<<std::endl<<std::endl;
  
    ros::init(argc, argv, "grasp_storage");
    
    std::thread th(&thread_body);
    
    grasp_storage grasp_stor(object_id, end_effector_id, grasp_name);
    
    char a;

    ROS_INFO("Press any key ('q' to exit, 's' to snapshot) to START recording TRAJECTORY: ");
//     ROS_INFO("Press any key ('q' to exit) to save the START POSITION: ");
    std::cin>>a;
    if(a=='q') return 0;
    if (a=='s')
    {
      bool end=false;
      while (!end)
      {
	ROS_INFO("Press 's' key ('f' for final pose, 'q' to exit) to snapshot: ");
	std::cin>>a;
	if(a=='q') end=true;
	if(a=='s') grasp_stor.single_step(true);
	if(a=='f') grasp_stor.save_end_pose();
      }
    }
    else
    {
    grasp_stor.save_start_pose();
    
//     ROS_INFO("Press any key ('q' to exit) to START recording TRAJECTORY: ");
//     std::cin>>a;
//     if(a=='q') return 0;
    grasp_stor.record_trajectory_pose();
    
    ROS_INFO("Press any key ('q' to exit) to STOP recording TRAJECTORY: ");
    std::cin>>a;
    if(a=='q') return 0;
    grasp_stor.stop_record_trajectory_pose();
    }
//     usleep(200000);
    
//     ROS_INFO("Press any key ('q' to exit) to save the END POSITION: ");
//     std::cin>>a;
//     if(a=='q') return 0;
//     grasp_stor.save_end_pose();
    
    ROS_INFO("!! ATTENTION !! Press any key ('q' to exit) to insert row in the database ('n' to abort) : ");
    std::cin>>a;
    if(a=='q') return 0;
    if(a!='n') grasp_stor.save_in_db();
    else ROS_WARN("ABORT writing in database");
    
    ROS_INFO("!! ATTENTION !! Press any key ('q' to exit) to serialize data ('n' to abort) : ");
    std::cin>>a;
    if(a=='q') return 0;
    if(a!='n') grasp_stor.serialize_data();
    else ROS_WARN("ABORT serialization");
    
    std::cout<<std::endl<<">>>>> Press any key to close <<<<<<"<<std::endl<<std::endl;
    std::cin>>a;
    
    return 0;
}