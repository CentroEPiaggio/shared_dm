#include "grasp_storage.h"
#include "ros/ros.h"

void thread_body()
{
    while(1)
    {
	ros::spinOnce(); 
	usleep(20000);
    }
}

int main(int argc, char** argv)
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
	ROS_INFO("Press 's' key ('q' to exit) to snapshot: ");
	std::cin>>a;
	if(a=='q') end=true;
	if(a=='s') grasp_stor.single_step(true);
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