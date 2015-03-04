#include "grasp_storage.h"

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
	std::cout<<"\t - object_id: identifies the object type (1=Cylinder, 2=Pot, 3=StarCylinder) "<<std::endl;
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
    std::cout<<"No data will be saved on disk until the \"!! CONFIRM !!\" messages"<<grasp_name<<std::endl<<std::endl;
  
    ros::init(argc, argv, "grasp_storage");
    
    std::thread th(&thread_body);
    
    grasp_storage grasp_stor(object_id, end_effector_id, grasp_name);
    
    char a;

    std::cout<<"Press any key to save the START POSITION: "<<std::endl;
    std::cin>>a;
    grasp_stor.save_start_pose();
    
    std::cout<<"Press any key to START recording TRAJECTORY: "<<std::endl;
    std::cin>>a;
    grasp_stor.record_trajectory_pose();
    
    std::cout<<"Press any key to STOP recording TRAJECTORY: "<<std::endl;
    std::cin>>a;
    grasp_stor.stop_record_trajectory_pose();
    
    std::cout<<"Press any key to save the END POSITION: "<<std::endl;
    std::cin>>a;
    grasp_stor.save_end_pose();
    
    std::cout<<"!! CONFIRM !! Press any key to insert row in the database (\"n\" to abort) : "<<std::endl;
    std::cin>>a;
    if(a!='n') grasp_stor.save_in_db();
    else std::cout<<"- ABORT writing in database"<<std::endl;
    
    std::cout<<"!! CONFIRM !! Press any key to serialize data (\"n\" to abort) : "<<std::endl;
    std::cin>>a;
    if(a!='n') grasp_stor.serialize_data();
    else std::cout<<"- ABORT serialization"<<std::endl;
    
    std::cout<<std::endl<<">>>>> Press any key to close <<<<<<"<<std::endl<<std::endl;
    std::cin>>a;
    
    return 0;
}