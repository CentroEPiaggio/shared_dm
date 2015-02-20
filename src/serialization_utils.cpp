#include "../include/dual_manipulation_shared/serialization_utils.h"

#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <ros/package.h>

#include <ros/serialization.h>
#include <fstream>

bool serialize_ik(const dual_manipulation_shared::ik_service::Request & my_ik, std::string filename="unnamed_grasp.txt"){
    
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    path.append("/grasp_trajectories/").append(filename);
    
    uint32_t serial_size = ros::serialization::serializationLength(my_ik);
    
    if (serial_size > max_grasp_size)
    {
        ROS_ERROR("Maximum grasp message size (%d) exeeded: current message size = %d",max_grasp_size,serial_size);
        return false;
    }
    
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, my_ik);
    
    std::fstream fs;
    // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app*/);
    fs.open (path, std::fstream::out);
    if (fs.fail())
        return false;
    
    // fs << my_ik; // this unfortunately doesn't work when reading back!
    for (int i=0; i<serial_size; ++i)
        fs << buffer[i];
    
    fs.close();
    
    return true;
    
}

bool deserialize_ik(dual_manipulation_shared::ik_service::Request & my_ik, std::string filename="unnamed_grasp.txt")
{
    
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    path.append("/grasp_trajectories/").append(filename);
    
    std::fstream fs;
    // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app);
    fs.open (path, std::fstream::in);
    if (fs.fail())
        return false;
    
    // create a temporary char array of maximum allowed length
    char * tmp;
    tmp = new char[max_grasp_size];
    fs.read(tmp,std::streamsize(max_grasp_size));
    
    // std::cout << "tmp:" << std::endl;
    // for (int i=0; i<fs.gcount(); ++i)
    // {
    //   std::cout << tmp[i];
    // }
    // std::cout << std::endl;
    
    ros::serialization::IStream stream((uint8_t*)tmp, fs.gcount()*sizeof(char)/sizeof(uint8_t));
    ros::serialization::deserialize(stream, my_ik);
    
    // std::cout << "my_ik:" << std::endl << my_ik << std::endl;
    
    fs.close();
    delete tmp;
    
    return true;
}
