#include "../include/dual_manipulation_shared/serialization_utils.h"

#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/ik_service_legacy.h"
#include "dual_manipulation_shared/grasp_trajectory.h"
#include <ros/package.h>

#include <ros/serialization.h>
#include <geometry_msgs/Pose.h>
#include <fstream>

template< typename T> bool serialize_ik(const T & my_ik, std::string filename)
{
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    path.append("/serialized_grasps/").append(filename);
    
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

template<>
bool serialize_ik<dual_manipulation_shared::ik_service::Request>(const dual_manipulation_shared::ik_service::Request & srv, std::string filename)
{
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  
  grasp_msg.attObject = srv.attObject;
  grasp_msg.ee_pose = srv.ee_pose;
  grasp_msg.grasp_trajectory = srv.grasp_trajectory;
  grasp_msg.object_db_id = srv.object_db_id;
  
  return serialize_ik(grasp_msg,filename);
}

template bool serialize_ik<dual_manipulation_shared::ik_service_legacy::Request>(const dual_manipulation_shared::ik_service_legacy::Request &, std::string);
template bool serialize_ik<dual_manipulation_shared::grasp_trajectory>(const dual_manipulation_shared::grasp_trajectory &, std::string);

template< typename T> bool deserialize_ik(T & my_ik, std::string filename)
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

template<>
bool deserialize_ik<dual_manipulation_shared::ik_service::Request>(dual_manipulation_shared::ik_service::Request & srv, std::string filename)
{
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  if(!deserialize_ik(grasp_msg,filename))
    return false;
  
  srv.attObject = grasp_msg.attObject;
  srv.ee_pose = grasp_msg.ee_pose;
  srv.grasp_trajectory = grasp_msg.grasp_trajectory;
  srv.object_db_id = grasp_msg.object_db_id;
  
  return true;
}

template bool deserialize_ik<dual_manipulation_shared::ik_service_legacy::Request>(dual_manipulation_shared::ik_service_legacy::Request &, std::string);
template bool deserialize_ik<dual_manipulation_shared::grasp_trajectory>(dual_manipulation_shared::grasp_trajectory &, std::string);

geometry_msgs::Point difference(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    geometry_msgs::Point result;
    result.x=p1.x-p2.x;
    result.y=p1.y-p2.y;
    result.z=p1.z-p2.z;
    return result;
}

double norm2(geometry_msgs::Point p)
{
    return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

void monotonic_decreasing_distance_filter(std::vector<geometry_msgs::Pose>& poses, geometry_msgs::Pose ref_pose=geometry_msgs::Pose())
{
    std::vector<geometry_msgs::Pose> new_poses;
    double min_norm = 9999;
    double norm;

    for(int i=0;i<poses.size();i++)
    {
	norm = norm2(difference(poses.at(i).position,ref_pose.position));
	if(norm<min_norm)
	{
	    new_poses.push_back(poses.at(i));
	    min_norm = norm;
	}
    }
    
    poses.swap(new_poses);
}

void down_sampling(std::vector<geometry_msgs::Pose>& poses, int period)
{
    std::vector<geometry_msgs::Pose> new_poses;
    
    new_poses.push_back(poses.front());

    for(int i=1;i<poses.size()-1;i++)
    {
        if(i % period == 0)
	{
	    new_poses.push_back(poses.at(i));
	}
    }
    
    new_poses.push_back(poses.back());
    
    poses.swap(new_poses);
}