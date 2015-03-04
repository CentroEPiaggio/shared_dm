#ifndef GRASP_STORAGE_H
#define GRASP_STORAGE_H

#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>
#include <thread>
#include "ros/package.h"
#include "dual_manipulation_shared/databasemapper.h"

class grasp_storage
{
public:
    grasp_storage(int object_id_, int end_effector_id_, std::string grasp_name_);

    void save_start_pose();
    void record_trajectory_pose();
    void stop_record_trajectory_pose();
    void save_end_pose();

    void serialize_data();
    void save_in_db();

    ~grasp_storage();
private:
    int object_id;
    int end_effector_id;
    std::string grasp_name;

    std::string path_to_db;
    ros::NodeHandle node;
    tf::TransformListener tf;

    void insert_db_entry();
    int scoreID=-1;

    databaseMapper db_mapper;
    std::map<int,std::string> object_name_map;
    std::map<int,std::string> ee_name_map;
    geometry_msgs::Pose obj0_hand;
    std::vector<geometry_msgs::Pose> obj0_trajectory;
    geometry_msgs::Pose hand_objF;
    
    std::string world_tf, hand_tf, object_tf;
    tf::StampedTransform get_transform(std::string target, std::string source);
    void thread_body();
    std::thread* tf_periodic_listener;
    bool stop_thread=false;
};

#endif //GRASP_STORAGE_H