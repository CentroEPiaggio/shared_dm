#include "grasp_storage.h"
#include <sqlite3.h>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"

std::string str_quotesql( const std::string& s ) {
    return std::string("'") + s + std::string("'");
}

std::string int_quotesql( const int& s ) {
    return std::string("'") + std::to_string(s) + std::string("'");
}

tf::StampedTransform grasp_storage::get_transform(std::string target, std::string source)
{
    std::string err_msg;

    tf::StampedTransform target_T_source;

    if(!tf.waitForTransform(target,source,ros::Time::now(), ros::Duration(1.0), ros::Duration(0.01), &err_msg))
    {
	ROS_ERROR("Error in tf: %s",err_msg.c_str());
	target_T_source.setIdentity();
    }
    else tf.lookupTransform(target,source, ros::Time(0), target_T_source);

    return target_T_source;
}

grasp_storage::grasp_storage(int object_id_, int end_effector_id_, std::string grasp_name_):object_id(object_id_),end_effector_id(end_effector_id_), grasp_name(grasp_name_)
{
    path_to_db = ros::package::getPath("dual_manipulation_grasp_db");
    
    object_name_map[1]="FakeCylinder";
    object_name_map[2]="Pot";
    object_name_map[3]="Cylinder";
    
    ee_name_map[1]="left_hand";
    ee_name_map[2]="right_hand";
    ee_name_map[3]="table";
    
    world_tf = "/phase_space_world";
    hand_tf ="/hand";
    object_tf = "/cylinder";
}

void grasp_storage::insert_db_entry()
{
    sqlite3 *db;
    sqlite3_stmt * stmt;

    std::string empty="";

    //INSERT INTO Grasps (Object_id, EndEffector_id, Grasp_id, Grasp_info, Grasp_name) VALUES ('3','1','','gatto')

    std::string sqlstatement =
    "INSERT INTO Grasps (Object_id, EndEffector_id, Grasp_info, Grasp_name) VALUES ("
    + int_quotesql(object_id) + ","
    + int_quotesql(end_effector_id) + ","
    + str_quotesql(empty) + ","
    + str_quotesql(grasp_name) + ");";

    if (sqlite3_open((path_to_db+"/test.db").c_str(), &db) == SQLITE_OK)
    {
	sqlite3_prepare( db, sqlstatement.c_str(), -1, &stmt, NULL );//preparing the statement

	if(sqlite3_step(stmt) == SQLITE_DONE)
	{
            scoreID = sqlite3_last_insert_rowid(db);
            ROS_INFO_STREAM("Last row score : "<<scoreID);
        }
        else 
	{
	    ROS_ERROR("Failed to store score in cache wihth error message: %s",sqlite3_errmsg(db));
        }
    }
    else
    {
        ROS_ERROR("Failed to open db");
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
}

void grasp_storage::save_start_pose()
{
    ROS_INFO_STREAM("saving start object position...");
    
    world_T_obj0 = get_transform(world_tf,object_tf);

    ROS_INFO_STREAM("saved!");
}

void grasp_storage::thread_body()
{
    geometry_msgs::Pose obj0_traj;
    tf::StampedTransform world_T_traj;

    while(!stop_thread)
    {
	world_T_traj = get_transform(world_tf,hand_tf);

	tf::poseTFToMsg(world_T_obj0.inverse()*world_T_traj,obj0_traj);

	obj0_trajectory.push_back(obj0_traj);

        usleep(100000);
    }
    ROS_INFO_STREAM("stop recording trajectory");
}

void grasp_storage::record_trajectory_pose()
{
    ROS_INFO_STREAM("start recording trajectory");

    tf_periodic_listener = new std::thread(&grasp_storage::thread_body, this);
}

void grasp_storage::stop_record_trajectory_pose()
{
    stop_thread=true;
}

void grasp_storage::save_end_pose()
{
    ROS_INFO_STREAM("saving end position...");

    tf::StampedTransform hand_T_objF = get_transform(hand_tf,object_tf);

    tf::poseTFToMsg(hand_T_objF,hand_objF);

    ROS_INFO_STREAM("saved!");
}

void grasp_storage::serialize_data()
{
    ROS_INFO_STREAM("serializing...");

    if(scoreID<0)
    {
	ROS_ERROR("Row for insertion not valid - ABORT serialization");
	return;
    }
    
    dual_manipulation_shared::ik_service srv;
    
    srv.request.command = "grasp";
    
    // create an object for grasping
    moveit_msgs::AttachedCollisionObject& attached_object = srv.request.attObject;
    trajectory_msgs::JointTrajectory& grasp_trajectory = srv.request.grasp_trajectory;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    
    // hand: only open to closed
    traj_point.positions.push_back(0.0);
    grasp_trajectory.points.push_back(traj_point);
    traj_point.positions.clear();
    traj_point.positions.push_back(1.0);
    grasp_trajectory.points.push_back(traj_point);
    
    attached_object.object.id = object_name_map.at(object_id);
    // this will be interpreted as the object ID (to read in the DB)
    attached_object.weight = 1.0;

    // grasp counter!
    int counter = 0;
    
    std::string ee = ee_name_map.at(end_effector_id);
    
    if(ee == "left_hand" || ee == "right_hand")
    {
      attached_object.link_name = ee + "_palm_link";
      grasp_trajectory.joint_names.clear();
      grasp_trajectory.joint_names.push_back(ee + "_synergy_joint");
    }
    else
    {
      attached_object.link_name = "world";
      grasp_trajectory.joint_names.clear();
    }
      
    // the frame where the object position is considered (only when inserted)
    attached_object.object.header.frame_id = attached_object.link_name; //"world";
    attached_object.object.mesh_poses.clear();
    attached_object.object.mesh_poses.push_back(hand_objF);

    srv.request.ee_pose.clear();
    
    for (auto item:obj0_trajectory) srv.request.ee_pose.push_back(item);
    
    // save the obtained grasp
    if(serialize_ik(srv.request,"object" + std::to_string(object_id) + "/grasp" + std::to_string(scoreID)))
    {
	ROS_INFO_STREAM("Serialization object" + std::to_string(object_id) << "/grasp" << scoreID << " OK!");
    }
    else
    {
        std::string temp = std::to_string(object_id);
	ROS_ERROR("In serialization object%s/grasp%d ! ", temp.c_str(), scoreID);
	return;
    }
    
    ROS_INFO_STREAM("Serialized");
}

void grasp_storage::save_in_db()
{
    ROS_INFO_STREAM("saving in db...");

    insert_db_entry();
}

grasp_storage::~grasp_storage()
{
    delete tf_periodic_listener;
}