#include "grasp_storage.h"
#include <sqlite3.h>
#include "dual_manipulation_shared/ik_service.h"
#include "dual_manipulation_shared/serialization_utils.h"

std::string make_it_red(std::string input)
{
    return "\033[31m"+input+"\033[0m";
}

std::string str_quotesql( const std::string& s ) {
    return std::string("'") + s + std::string("'");
}

std::string int_quotesql( const int& s ) {
    return std::string("'") + std::to_string(s) + std::string("'");
}

grasp_storage::grasp_storage(int object_id_, int end_effector_id_, std::string grasp_name_):object_id(object_id_),end_effector_id(end_effector_id_), grasp_name(grasp_name_)
{
    path_to_db = ros::package::getPath("dual_manipulation_grasp_db");
    error =  make_it_red("ERROR");
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
            std::cout << "Last row score : "<<scoreID<<std::endl;
        }
        else 
	{
	    std::cout<<error<<": Failed to store score in cache wihth error message: "<<sqlite3_errmsg(db)<<std::endl;
        }
    }
    else
    {
        std::cout<<error<<": Failed to open db"<<std::endl;
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
}

void grasp_storage::save_start_pose()
{
    std::cout<<"- saving start position..."<<std::endl;

    
}

void grasp_storage::record_trajectory_pose()
{
    std::cout<<"- start recording trajectory..."<<std::endl;

    
}

void grasp_storage::stop_record_trajectory_pose()
{
    std::cout<<"- stop recording trajectory..."<<std::endl;

    
}

void grasp_storage::save_end_pose()
{
    std::cout<<"- saving end position..."<<std::endl;

    
}

void grasp_storage::serialize_data()
{
    std::cout<<"- serializing..."<<std::endl;

    if(scoreID<0)
    {
	std::cout<<error<<": row for insertion not valid - ABORT serialization"<<std::endl;
	return;
    }
    
    dual_manipulation_shared::ik_service srv;

    srv.request.command="";
    trajectory_msgs::JointTrajectoryPoint asd;
    srv.request.grasp_trajectory.points.push_back(asd);
    srv.request.ee_pose.clear();
    geometry_msgs::Pose ee_pose;
    ee_pose.orientation.w=1;
    ee_pose.position.x=7;
    srv.request.ee_pose.push_back(ee_pose);
    
    // save the obtained grasp
    if(serialize_ik(srv.request,"object" + std::to_string(object_id) + "/grasp" + std::to_string(scoreID)))
    {
	std::cout<<"Serialization object" + std::to_string(object_id) << "/grasp" << scoreID << " OK!" << std::endl;
    }
    else
    {
	std::cout<<error<<": in serialization object" + std::to_string(object_id) << "/grasp" << scoreID << "!" << std::endl;
	return;
    }
}

void grasp_storage::save_in_db()
{
    std::cout<<"- saving in db..."<<std::endl;

    insert_db_entry();
}

grasp_storage::~grasp_storage()
{
  
}