#ifndef DATABASEWRITER_H
#define DATABASEWRITER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include "ros/package.h"
#include "dual_manipulation_shared/databasemapper.h"
#include <kdl/frames.hpp>

class databaseWriter
{
public:
  
  /**
    * @brief constructor
    * 
    * @param db_name name of the database to use for writing
    */
  databaseWriter(std::string db_name = "test.db");

  int writeNewWorkspace(int workspace_id, std::string workspace_name);
  int writeNewAdjacency(int workspace_id_s, int workspace_id_t);
  int writeNewReachability(int end_effector_id, int workspace_id);
  int writeNewEndEffectors(int end_effector_id, std::string name, bool movable);
  int writeNewGeometry(int workspace_id, std::string geometry_string);

  /**
    * @brief Insert a new object in the database
    * 
    * @param obj_name the name of the object
    * @param mesh_path the relative path of the object mesh (only the mesh name stored in the object_meshes subfolder)
    * @param obj_center transformation between the object reference frame and the object center
    * 
    * @return the newly inserted object ID (-1 on failure)
    */
  int writeNewObject(std::string obj_name, std::string mesh_path, KDL::Frame obj_center = KDL::Frame::Identity());

  /**
    * @brief Insert a new object in the database, specifying also the object_id (Attention! This will fail if the value already exists!)
    * 
    * @param object_id the object id
    * @param obj_name the name of the object
    * @param mesh_path the relative path of the object mesh (only the mesh name stored in the object_meshes subfolder)
    * @param obj_center transformation between the object reference frame and the object center
    * 
    * @return the newly inserted object ID (-1 on failure)
    */
  int writeNewObject(int object_id, std::string obj_name, std::string mesh_path, KDL::Frame obj_center = KDL::Frame::Identity());

  /**
    * @brief Insert a new grasp in the database
    * 
    * @param object_id the object id
    * @param end_effector_id the end-effector id
    * @param grasp_name the name to associate to the grasp
    * 
    * @return the newly inserted grasp ID (-1 on failure)
    */
  int writeNewGrasp(int object_id, int end_effector_id, std::string grasp_name);

  /**
    * @brief Insert a new grasp in the database, specifying also the grasp_id (Attention! This will fail if the value already exists!)
    * 
    * @param grasp_id the grasp id
    * @param object_id the object id
    * @param end_effector_id the end-effector id
    * @param grasp_name the name to associate to the grasp
    * 
    * @return the newly inserted grasp ID (-1 on failure)
    */
  int writeNewGrasp(int grasp_id, int object_id, int end_effector_id, std::string grasp_name);

  /**
    * @brief Insert a new grasp transition in the database
    * 
    * @param source_grasp_id source grasp id
    * @param target_grasp_id target grasp id
    * 
    * @return the newly inserted grasp transition ID (-1 on failure)
    */
  int writeNewTransition(int source_grasp_id, int target_grasp_id, bool just_dont=false);

  /**
    * @brief Delete a grasp from the database
    * 
    * @param grasp_id the id of the grasp to be deleted
    * 
    * @return true on success
    */
  bool deleteGrasp(int grasp_id);

  /**
    * @brief Delete an object from the database
    * 
    * @param obj_id the id of the object to be deleted
    * 
    * @return true on success
    */
  bool deleteObject(int obj_id);

  /**
    * @brief Delete a grasp transition from the database
    * 
    * @param source_grasp_id source grasp id
    * @param target_grasp_id target grasp id
    * 
    * @return true on success
    */
  bool deleteGraspTransition(int source_grasp_id, int target_grasp_id);
  
  ~databaseWriter();
  bool open_global();
  bool close_global();
  
private:
  std::string path_to_db_,db_name_;
  databaseMapper* db_mapper_;
  std::map<int,std::string> object_name_map_;
  std::map<int,std::string> ee_name_map_;
  std::map<int,std::string> grasp_name_map_;
  std::set<std::pair<int,int>> transitions_set_;
  std::map<int,std::string> workspace_name_map_;
  std::map<int,std::set<int>> adjacency_map_, reachability_map_;
  sqlite3* global_db;


  int insert_db_entry(const std::string& sqlstatement, bool remove=false, bool just_dont=false);

  void bind_value_unwrap(sqlite3_stmt *stmt, int index, int value)
  {
      sqlite3_bind_int(stmt,index,value);
  }
  void bind_value_unwrap(sqlite3_stmt *stmt, int index, std::string value)
  {
      sqlite3_bind_text(stmt,index,value.c_str(),value.size(),SQLITE_TRANSIENT);
  }
  void bind_value(sqlite3_stmt *stmt, int ){};
  template<class T, class ...Args>
  void bind_value(sqlite3_stmt *stmt, int index,T t, Args... args)
  {
      bind_value_unwrap(stmt,index,t);
      bind_value(stmt,++index,args...);
  }

  template <typename ... Args>
  int writeNewSomething(const std::string& sqlstatement, Args... args)
  {
      sqlite3 *db;
      sqlite3_stmt * stmt;
      int scoreID = -1;

      if (sqlite3_open((path_to_db_+"/"+db_name_).c_str(), &db) == SQLITE_OK)
      {
          sqlite3_prepare( db, sqlstatement.c_str(), -1, &stmt, NULL );//preparing the statement
          if (!sqlite3_bind_parameter_count(stmt) == sizeof...(args)) abort();

          bind_value(stmt,1,args...);

          if(sqlite3_step(stmt) == SQLITE_DONE)
          {
              if(!remove)
                  scoreID = sqlite3_last_insert_rowid(db);
              else
                  scoreID = 1;
          }
          else
          {
              ROS_ERROR("databaseWriter::insert_db_entry : Failed to store in cache with error message: %s",sqlite3_errmsg(db));
          }
      }
      else
      {
          ROS_ERROR("databaseWriter::insert_db_entry : Failed to open db");
      }

      sqlite3_finalize(stmt);
      sqlite3_close(db);

      return scoreID;
  }
};

#endif //DATABASEWRITER_H