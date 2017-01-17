/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Alessandro Settimi, Hamal Marino, Mirko Ferrati, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
  int writeNewWorkspace(int workspace_id, std::string workspace_name, std::vector<std::pair<double,double>> polygon, std::pair<double, double> height_min_max, KDL::Frame centroid);

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
    * @param ec_id the id of the currently active environment constraint
    * 
    * @return the newly inserted grasp ID (-1 on failure)
    */
  int writeNewGrasp(int object_id, int end_effector_id, std::string grasp_name, int ec_id = 0);

  /**
    * @brief Insert a new grasp in the database, specifying also the grasp_id (Attention! This will fail if the value already exists!)
    * 
    * @param grasp_id the grasp id
    * @param object_id the object id
    * @param end_effector_id the end-effector id
    * @param grasp_name the name to associate to the grasp
    * @param ec_id the id of the currently active environment constraint
    * 
    * @return the newly inserted grasp ID (-1 on failure)
    */
  int writeNewGrasp(int grasp_id, int object_id, int end_effector_id, std::string grasp_name, int ec_id = 0);
  
  /**
   * @brief Insert a new grasp transition in the database
   * This will use default values for:
   * - the transition cost (1)
   * - the transition type (GRASP, UNGRASP, EXCHANGE_GRASP depending on whether end-effectors are movable)
   * - extra involved end-effectors (none)
   * 
   * @param source_grasp_id source grasp id
   * @param target_grasp_id target grasp id
   * 
   * @return the newly inserted grasp transition ID (-1 on failure)
   */
  int writeNewTransition(int source_grasp_id, int target_grasp_id, bool just_dont=false);
  
  /**
   * @brief Insert a new grasp transition in the database
   * 
   * @param source_grasp_id source grasp id
   * @param target_grasp_id target grasp id
   * @param cost the transition cost
   * @param type the transition type
   * @param extra_ees a vector of other end-effectors which may be needed to perform the transition
   * 
   * @return the newly inserted grasp transition ID (-1 on failure)
   */
  int writeNewTransition(int source_grasp_id, int target_grasp_id, double cost, dual_manipulation::shared::NodeTransitionTypes type, std::vector<endeffector_id> extra_ees, bool just_dont=false);

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
  
  /**
   * @brief Insert a new Environment Constraint in the database
   * 
   * @param id the id of the constraint
   * @param name the name of the constraint
   * 
   * @return the newly inserted ec ID (-1 on failure)
   */
  int writeNewEnvironmentConstraintType(int id, std::string name);
  
  /**
   * @brief Insert a new EC adjacenty in the database
   * 
   * @param source_id the id of the source constraint
   * @param target_id the id of the target constraint
   * 
   * @return -1 on failure
   */
  
  int writeNewECAdjacency(int source_id, int target_id);
  
  /**
   * @brief Insert a new EC reachability in the database
   * 
   * @param ec_id the id of the constraint
   * @param workspace_id the id of the workspace
   * 
   * @return -1 on failure
   */
  int writeNewECReachability(int ec_id, int workspace_id);
  /**
   * @brief Insert a new Environment constraint in the database
   * 
   * @param constraint_id the id of the constraint
   * @param constraint_name The name of the constraint
   * @param type constraint type
   * @param pose The reference frame of the constraint.
   * @param min the minimum of the constraint on each axis
   * @param max the maximum of the constraint on each axis
   * 
   * @return -1 on failure
   */
  int writeNewEnvironmentConstraint (int constraint_id, std::string constraint_name, int type, KDL::Frame pose, KDL::Twist min, KDL::Twist max);
  
  ~databaseWriter();
  bool open_global();
  bool close_global();
  int checkGraspId(int grasp_id);
  
private:
  std::string path_to_db_,db_name_;
  databaseMapper* db_mapper_;
  std::map<int,std::string> object_name_map_;
  std::map<int,std::string> ee_name_map_; /// redundant on @p ee_map_, should be removed
  std::map<int,std::string> grasp_name_map_;
  std::map<int,int> grasp_ee_map_;
  std::map<int,endeffector_info> ee_map_;
  std::set<std::pair<int,int>> transitions_set_;
  std::map<int,std::string> workspace_name_map_;
  std::map<int,std::set<int>> adjacency_map_, reachability_map_;
  sqlite3* global_db;
  // maps for environment constraints
  std::map<int,std::string> ec_name_map_;
  std::map<int,std::string> ec_type_map_;
  std::map<int,std::set<int>> ec_adjacency_map_, ec_reachability_map_;
  
  int insert_db_entry(const std::string& sqlstatement, bool remove=false, bool just_dont=false);

  void bind_value_unwrap(sqlite3_stmt *stmt, int index, int value)
  {
      sqlite3_bind_int(stmt,index,value);
  }
  void bind_value_unwrap(sqlite3_stmt *stmt, int index, double value)
  {
      sqlite3_bind_double(stmt,index,value);
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