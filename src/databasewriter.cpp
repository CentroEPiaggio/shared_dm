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

#include "dual_manipulation_shared/databasewriter.h"
#include <sqlite3.h>

std::string str_quotesql( const std::string& s ) {
  return std::string("'") + s + std::string("'");
}

std::string int_quotesql( const int& s ) {
    return std::string("'") + std::to_string(s) + std::string("'");
}

std::string double_quotesql( const double& s ) {
    return std::string("'") + std::to_string(s) + std::string("'");
}

databaseWriter::databaseWriter(std::string db_name):db_name_(db_name)
{
  path_to_db_ = ros::package::getPath("dual_manipulation_grasp_db");
  
  db_mapper_ = new databaseMapper(db_name_);
  
  for(auto& obj:db_mapper_->Objects)
    object_name_map_[obj.first] = std::get<0>(obj.second);
  
  std::cout << "Object already in DB:\n";
  for(auto& obj:object_name_map_)
    std::cout << std::to_string(obj.first) << " -> " << obj.second << std::endl;
  std::cout << std::endl;
  
  for(auto& ee:db_mapper_->EndEffectors)
    ee_name_map_[ee.first] = std::get<0>(ee.second);

  std::cout << "End-effectors in DB:\n";
  for(auto& ee:ee_name_map_)
    std::cout << std::to_string(ee.first) << " -> " << ee.second << std::endl;
  std::cout << std::endl;
  
  for(auto& ec:db_mapper_->EnvironmentConstraints)
      ec_name_map_[ec.first] = ec.second;
  
  for(auto& ee:db_mapper_->Grasps)
      grasp_name_map_[ee.first] = std::get<2>(ee.second);
  
  for(auto& trans:db_mapper_->Grasp_transitions)
    for(auto target:trans.second)
      transitions_set_.insert(std::make_pair<int,int>(trans.first,target));
    
  std::cout << "Found " << grasp_name_map_.size() << " grasps already in the DB" << std::endl;
}

bool databaseWriter::open_global()
{
    sqlite3_open((path_to_db_+"/"+db_name_).c_str(), &global_db) == SQLITE_OK;
    sqlite3_exec(global_db, "BEGIN TRANSACTION;", NULL, NULL, NULL);
    return true;
}

bool databaseWriter::close_global()
{
    sqlite3_exec(global_db, "END TRANSACTION;", NULL, NULL, NULL);
    sqlite3_close(global_db);
    return true;
}

int databaseWriter::insert_db_entry(const std::string& sqlstatement, bool remove,bool just_dont)
{
  sqlite3 *db;
  sqlite3_stmt * stmt;
  int scoreID = -1;
  bool opened=true;
  if (!just_dont)
    opened = sqlite3_open((path_to_db_+"/"+db_name_).c_str(), &db) == SQLITE_OK;
  else
      db=global_db;
  if (opened)
  {
    sqlite3_prepare( db, sqlstatement.c_str(), -1, &stmt, NULL );//preparing the statement

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
  if (!just_dont) sqlite3_close(db);

  return scoreID;
}

databaseWriter::~databaseWriter()
{
  delete db_mapper_;
}

int databaseWriter::checkGraspId(int grasp_id)
{
    while(1)
    {
        if(grasp_name_map_.count(grasp_id)) grasp_id++;
        else break;
    }
    return grasp_id;
}

int databaseWriter::writeNewGrasp(int object_id, int end_effector_id, std::string grasp_name, int ec_id)
{
  int grasp_id = checkGraspId(1);
  return writeNewGrasp(grasp_id,object_id,end_effector_id,grasp_name,ec_id);
}

int databaseWriter::writeNewGrasp(int grasp_id, int object_id, int end_effector_id, std::string grasp_name, int ec_id)
{
  if(object_name_map_.count(object_id) == 0)
  {
    ROS_ERROR_STREAM("No object found in the DB with ID " << object_id);
    return -1;
  }
  if(ee_name_map_.count(end_effector_id) == 0)
  {
      ROS_ERROR_STREAM("No end-effector found in the DB with ID " << end_effector_id);
      return -1;
  }
  if(ec_name_map_.count(ec_id) == 0)
  {
      ROS_WARN_STREAM("No environment constraint found in the DB with ID " << ec_id << " - proceeding anyway, just to let you know..." );
  }
  
  const std::string& obj_name = object_name_map_.at(object_id);
  const std::string& ee_name = ee_name_map_.at(end_effector_id);
  std::string empty = "";

  //INSERT INTO Grasps (Object_id, EndEffector_id, Grasp_id, Grasp_info, Grasp_name) VALUES ('3','1','','gatto')

  std::string sqlstatement =
    "INSERT INTO Grasps (Object_id, EndEffector_id, Grasp_id, Grasp_info, Grasp_name, EC_id) VALUES ("
    + int_quotesql(object_id) + ","
    + int_quotesql(end_effector_id) + ","
    + int_quotesql(grasp_id) + ","
    + str_quotesql(empty) + ","
    + str_quotesql(grasp_name) + ","
    + int_quotesql(ec_id) + ");";

  int newID = insert_db_entry(sqlstatement);
  
  if(newID <= 0)
  {
    newID = -1;
  }
  else
  {
    ROS_INFO_STREAM("New grasp \"" << grasp_name << "\" for object " << obj_name << " with end-effector " << ee_name << " added with ID " << newID);
    grasp_name_map_[newID] = grasp_name;
  }

  return newID;
}

int databaseWriter::writeNewWorkspace(int workspace_id, std::string workspace_name)
{
    std::string sqlstatement =
    "INSERT INTO Workspaces VALUES (? , ?)";
    if (workspace_name_map_.count(workspace_id)) 
    {
        ROS_ERROR_STREAM("Error adding workspace \"" << workspace_name << "\" with ID " << workspace_id << "already in the database");
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, workspace_id, workspace_name);

    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error adding workspace \"" << workspace_name << "\" with ID " << newID);
    }
    else
    {
        ROS_INFO_STREAM("New workspace \"" << workspace_name << "\" added with ID " << newID);
        workspace_name_map_[workspace_id] = workspace_name;
    }
    return newID;
}

int databaseWriter::writeNewAdjacency(int workspace_id_s, int workspace_id_t)
{
    std::string sqlstatement =
    "INSERT INTO WorkspacesAdjacency VALUES (? , ?)";
    if (adjacency_map_.count(workspace_id_s) && adjacency_map_.at(workspace_id_s).count(workspace_id_t))
    {
        ROS_ERROR_STREAM("Error already exist Adjacency \"" << workspace_id_s << "\" to " << workspace_id_t);
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, workspace_id_s, workspace_id_t );
    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error writing new Adjacency \"" << workspace_id_s << "\" to " << workspace_id_t);
    }
    else
    {
        ROS_INFO_STREAM("New Adjacency \"" << workspace_id_s << "\" to " << workspace_id_t);
        adjacency_map_[workspace_id_s].insert(workspace_id_t);
    }
    return newID;
}

int databaseWriter::writeNewReachability(int end_effector_id, int workspace_id)
{
    std::string sqlstatement = "INSERT INTO Reachability VALUES (? , ?)";
    if (reachability_map_.count(end_effector_id) && reachability_map_.at(end_effector_id).count(workspace_id))
    {
        ROS_ERROR_STREAM("Error already exist Reachability with id \"" << end_effector_id<< "\" to workspace " << workspace_id);
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, end_effector_id, workspace_id);

    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error adding Reachability with id \"" << end_effector_id<< "\" to workspace " << workspace_id);
    }
    else
    {
        ROS_INFO_STREAM("New Reachability with id \"" << end_effector_id<< "\" to workspace " << workspace_id);
        reachability_map_[end_effector_id].insert(workspace_id);
    }
    return newID;
}

int databaseWriter::writeNewEndEffectors(int end_effector_id, std::string name, bool movable)
{
    std::string sqlstatement = "INSERT INTO EndEffectors VALUES (?, ?, ?)";
    if (ee_name_map_.count(end_effector_id)) 
    {
        ROS_ERROR_STREAM("Error already exists End Effector \"" << end_effector_id << "\" with name " << name << movable?" movable ":" NOT movable");
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, end_effector_id, name, (int)movable);
    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error adding New End Effector \"" << end_effector_id << "\" with name " << name << movable?" movable ":" NOT movable");
    }
    else
    {
        ROS_INFO_STREAM("New End Effector \"" << end_effector_id << "\" added with name " << name << movable?" movable ":" NOT movable");
        ee_name_map_[end_effector_id]=name;
    }
    return newID;
}

int databaseWriter::writeNewGeometry(int workspace_id, std::string geometry_string)
{
    std::string sqlstatement =
    "INSERT INTO WorkspaceGeometry VALUES (? , ?)";

    int newID = writeNewSomething(sqlstatement, workspace_id, geometry_string);
    //TODO manage maps
    if(newID <= 0)
    {
        newID = -1;
    }
    else
    {
        ROS_INFO_STREAM("New Geometry of workspace\"" << workspace_id << "\" added:  " << geometry_string);
    }
    return newID;
}


int databaseWriter::writeNewObject(std::string obj_name, std::string mesh_path, KDL::Frame obj_center)
{
  // find the first free object ID, starting from 1
  int obj_id(1);
  while(object_name_map_.count(obj_id))
    obj_id++;

  return writeNewObject(obj_id,obj_name,mesh_path,obj_center);
}

int databaseWriter::writeNewObject(int object_id, std::string obj_name, std::string mesh_path, KDL::Frame obj_center)
{
  //INSERT INTO Objects (Id, Name, MeshPath) VALUES ('Gatto','gatto.dae')

  std::string obj_center_string;
  double x,y,z,w;
  obj_center.M.GetQuaternion(x,y,z,w);
  obj_center_string = std::to_string(obj_center.p.data[0]) + " " + std::to_string(obj_center.p.data[1]) + " " + std::to_string(obj_center.p.data[2]) + " " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(w);
  
  std::string sqlstatement =
    "INSERT INTO Objects (Id, Name, MeshPath, ObjectCenter) VALUES ("
    + int_quotesql(object_id) + ","
    + str_quotesql(obj_name) + ","
    + str_quotesql(mesh_path) + ","
    + str_quotesql(obj_center_string) + ");";

  int newID = insert_db_entry(sqlstatement);
  
  if(newID <= 0)
  {
    newID = -1;
  }
  else
  {
    ROS_INFO_STREAM("New object \"" << obj_name << "\" added with ID " << newID);
    object_name_map_[newID] = obj_name;
  }

  return newID;
}

int databaseWriter::writeNewTransition(int source_grasp_id, int target_grasp_id, bool just_dont)
{
  if(grasp_name_map_.count(source_grasp_id) == 0)
  {
    ROS_ERROR_STREAM("No grasp (source) found in the DB with ID " << source_grasp_id);
    return -1;
  }
  if(grasp_name_map_.count(target_grasp_id) == 0)
  {
    ROS_ERROR_STREAM("No grasp (target) found in the DB with ID " << target_grasp_id);
    return -1;
  }
  
  const std::string& source_grasp = grasp_name_map_.at(source_grasp_id);
  const std::string& target_grasp = grasp_name_map_.at(target_grasp_id);
  
  std::pair<int,int> tx(source_grasp_id, target_grasp_id);
  
  if (transitions_set_.count(tx) != 0)
  {
    ROS_INFO_STREAM("Requested grasp transition between source:\"" << source_grasp << "\" and target:\"" << target_grasp << "\" was already present, returning...");
    return 0;
  }

  //INSERT INTO Grasp_transitions (Source_id, Target_id) VALUES ('3','1')

  std::string sqlstatement =
    "INSERT INTO Grasp_transitions (Source_id, Target_id) VALUES ("
    + int_quotesql(source_grasp_id) + ","
    + int_quotesql(target_grasp_id) + ");";

  int newID = insert_db_entry(sqlstatement,false,just_dont);
  
  if(newID <= 0)
  {
    newID = -1;
  }
  else
  {
    ROS_INFO_STREAM("New grasp transition successfully added between source:\"" << source_grasp << "\" and target:\"" << target_grasp << "\"!");
    transitions_set_.insert(tx);
  }

  return newID;
}

bool databaseWriter::deleteGrasp(int grasp_id)
{
  if(grasp_name_map_.count(grasp_id) == 0)
  {
    ROS_ERROR_STREAM("No grasp found in the DB with ID " << grasp_id);
    return -1;
  }
  
  std::string sqlstatement =
    "DELETE FROM Grasps WHERE Grasp_id="
    + int_quotesql(grasp_id) + ";";
  
  bool remove = true;
  int result = insert_db_entry(sqlstatement,remove);
  
  std::string grasp_name = grasp_name_map_.at(grasp_id);
  
  if(result == 1)
  {
    ROS_INFO_STREAM("Grasp \"" << grasp_name << "\" (id:" << grasp_id << ") successfully removed!");
    grasp_name_map_.erase(grasp_id);
  }
  
  return (result == 1);
}

bool databaseWriter::deleteObject(int obj_id)
{
  if(object_name_map_.count(obj_id) == 0)
  {
    ROS_ERROR_STREAM("No object found in the DB with ID " << obj_id);
    return -1;
  }
  
  std::string sqlstatement =
    "DELETE FROM Objects WHERE Id="
    + int_quotesql(obj_id) + ";";
  
  bool remove = true;
  int result = insert_db_entry(sqlstatement,remove);
  
  std::string obj_name = object_name_map_.at(obj_id);
  
  if(result == 1)
  {
    ROS_INFO_STREAM("Object \"" << obj_name << "\" (id:" << obj_id << ") successfully removed!");
    object_name_map_.erase(obj_id);
  }
  
  return (result == 1);
}

bool databaseWriter::deleteGraspTransition(int source_grasp_id, int target_grasp_id)
{
  if(grasp_name_map_.count(source_grasp_id) == 0)
  {
    ROS_ERROR_STREAM("No grasp (source) found in the DB with ID " << source_grasp_id);
    return -1;
  }
  if(grasp_name_map_.count(target_grasp_id) == 0)
  {
    ROS_ERROR_STREAM("No grasp (target) found in the DB with ID " << target_grasp_id);
    return -1;
  }
  
  const std::string& source_grasp = grasp_name_map_.at(source_grasp_id);
  const std::string& target_grasp = grasp_name_map_.at(target_grasp_id);

  std::string sqlstatement =
    "DELETE FROM Grasp_transitions WHERE Source_id="
    + int_quotesql(source_grasp_id) + " AND Target_id="
    + int_quotesql(target_grasp_id) + ");";

  bool remove = true;
  int result = insert_db_entry(sqlstatement,remove);
  
  if(result == 1)
  {
    ROS_INFO_STREAM("Transition between source:\"" << source_grasp << "\" and target:\"" << target_grasp << "\" successfully removed!");
  }
  
  return (result == 1);
}

int databaseWriter::writeNewEnvironmentConstraint(int id, std::string name)
{
    std::string entity = "EnvironmentConstraint";
    std::string table = entity + "s";
    std::string sqlstatement =
    "INSERT INTO " + table + " VALUES (? , ?)";
    if (ec_name_map_.count(id))
    {
        ROS_ERROR_STREAM("Error adding " << entity << " \"" << name << "\" with ID " << id << " - ID already in the database");
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, id, name);
    
    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error adding " << entity << " \"" << name << "\" with ID " << newID);
    }
    else
    {
        ROS_INFO_STREAM("New " << entity << " \"" << name << "\" added with ID " << newID);
        ec_name_map_[id] = name;
    }
    return newID;
}

int databaseWriter::writeNewECAdjacency(int source_id, int target_id)
{
    std::string sqlstatement = "INSERT INTO EC_Adjacency VALUES (? , ?)";
    if (!ec_name_map_.count(source_id) || !ec_name_map_.count(target_id))
    {
        ROS_ERROR_STREAM("One or both EnvironmentConstraints " << source_id << " and " << target_id << " do(es) not exist!");
        return -1;
    }
    if (ec_adjacency_map_.count(source_id) && ec_adjacency_map_.at(source_id).count(target_id))
    {
        ROS_ERROR_STREAM("Error already exist EC_Adjacency \"" << source_id << "\" to " << target_id);
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, source_id, target_id );
    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error writing new EC_Adjacency \"" << source_id << "\" to " << target_id);
    }
    else
    {
        ROS_INFO_STREAM("New EC_Adjacency \"" << source_id << "\" to " << target_id);
        ec_adjacency_map_[source_id].insert(target_id);
    }
    return newID;
}

int databaseWriter::writeNewECReachability(int ec_id, int workspace_id)
{
    std::string sqlstatement = "INSERT INTO EC_Reachability VALUES (? , ?)";
    if (!ec_name_map_.count(ec_id) || !workspace_name_map_.count(workspace_id))
    {
        ROS_ERROR_STREAM("EnvironmentConstraint " << ec_id << " and/or Worskspace " << workspace_id << " do(es) not exist!");
        return -1;
    }
    if (ec_reachability_map_.count(ec_id) && ec_reachability_map_.at(ec_id).count(workspace_id))
    {
        ROS_ERROR_STREAM("Error already exist EC_Reachability with id \"" << ec_id<< "\" to workspace " << workspace_id);
        return -1;
    }
    int newID = writeNewSomething(sqlstatement, ec_id, workspace_id);
    
    if(newID <= 0)
    {
        newID = -1;
        ROS_ERROR_STREAM("Error adding EC_Reachability with id \"" << ec_id<< "\" to workspace " << workspace_id);
    }
    else
    {
        ROS_INFO_STREAM("New EC_Reachability with id \"" << ec_id<< "\" to workspace " << workspace_id);
        ec_reachability_map_[ec_id].insert(workspace_id);
    }
    return newID;
}
