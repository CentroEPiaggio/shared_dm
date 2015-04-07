#include "dual_manipulation_shared/databasewriter.h"
#include <sqlite3.h>

std::string str_quotesql( const std::string& s ) {
  return std::string("'") + s + std::string("'");
}

std::string int_quotesql( const int& s ) {
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
  
  for(auto& ee:db_mapper_->Grasps)
    grasp_name_map_[ee.first] = std::get<2>(ee.second);
  
  std::cout << "Found " << grasp_name_map_.size() << " grasps already in the DB" << std::endl;
}

int databaseWriter::insert_db_entry(const std::string& sqlstatement, bool remove)
{
  sqlite3 *db;
  sqlite3_stmt * stmt;
  int scoreID = -1;

  if (sqlite3_open((path_to_db_+"/"+db_name_).c_str(), &db) == SQLITE_OK)
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
  sqlite3_close(db);

  return scoreID;
}

databaseWriter::~databaseWriter()
{
  delete db_mapper_;
}

int databaseWriter::writeNewGrasp(int object_id, int end_effector_id, std::string grasp_name)
{
  if(object_name_map_.count(object_id) == 0)
  {
    ROS_ERROR_STREAM("No object found in the DB with ID " << object_id);
    return -1;
  }
  if(ee_name_map_.count(object_id) == 0)
  {
    ROS_ERROR_STREAM("No end-effector found in the DB with ID " << end_effector_id);
    return -1;
  }
  
  const std::string& obj_name = object_name_map_.at(object_id);
  const std::string& ee_name = ee_name_map_.at(end_effector_id);
  std::string empty = "";

  //INSERT INTO Grasps (Object_id, EndEffector_id, Grasp_id, Grasp_info, Grasp_name) VALUES ('3','1','','gatto')

  std::string sqlstatement =
    "INSERT INTO Grasps (Object_id, EndEffector_id, Grasp_info, Grasp_name) VALUES ("
    + int_quotesql(object_id) + ","
    + int_quotesql(end_effector_id) + ","
    + str_quotesql(empty) + ","
    + str_quotesql(grasp_name) + ");";

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

int databaseWriter::writeNewObject(std::string obj_name, std::string mesh_path)
{
  //INSERT INTO Objects (Id, Name, MeshPath) VALUES ('Gatto','gatto.dae')

  std::string sqlstatement =
    "INSERT INTO Objects (Name, MeshPath) VALUES ("
    + str_quotesql(obj_name) + ","
    + str_quotesql(mesh_path) + ");";

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

int databaseWriter::writeNewTransition(int source_grasp_id, int target_grasp_id)
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

  //INSERT INTO Grasp_transitions (Source_id, Target_id) VALUES ('3','1')

  std::string sqlstatement =
    "INSERT INTO Grasp_transitions (Source_id, Target_id) VALUES ("
    + int_quotesql(source_grasp_id) + ","
    + int_quotesql(target_grasp_id) + ");";

  int newID = insert_db_entry(sqlstatement);
  
  if(newID <= 0)
  {
    newID = -1;
  }
  else
  {
    ROS_INFO_STREAM("New grasp transition successfully added between source:\"" << source_grasp << "\" and target:\"" << target_grasp << "\"!");
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
