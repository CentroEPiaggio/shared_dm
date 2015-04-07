#ifndef DATABASEWRITER_H
#define DATABASEWRITER_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include "ros/package.h"
#include "dual_manipulation_shared/databasemapper.h"

class databaseWriter
{
public:
  
  /**
    * @brief constructor
    * 
    * @param db_name name of the database to use for writing
    */
  databaseWriter(std::string db_name = "test.db");
  
  /**
    * @brief Insert a new object in the database
    * 
    * @param obj_name the name of the object
    * @param mesh_path the relative path of the object mesh (only the mesh name stored in the object_meshes subfolder)
    * 
    * @return the newly inserted object ID (-1 on failure)
    */
  int writeNewObject(std::string obj_name, std::string mesh_path);
  
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
    * @brief Insert a new grasp transition in the database
    * 
    * @param source_grasp_id source grasp id
    * @param target_grasp_id target grasp id
    * 
    * @return the newly inserted grasp transition ID (-1 on failure)
    */
  int writeNewTransition(int source_grasp_id, int target_grasp_id);

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
  
  ~databaseWriter();
  
private:
  std::string path_to_db_,db_name_;
  databaseMapper* db_mapper_;
  std::map<int,std::string> object_name_map_;
  std::map<int,std::string> ee_name_map_;
  std::map<int,std::string> grasp_name_map_;

  int insert_db_entry(const std::string& sqlstatement, bool remove = false);
};

#endif //DATABASEWRITER_H