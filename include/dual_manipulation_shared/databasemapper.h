#ifndef DATABASEMAPPER_H
#define DATABASEMAPPER_H
#include <string>
#include <map>
#include <vector>
#include <sqlite3.h>

typedef uint64_t object_id;
typedef uint64_t grasp_id;
typedef uint64_t workspace_id;
typedef uint64_t endeffector_id;

/**
 * @brief This is a low level database mapper that directly exposes
 * database tables as std::tuples or std::maps. It should not be used 
 * unless you really know what you are doing. 
 * 
 */
class databaseMapper
{
public:
    databaseMapper(std::string database_name="test.db");
    /**
     * @brief List of objects and their names
     * 
     */
    std::map<object_id,std::string> Objects;
    /**
     * @brief From a workspace to an adjacent one
     * 
     */
    std::vector<std::pair<workspace_id,workspace_id>> Workspaces;
    /**
     * @brief List of grasps, each grasp is associated to an object, an e.e. and a name
     * grasp_id -> object_id,endeffector_id,std::string
     */
    std::map<grasp_id, std::tuple<object_id,endeffector_id,std::string>> Grasps;
    std::vector<std::pair<endeffector_id,workspace_id>> Reachability;
    /**
     * @brief Currently not used!
     * 
     */
    std::map<workspace_id,std::vector<std::pair<int,int>>> Workspace_geometry;
    /**
     * @brief From a grasp to another
     * 
     */
    std::map<grasp_id,grasp_id> Grasp_transitions;
private:
    bool fillTableList();
    std::vector<std::string> tables;    
    sqlite3 *db;
    
};

#endif // DATABASEMAPPER_H
