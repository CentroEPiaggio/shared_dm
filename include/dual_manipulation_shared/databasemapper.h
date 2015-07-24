#ifndef DATABASEMAPPER_H
#define DATABASEMAPPER_H
#include <string>
#include <map>
#include <vector>
#include <set>
#include <sqlite3.h>
#include <kdl/frames.hpp>

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
    databaseMapper(int i);
    databaseMapper(std::string database_name);
    /**
     * @brief List of objects and their names
     * 
     */
    std::map<object_id,std::tuple<std::string,std::string,KDL::Frame>> Objects;
    /**
     * @brief List of endeffectors and their names, and a bool if the e.e. is movable
     * 
     */
    std::map<endeffector_id,std::tuple<std::string,bool>> EndEffectors;
    /**
     * @brief List of workspaces and their names
     * 
     */
    std::map<workspace_id,std::string> Workspaces;
    /**
     * @brief From a workspace to an adjacent one
     * 
     */
    std::map<workspace_id,std::set<workspace_id>> WorkspacesAdjacency;
    /**
     * @brief List of grasps, each grasp is associated to an object, an e.e. and a name
     * grasp_id -> object_id,endeffector_id,std::string
     */
    std::map<grasp_id, std::tuple<object_id,endeffector_id,std::string>> Grasps;
    std::map<endeffector_id,std::set<workspace_id>> Reachability;

    /**
     * @brief This table associates a 2Dpolygon (vector of 2D points) to each workspace
     * It is currently used to convert semantic into cartesian and viceversa
     */
    std::map<workspace_id,std::vector<std::pair<double,double>>> WorkspaceGeometry;
    /**
     * @brief From a grasp to another
     * 
     */
    std::map<grasp_id,std::set<grasp_id>> Grasp_transitions;
private:
    void initialize_database(std::string database_name);
    bool prepare_query(std::string table_name, sqlite3_stmt **stmt);
    bool step_query(sqlite3_stmt *stmt, int& rc);
    bool check_type_and_copy(uint64_t& data, int column_index, sqlite3_stmt* stmt);
    bool check_type_and_copy(std::string& data, int column_index, sqlite3_stmt *stmt);
    bool check_type_and_copy(char* &pzBlob, int column_index, sqlite3_stmt *stmt, int& pnBlob);
    bool fillTableList();
    bool fill(std::map<grasp_id, std::tuple<object_id,endeffector_id,std::string>>& data, std::string table_name);
    bool fill(std::map<uint64_t,std::string>& data, std::string table_name);
    bool fill(std::map< uint64_t, std::tuple< std::string, std::string, KDL::Frame > >& data, std::string table_name);
    bool fill(std::map<uint64_t,std::set<uint64_t>>& data, std::string table_name);
    bool fill(std::map<endeffector_id,std::tuple<std::string,bool>>& data, std::string table_name);
    bool fill(std::map<workspace_id,std::vector<std::pair<double,double>>>& data, std::string table_name);
    std::vector<std::string> tables;
    sqlite3 *db;
};

#endif // DATABASEMAPPER_H
