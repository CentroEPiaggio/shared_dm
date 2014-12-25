#ifndef DATABASEMAPPER_H
#define DATABASEMAPPER_H
#include <string>
#include <map>
#include <vector>
#include <set>
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
    void createFakeDatabase();
    /**
     * @brief List of objects and their names
     * 
     */
    std::map<object_id,std::string> Objects;
    /**
     * @brief List of endeffectors and their names
     * 
     */
    std::map<endeffector_id,std::string> EndEffectors;
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
     * @brief Currently not used!
     * 
     */
    std::map<workspace_id,std::vector<std::pair<int,int>>> WorkspaceGeometry;
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
    bool fillTableList();
    bool fill(std::map<grasp_id, std::tuple<object_id,endeffector_id,std::string>>& data, std::string table_name);
    bool fill(std::map<object_id,std::string>& data, std::string table_name);
    bool fill(std::map<uint64_t,std::set<uint64_t>>& data, std::string table_name);
    std::vector<std::string> tables;    
    sqlite3 *db;
    
};

#endif // DATABASEMAPPER_H
