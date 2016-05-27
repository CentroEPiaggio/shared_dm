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
    databaseMapper();
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
