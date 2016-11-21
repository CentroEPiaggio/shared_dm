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
#include <dual_manipulation_shared/node_transitions.h>
#include <dual_manipulation_shared/geometry_tools.h>

typedef uint64_t object_id;
typedef uint64_t grasp_id;
typedef uint64_t workspace_id;
typedef uint64_t endeffector_id;
typedef dual_manipulation::shared::NodeTransitionTypes grasp_transition_type;
typedef double transition_cost_t;
typedef uint64_t constraint_id; // not used for now

/**
 * @brief State of an object, represented by its grasp_id and workspace_id
 */
struct object_state{
    object_state(const grasp_id& g, const workspace_id& w):grasp_id_(g), workspace_id_(w){};
    grasp_id grasp_id_;
    workspace_id workspace_id_;
};
std::ostream& operator<<( std::ostream& os, const object_state& t );

/**
 * @brief Information related to an object
 */
struct object_info{
    std::string name;
    std::string mesh_path;
    KDL::Frame object_center;
};
std::ostream& operator<<( std::ostream& os, const object_info& t );

/**
 * @brief struct containing all the information related to a workspace
 */
struct workspace_info{
    std::string name;
    std::vector<std::pair<double,double>> polygon;
    std::pair<double,double> z_min_max;
    KDL::Frame center;
    std::set<workspace_id> adjacent_ws;
};
std::ostream& operator<<( std::ostream& os, const workspace_info& t );

/**
 * @brief Information regarding a certain grasp
 */
struct grasp_info{
    object_id obj_id; // the ID of the object this grasp refers to
    endeffector_id ee_id; // the ID of the end-effector for this grasp
    std::string name; // the name of the grasp
    constraint_id ec_id; // the ID of the environmental constraint for this grasp
};
std::ostream& operator<<( std::ostream& os, const grasp_info& t );

/**
 * @brief struct containing all the information related to an environmental constraint
 */
struct constraint_info{
    constraint_info(const std::string& name_) : name(name_) {};
    std::string name;
};
std::ostream& operator<<(std::ostream& os, const constraint_info& t);

/**
 * @brief Information about a transition between two object_states, containing a cost, type, and list of end-effectors
 * Notice that the field @p ee_ids_ is a list of extra candidate end effectors to be used, but only ONE will be needed to perform a given transition
 */
struct transition_info{
    transition_info(const transition_cost_t& c = 0, const grasp_transition_type& g_t = dual_manipulation::shared::NodeTransitionTypes::UNKNOWN, const std::vector<endeffector_id>& ees = std::vector<endeffector_id>()):transition_cost_(c), grasp_transition_type_(g_t){
        ee_ids_ = ees;
    };
    transition_info(const transition_info& t):transition_cost_(t.transition_cost_), grasp_transition_type_(t.grasp_transition_type_){
        ee_ids_ = t.ee_ids_;
    };
    transition_cost_t transition_cost_;
    grasp_transition_type grasp_transition_type_;
    std::vector<endeffector_id> ee_ids_; //list of extra candidate end effectors to be used: ONE will be needed to perform a given transition
};
std::ostream& operator<<( std::ostream& os, const transition_info& t );

/**
 * @brief This is a low level database mapper that directly exposes
 * database tables as std::tuples or std::maps. It should not be used 
 * unless you really know what you are doing. 
 * 
 */
class databaseMapper
{
public:
    
    typedef std::map<object_id, object_info> object_map_t;
    typedef std::map<grasp_id, grasp_info> grasp_map_t;
    typedef std::map<constraint_id, constraint_info> constraint_map_t;
    
    databaseMapper();
    databaseMapper(std::string database_name);
    /**
     * @brief List of objects and their names
     */
    object_map_t Objects;
    /**
     * @brief List of endeffectors and their names, and a bool if the e.e. is movable
     * 
     */
    std::map<endeffector_id,std::tuple<std::string,bool>> EndEffectors;
    /**
     * @brief List of workspaces and their information
     * 
     */
    std::map<workspace_id,workspace_info> Workspaces;
    /**
     * @brief List of grasps
     */
    grasp_map_t Grasps;
    std::map<endeffector_id,std::set<workspace_id>> Reachability;
    /**
     * @brief From a grasp to another
     * 
     */
    std::map<grasp_id,std::set<grasp_id>> Grasp_transitions;
    
    /**
     * @brief List of environmental constraints and their names
     */
    constraint_map_t EnvironmentConstraints;
    
    /**
     * @brief Get information about a transition 
     * 
     * @param source Initial object state of the transition (input)
     * @param target Final object state of the transition (input)
     * @param t_info Transition information
     * 
     * @return false if the transition does not exist, true otherwise
     */
    bool getTransitionInfo(const object_state& source, const object_state& target, transition_info& t_info) const;
    
    /**
     * @brief Return the ID of the workspace which contains the @p object_pose passed as input
     * 
     * @param object_pose pose of the object to find in the environment, expressed in world frame
     * 
     * @return -1 if the pose is outside all known workspaces; the found workspace_id otherwise.
     */
    workspace_id getWorkspaceIDFromPose(const KDL::Frame& object_pose) const;
    
    /**
     * @brief Return the Workspace centroid
     * 
     * @param ws_id the ID of the workspace
     * @param high_centroid a flag to decide whether to use a low (false) or high (true) centroid position
     * @param ws_centroid pose of the workspace centroid
     * 
     * @return false if the workspace does not exist
     */
    bool getWorkspaceCentroid(const workspace_id& ws_id, bool high_centroid, KDL::Frame& ws_centroid) const;
    
private:
    
    void initialize_database(std::string database_name);
    bool prepare_query(std::string table_name, sqlite3_stmt **stmt);
    bool step_query(sqlite3_stmt *stmt, int& rc);
    bool check_type_and_copy(uint64_t& data, int column_index, sqlite3_stmt* stmt);
    bool check_type_and_copy_silent(std::string& data, int column_index, sqlite3_stmt *stmt);
    bool check_type_and_copy_silent(double& data, int column_index, sqlite3_stmt *stmt);
    bool check_type_and_copy(std::string& data, int column_index, sqlite3_stmt *stmt);
    bool check_type_and_copy(char* &pzBlob, int column_index, sqlite3_stmt *stmt, int& pnBlob);
    bool fillTableList();
    bool fill(grasp_map_t& data, std::string table_name);
    bool fill(constraint_map_t& data, std::string table_name);
    bool fill(object_map_t& data, std::string table_name);
    bool fill(std::map<uint64_t,std::set<uint64_t>>& data, std::string table_name);
    bool fill_grasp_transitions(std::map< grasp_id, std::set< grasp_id > >& transitions, std::map< grasp_id, std::map< grasp_id, transition_info > >& t_info, std::string table_name);
    bool fill(std::map<endeffector_id,std::tuple<std::string,bool>>& data, std::string table_name);
    bool fill(std::map<workspace_id,std::vector<std::pair<double,double>>>& data, std::string table_name);
    bool fill_workspaces(std::map<workspace_id, workspace_info>& data, std::string table_name);
    void makeMapBidirectional(std::map< uint64_t, std::set< uint64_t > >& map);
    std::vector<std::string> tables;
    sqlite3 *db;
    /// Between two grasps, tell me the type of the transition and other useful information
    std::map<grasp_id,std::map<grasp_id,transition_info>> Grasp_transition_info;
    /// contains information about transition types from names
    const dual_manipulation::shared::NodeTransitions node_transitions;
    geometry_tools geom;
    std::map<workspace_id,std::set<workspace_id>> WorkspacesAdjacency;
};

#endif // DATABASEMAPPER_H
