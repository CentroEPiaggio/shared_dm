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

#include "dual_manipulation_shared/databasemapper.h"
#include <string>
#include <sqlite3.h>
#include <iostream>
#include <sstream>      // std::istringstream
#include <string>       // std::string
#include <unistd.h>
#include <string.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "dual_manipulation_shared/stream_utils.h"
#include <dual_manipulation_shared/parsing_utils.h>
#include <kdl/frames_io.hpp>

#define DEBUG 0

/*
 * typedef int (*sqlite3_callback)(
 * void*,    // Data provided in the 4th argument of sqlite3_exec() 
 * int,      // The number of columns in row 
 * char**,   // An array of strings representing fields in the row 
 * char**    // An array of strings representing column names 
 * );
 */

bool databaseMapper::prepare_query(std::string table_name, sqlite3_stmt **stmt)
{
    std::string sql = "SELECT * FROM ";
    sql.append(table_name);
    sql.append(";");
    int rc = sqlite3_prepare_v2(db, sql.c_str(), -1, stmt, NULL);
    if (rc != SQLITE_OK)
    {
        std::cout<<std::string(sqlite3_errmsg(db))<<std::endl;
        return false;//throw std::string(sqlite3_errmsg(db));
    }
    return true;
}

bool databaseMapper::step_query(sqlite3_stmt *stmt, int& rc)
{
    bool busy=true;
    while (busy)
    {
        rc = sqlite3_step(stmt);
        if (rc==SQLITE_BUSY)
        {
            std::cout<<"busy, wait 1 seconds"<<std::endl;
            sleep(1);
            busy=true;
        }
        else
            busy=false;
    }
    if (rc != SQLITE_ROW && rc != SQLITE_DONE) {
        std::string errmsg(sqlite3_errmsg(db));
        sqlite3_finalize(stmt);
        std::cout<<errmsg<<std::endl;
        return false;//throw errmsg;
    }
    else if (rc == SQLITE_DONE) {
        return true;
    }
    else if (rc == SQLITE_ROW)
    {
        return true;
    }
}

bool databaseMapper::check_type_and_copy_silent(std::string& data, int column_index, sqlite3_stmt *stmt)
{
    if (SQLITE_TEXT==sqlite3_column_type(stmt, column_index))
        data=std::string((const char *)sqlite3_column_text(stmt,column_index));
    else
    {
        return false;
    }
    return true;
}

bool databaseMapper::check_type_and_copy_silent(double& data, int column_index, sqlite3_stmt *stmt)
{
    if (SQLITE_FLOAT==sqlite3_column_type(stmt, column_index))
        data=sqlite3_column_double(stmt,column_index);
    else if(SQLITE_INTEGER==sqlite3_column_type(stmt, column_index))
        data=sqlite3_column_int64(stmt,column_index);
    else
    {
        return false;
    }
    return true;
}

bool databaseMapper::check_type_and_copy(std::string& data, int column_index, sqlite3_stmt *stmt)
{
    if (SQLITE_TEXT==sqlite3_column_type(stmt, column_index))
        data=std::string((const char *)sqlite3_column_text(stmt,column_index));
    else
    {
        std::cout<<"expected a string into column"<< column_index<<" but it was not"<<std::endl;
        return false;
    }
    return true;
}

bool databaseMapper::check_type_and_copy(uint64_t& data, int column_index, sqlite3_stmt *stmt)
{
    if (SQLITE_INTEGER==sqlite3_column_type(stmt, column_index))
        data=sqlite3_column_int64(stmt,column_index);
    else
    {
        std::cout<<"expected an integer into column"<< column_index<<" but it was not"<<std::endl;
        return false;
    }
    return true;
}

bool databaseMapper::fill(std::map< workspace_id, std::vector< std::pair< double, double >>>& data, std::string table_name)
{
    std::map< workspace_id, std::vector< std::pair< double, double >>> result;
    sqlite3_stmt *stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE)
        {
            exit = true;
        }
        else if (rc==SQLITE_ROW)
        {
            uint64_t id;
            std::string data;
            std::vector<std::pair<double,double>> blob;
            check_type_and_copy(id,0,stmt);
            check_type_and_copy(data,1,stmt);
            double x=0,y=0;
            std::istringstream iss (data);
            while (!iss.eof())
            {
                iss >> x >> y;
                blob.push_back(std::make_pair(x,y));
            }
            result[id]=blob;
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;
    
}

bool databaseMapper::fill(std::map<uint64_t,std::set<uint64_t>>& data, std::string table_name)
{
    std::map<uint64_t,std::set<uint64_t>> result;
    sqlite3_stmt *stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE)
        {
            exit = true;
        }
        else if (rc==SQLITE_ROW)
        {
            uint64_t id;
            uint64_t item;
            check_type_and_copy(id,0,stmt);
            check_type_and_copy(item,1,stmt);
            result[id].insert(item);
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;
}

bool databaseMapper::fill_grasp_transitions(std::map< grasp_id, std::set< grasp_id > >& transitions, std::map< grasp_id, std::map< grasp_id, transition_info > >& t_info, std::string table_name)
{
    std::map<grasp_id, std::set<grasp_id>> tr_result;
    std::map<grasp_id, std::map<grasp_id, transition_info >> tr_info_result;
    sqlite3_stmt *stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE)
        {
            exit = true;
        }
        else if (rc==SQLITE_ROW)
        {
            grasp_id id;
            grasp_id item;
            std::string tr_type = "bad transition name";
            transition_cost_t tr_cost = 1.0;
            std::string busy_ee_string;
            std::vector<endeffector_id> busy_ees;
            check_type_and_copy(id,0,stmt);
            check_type_and_copy(item,1,stmt);
            
            // for backward compatibility, to avoid a huge amount of output for no reason...
            check_type_and_copy_silent(tr_cost,2,stmt);
            check_type_and_copy_silent(tr_type,3,stmt);
            check_type_and_copy_silent(busy_ee_string,4,stmt);
            
            // convert busy_ee_string to a set of end-effectors
            std::istringstream iss (busy_ee_string);
            endeffector_id ee_id;
            while (!busy_ee_string.empty() && !iss.eof())
            {
                iss >> ee_id;
                busy_ees.push_back(ee_id);
            }
            
            tr_result[id].insert(item);
            tr_info_result[id][item] = transition_info(tr_cost, node_transitions.getTransitionTypeFromName(tr_type), busy_ees);
        }
    }
    transitions.swap(tr_result);
    t_info.swap(tr_info_result);
    sqlite3_finalize(stmt);
    return true;
}

bool databaseMapper::fill(std::map<endeffector_id,std::tuple<std::string,bool>>& data, std::string table_name)
{
    std::map<endeffector_id,std::tuple<std::string,bool>> result;
    sqlite3_stmt* stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            uint64_t index;
            std::string name;
            uint64_t property;
            check_type_and_copy(index,0,stmt);
            check_type_and_copy(name,1,stmt);
            check_type_and_copy(property,2,stmt);
            result[index]=std::make_tuple(name,property);
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;
}

bool databaseMapper::fill(constraint_map_t& data, std::string table_name)
{
    constraint_map_t result;
    sqlite3_stmt* stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            uint64_t index;
            std::string name;
            check_type_and_copy(index,0,stmt);
            check_type_and_copy(name,1,stmt);
            constraint_info c_info(name);
            result.emplace(index,c_info);
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;
}

bool databaseMapper::fill(object_map_t& data, std::string table_name)
{
    object_map_t result;
    sqlite3_stmt* stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            object_id index;
            object_info obj_info;
            std::string data;
            check_type_and_copy(index,0,stmt);
            check_type_and_copy(obj_info.name,1,stmt);
            check_type_and_copy(obj_info.mesh_path,2,stmt);
            check_type_and_copy(data,3,stmt);
            std::istringstream iss (data);
            std::vector<double> object_center;
            KDL::Frame obj_f;
            while (!iss.eof())
            {
              double x;
              iss >> x;
              object_center.push_back(x);
            }
            if (object_center.size() < 6)
            {
              std::cout << "databaseMapper::fill : Error getting object center pose from DB" << std::endl;
            }
            else
            {
              obj_f.p.x(object_center[0]);
              obj_f.p.y(object_center[1]);
              obj_f.p.z(object_center[2]);
            
              if(object_center.size() == 6)
                obj_f.M = KDL::Rotation::RPY(object_center[3],object_center[4],object_center[5]);
              else
                obj_f.M = KDL::Rotation::Quaternion(object_center[3],object_center[4],object_center[5],object_center[6]);
              
              obj_info.object_center = obj_f;
            }
            result[index] = obj_info;
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;
}

bool databaseMapper::fill(databaseMapper::grasp_map_t& data, std::string table_name)
{
    databaseMapper::grasp_map_t result;
    std::string sql = "SELECT Grasp_id, Object_id, EndEffector_id, Grasp_name, EC_id FROM ";
    sql.append(table_name);
    sql.append(";");
    sqlite3_stmt *stmt;
    int rc = sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL);
    if (rc != SQLITE_OK)
    {
        std::cout<<std::string(sqlite3_errmsg(db))<<std::endl;
        return false;//throw std::string(sqlite3_errmsg(db));
    }
    bool exit=false;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            grasp_id graspId;
            grasp_info g_info;
            check_type_and_copy(graspId,0,stmt);
            check_type_and_copy(g_info.obj_id,1,stmt);
            check_type_and_copy(g_info.ee_id,2,stmt);
            check_type_and_copy(g_info.name,3,stmt);
            check_type_and_copy(g_info.ec_id,4,stmt);
            result[graspId] = g_info;
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;   
}


bool databaseMapper::fillTableList()
{
    const char *sql="SELECT name FROM sqlite_master WHERE type='table';";
    sqlite3_stmt *stmt;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK)
    {
        std::cout<<std::string(sqlite3_errmsg(db))<<std::endl;
        return false;//throw std::string(sqlite3_errmsg(db));
    }
    bool exit=false;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            std::string name;
            check_type_and_copy(name,0, stmt);
            tables.push_back(name);
        }
    }
    sqlite3_finalize(stmt);
    return true;
}

void databaseMapper::initialize_database(std::string database_name)
{
    if (database_name.empty())
    {
        std::cout << "FATAL in databaseMapper : I need the name of a database to open: did you give me one, or loaded one on the server?" << std::endl;
        abort();
    }
    
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    
    int rc;
    /* Open database */
    rc = sqlite3_open(path.append("/").append(database_name).c_str(), &db);
    if( rc ){
        std::cout<< "Can't open database: " << sqlite3_errmsg(db)<<std::endl;
        return;
    }else{
        std::cout<< "Opened database successfully" <<std::endl;
    }
    fillTableList();
    for (auto table:tables)
    {
        if (table=="Objects")
        {
            fill(Objects,"Objects");
        }
        else if (table=="Workspaces")
        {
            fill_workspaces(Workspaces, "Workspaces");
        }
        else if (table=="Grasps")
        {
            fill(Grasps,"Grasps");
        }
        else if (table=="Reachability")
        {
            fill(Reachability, "Reachability");
        }
        else if (table=="Grasp_transitions")
        {
            fill_grasp_transitions(Grasp_transitions,Grasp_transition_info,"Grasp_transitions");
        }
        else if (table=="WorkspacesAdjacency")
        {
            fill(WorkspacesAdjacency, "WorkspacesAdjacency");
            makeMapBidirectional(WorkspacesAdjacency);
            for(auto& rt:WorkspacesAdjacency)
            {
                Workspaces[rt.first].adjacent_ws = rt.second;
            }
        }
        else if (table=="EndEffectors")
        {
            fill(EndEffectors, "EndEffectors");
        }
        else if (table=="EnvironmentConstraints")
        {
            fill(EnvironmentConstraints, table);
        }
    }
    sqlite3_close(db);
}

databaseMapper::databaseMapper()
{
  std::string database_name;
  
  XmlRpc::XmlRpcValue params;
  ros::NodeHandle node;
  
  if (node.getParam("dual_manipulation_parameters", params))
    parseSingleParameter(params,database_name,"database_name");
  
  initialize_database(database_name);
  
#if DEBUG>1
  std::cout<<tables<<std::endl;
  std::cout<<Grasp_transitions<<std::endl;
  std::cout<<Grasp_transition_info<<std::endl;
  //     std::cout<<Objects<<std::endl;
  std::cout<<Workspaces<<std::endl;
  std::cout<<Reachability<<std::endl;
  std::cout<<Grasps<<std::endl;
  std::cout << "EnvironmentConstraints = \n" << EnvironmentConstraints << std::endl;
#endif
}

databaseMapper::databaseMapper(std::string database_name)
{
  initialize_database(database_name);
  
#if DEBUG>1
    std::cout<<tables<<std::endl;
    std::cout<<Grasp_transitions<<std::endl;
    std::cout<<Grasp_transition_info<<std::endl;
//     std::cout<<Objects<<std::endl;
    std::cout<<Workspaces<<std::endl;
    std::cout<<Reachability<<std::endl;
    std::cout<<Grasps<<std::endl;
    std::cout << "EnvironmentConstraints = \n" << EnvironmentConstraints << std::endl;
#endif
}

bool databaseMapper::getTransitionInfo(const object_state& source, const object_state& target, transition_info& t_info) const
{
#if DEBUG>1
    std::cout << __func__ << std::endl;
    std::cout << " - source:" << source << std::endl;
    std::cout << " - target:" << target << std::endl;
#endif
    try
    {
        endeffector_id source_ee_id =  Grasps.at(source.grasp_id_).ee_id;
        endeffector_id target_ee_id =  Grasps.at(target.grasp_id_).ee_id;
        /* Allowed transitions are as follows:
        1 - Not in the database:
                Transition between adjacent workspaces using movable end effector
        2 - In the database 
                a) In the same Workspace 
                b) Between adjacent workspaces; only not movable end effectors; at least one additional end effector can reach both source and target workspaces
        */
        // 1
        if( source.grasp_id_ == target.grasp_id_ &&
            std::get<1>(EndEffectors.at(source_ee_id)) &&//E.E is movable
            Workspaces.at(source.workspace_id_).adjacent_ws.count(target.workspace_id_) &&
            Reachability.at(source_ee_id).count(source.workspace_id_) && Reachability.at(source_ee_id).count(target.workspace_id_))
        {
            t_info.ee_ids_.clear();
            t_info.grasp_transition_type_ = dual_manipulation::shared::NodeTransitionTypes::MOVE_NONBLOCKING;
            t_info.transition_cost_ = 1;
            return true;
        }
        // from now on we are considering transitions written in the data base
        bool found = false;
        // not allowed 2
        if (!Grasp_transition_info.count(source.grasp_id_) || !Grasp_transition_info.at(source.grasp_id_).count(target.grasp_id_))
        {
        }
        // 2.a
        else if (source.workspace_id_ == target.workspace_id_)
        {
            found = true;
        }
        // 2.b
        else if( source.workspace_id_ != target.workspace_id_ && Workspaces.at(source.workspace_id_).adjacent_ws.count(target.workspace_id_) &&
            source_ee_id == target_ee_id && !std::get<1>(EndEffectors.at(source_ee_id)) )
        {
            for(auto ee:(Grasp_transition_info.at(source.grasp_id_).at(target.grasp_id_)).ee_ids_)
            {
                if (Reachability.at(ee).count(source.workspace_id_) && Reachability.at(ee).count(target.workspace_id_))
                {
                    found = true;
                    break;
                }
            }
        }
        
        if (found)
        {
            t_info = Grasp_transition_info.at(source.grasp_id_).at(target.grasp_id_);
        }
        else
            t_info.grasp_transition_type_ = dual_manipulation::shared::NodeTransitionTypes::UNKNOWN;
        return found;
    }
    catch(std::exception e)
    {
        std::cout << "databaseMapper::" << __func__ << " : failed because of the following reason \'" << e.what() << "\'\nAre you sure you wrote your database correctly?" << std::endl;
        throw e;
    }
}

void databaseMapper::makeMapBidirectional(std::map< uint64_t, std::set< uint64_t > >& map)
{
    std::map< uint64_t, std::set< uint64_t >> tmp;
    tmp = map;
    for(auto& e:tmp)
        for(auto& f:e.second)
            if(!map[f].count(e.first))
                map[f].insert(e.first);
}

std::ostream& operator<<( std::ostream& os, const transition_info& t )
{
    os << "c:" << t.transition_cost_ << " | type:" << t.grasp_transition_type_ << " | extra_ees: [ - ";
    for(auto& e:t.ee_ids_) os << e << " - ";
    os << "]" << std::endl;
    return os;
}

std::ostream& operator<<( std::ostream& os, const object_state& t )
{
    os << "g_id:" << t.grasp_id_ << " | ws_id:" << t.workspace_id_ << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const grasp_info& t)
{
    os << "obj:" << t.obj_id << " ee:" << t.ee_id << " ec:" << t.ec_id << " | " << t.name << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const object_info& t)
{
    os << " name: " << t.name << " | mesh: " << t.mesh_path << " | center:" << t.object_center << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const constraint_info& t)
{
    os << " name: " << t.name << std::endl;
    return os;
}

workspace_id databaseMapper::getWorkspaceIDFromPose(const KDL::Frame& object_pose) const
{
    for (auto workspace: Workspaces)
    {
        KDL::Frame workspace_object;
        workspace_object = workspace.second.center.Inverse()*object_pose;
        const double& xs(workspace_object.p.data[0]);
        const double& ys(workspace_object.p.data[1]);
        const double& zs(workspace_object.p.data[2]);
    
        if (zs < workspace.second.z_min_max.first || zs > workspace.second.z_min_max.second)
            continue;
        std::vector<Point> temp;
        for (auto point : workspace.second.polygon)
            temp.emplace_back(point.first,point.second);
        if (geom.point_in_ordered_polygon(xs,ys,temp))
        {
            return workspace.first;
        }
    }
    return -1;
}

bool databaseMapper::getWorkspaceCentroid(const workspace_id& ws_id, bool high_centroid, KDL::Frame& ws_centroid) const
{
    if(!Workspaces.count(ws_id))
        return false;
    
    if(!high_centroid)
        ws_centroid = Workspaces.at(ws_id).center;
    else
        ws_centroid = Workspaces.at(ws_id).center*KDL::Frame(KDL::Vector(0.0,0.0, (Workspaces.at(ws_id).z_min_max.second + Workspaces.at(ws_id).z_min_max.first)/2.0 ));
    
    return true;
}

bool databaseMapper::fill_workspaces(std::map< workspace_id, workspace_info >& data, std::string table_name)
{
    
    std::map<workspace_id, workspace_info> result;
    sqlite3_stmt* stmt;
    prepare_query(table_name,&stmt);
    bool exit=false;
    int rc;
    while(!exit)
    {
        if (!step_query(stmt,rc))
            return false;
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            uint64_t index;
            std::string name, polygon, z_min_max, centroid;
            check_type_and_copy(index,0,stmt);
            check_type_and_copy(name,1,stmt);
            check_type_and_copy(polygon,2,stmt);
            check_type_and_copy(z_min_max,3,stmt);
            check_type_and_copy(centroid,4,stmt);
            workspace_info ws_info;
            ws_info.name = name;
            double x=0,y=0;
            std::istringstream iss (polygon);
            while (!iss.eof())
            {
                iss >> x >> y;
                ws_info.polygon.push_back(std::make_pair(x,y));
            }
            
            std::istringstream iss_z (z_min_max);
            iss_z >> x >> y;
            ws_info.z_min_max = std::make_pair(x,y);
            
            
            std::istringstream iss_c (centroid);
            std::vector<double> object_center;
            KDL::Frame obj_f;
            while (!iss_c.eof())
            {
                double x;
                iss_c >> x;
                object_center.push_back(x);
            }
            if (object_center.size() < 6)
            {
                std::cout << "databaseMapper::fill : Error getting object center pose from DB" << std::endl;
            }
            else
            {
                obj_f.p.x(object_center[0]);
                obj_f.p.y(object_center[1]);
                obj_f.p.z(object_center[2]);
                
                if(object_center.size() == 6)
                    obj_f.M = KDL::Rotation::RPY(object_center[3],object_center[4],object_center[5]);
                else
                    obj_f.M = KDL::Rotation::Quaternion(object_center[3],object_center[4],object_center[5],object_center[6]);
            }
            
            ws_info.center = obj_f;
            if(data.count(index))
                ws_info.adjacent_ws = data.at(index).adjacent_ws;
            result[index]=ws_info;
        }
    }
    
    data.swap(result);
    sqlite3_finalize(stmt);
    
    return true;
}
