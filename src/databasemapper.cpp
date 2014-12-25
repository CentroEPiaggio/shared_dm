#include "dual_manipulation_shared/databasemapper.h"
#include <string>
#include <sqlite3.h>
#include <iostream>
#include <unistd.h>
#include <ros/package.h>
#include "dual_manipulation_shared/stream_utils.h"
/*
 * typedef int (*sqlite3_callback)(
 * void*,    // Data provided in the 4th argument of sqlite3_exec() 
 * int,      // The number of columns in row 
 * char**,   // An array of strings representing fields in the row 
 * char**    // An array of strings representing column names 
 * );
 */
void databaseMapper::createFakeDatabase()
{
    
}

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

bool databaseMapper::check_type_and_copy(std::string& data, int column_index, sqlite3_stmt *stmt)
{
    if (SQLITE_TEXT==sqlite3_column_type(stmt, column_index))
        data=std::string((const char *)sqlite3_column_text(stmt,column_index));
    else
    {
        std::cout<<"expected an integer into column"<< column_index<<" but it was not"<<std::endl;
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

bool databaseMapper::fill(std::map<object_id,std::string>& data, std::string table_name)
{
    std::map<object_id,std::string> result;
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
            result[index]=name;
        }
    }
    data.swap(result);
    sqlite3_finalize(stmt);
    return true;
}

bool databaseMapper::fill(std::map<grasp_id, std::tuple<object_id,endeffector_id,std::string>>& data, std::string table_name)
{
    std::map<grasp_id, std::tuple<object_id,endeffector_id,std::string>> result;
    std::string sql = "SELECT Grasp_id, Object_id, EndEffector_id, Grasp_name FROM ";
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
            object_id objectId;
            endeffector_id endeffectorId;
            std::string name;
            check_type_and_copy(graspId,0,stmt);
            check_type_and_copy(objectId,1,stmt);
            check_type_and_copy(endeffectorId,2,stmt);
            check_type_and_copy(name,3,stmt);
            result[graspId]=std::tuple<object_id,endeffector_id,std::string>(objectId,endeffectorId,name);
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
    std::string path = ros::package::getPath("dual_manipulation_grasp_DB");
    
    int rc;
    /* Open database */
    rc = sqlite3_open(path.append("/test.db").c_str(), &db);
    if( rc ){
        std::cout<< "Can't open database" << sqlite3_errmsg(db)<<std::endl;
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
            fill(Workspaces, "Workspaces");
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
            fill(Grasp_transitions, "Grasp_transitions");
        }
        else if (table=="WorkspacesAdjacency")
        {
            fill(WorkspacesAdjacency, "WorkspacesAdjacency");
        }
        else if (table=="EndEffectors")
        {
            fill(EndEffectors, "EndEffectors");
        }
    }
    sqlite3_close(db);
}

databaseMapper::databaseMapper(std::string database_name)
{
     initialize_database(database_name);
//    createFakeDatabase();
    std::cout<<tables<<std::endl;        
    std::cout<<Grasp_transitions<<std::endl;
    std::cout<<Objects<<std::endl;
    std::cout<<Workspaces<<std::endl;
    std::cout<<Reachability<<std::endl;
    std::cout<<Grasps<<std::endl;        
}