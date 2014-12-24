#include "dual_manipulation_shared/databasemapper.h"
#include <string>
#include <sqlite3.h>
#include <iostream>
#include <unistd.h>
#include <ros/package.h>
/*
 * typedef int (*sqlite3_callback)(
 * void*,    // Data provided in the 4th argument of sqlite3_exec() 
 * int,      // The number of columns in row 
 * char**,   // An array of strings representing fields in the row 
 * char**    // An array of strings representing column names 
 * );
 */


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
        rc = sqlite3_step(stmt);
        if (rc==SQLITE_BUSY)
        {
            std::cout<<"busy, wait 1 seconds"<<std::endl;
            sleep(1);
            continue;
        }
        else if (rc != SQLITE_ROW && rc != SQLITE_DONE) {
            std::string errmsg(sqlite3_errmsg(db));
            sqlite3_finalize(stmt);
            std::cout<<errmsg<<std::endl;
            return false;//throw errmsg;
        }
        else if (rc == SQLITE_DONE) {
            exit=true;
        }
        else if (rc == SQLITE_ROW)
        {
            if (SQLITE_TEXT==sqlite3_column_type(stmt, 0))
                this->tables.push_back(std::string((const char*)sqlite3_column_text(stmt, 0)));
            else
                std::cout<<"expected a text into column 0 of table list, but it was not"<<std::endl;
        }
    }
    sqlite3_finalize(stmt);
    return true;
}

databaseMapper::databaseMapper(std::string database_name)
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
    sqlite3_close(db);
    for (auto table:tables)
        std::cout<<table<<" ";
    std::cout<<std::endl;
    
    //TODO go on like this for all the tables!
    
    
}