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

#include "../include/dual_manipulation_shared/parsing_utils.h"

// only used for INFO, ASSERT, and WARN functions
#include "ros/ros.h"

#define CLASS_NAMESPACE "ParsingUtils::"

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use " << (param?"true":"false") << " as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
    param = (bool) params[param_name];
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = " << (param?"true":"false"));

    return true;
}

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use " << param << " as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    param = (double) params[param_name];
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = " << param);

    return true;
}

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, int& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use " << param << " as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt);
    param = (int) params[param_name];
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = " << param);

    return true;
}

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use " << param << " as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeString);
    param = (std::string) params[param_name];
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = " << param);

    return true;
}

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector< double >& param, std::string param_name, int min_size)
{
    std::string vector_str;
    
    if( !params.hasMember(param_name) )
    {
        for(auto item:param)
            vector_str.append( std::to_string(item) ).append(" | ");
        
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use | " << vector_str << "as default value.");
        return false;
    }
    
    if( params[param_name].size() < min_size )
    {
        for(auto item:param)
            vector_str.append( std::to_string(item) ).append(" | ");
        
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Parameter array " << param_name << " did not have enough elements (min " << min_size << "). Check the yaml configuration, we will use | " << vector_str << "as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    param.clear();
    for(int i=0; i< params[param_name].size(); i++)
    {
      param.push_back( (double) params[param_name][i]);
      vector_str.append( std::to_string(param.back()) ).append(" | ");
    }
    
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = | " << vector_str);

    return true;
}

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector< std::string >& param, std::string param_name, int min_size)
{
    std::string vector_str;
    
    if( !params.hasMember(param_name) )
    {
        for(auto item:param)
            vector_str.append( item ).append(" | ");
        
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use | " << vector_str << "as default value.");
        return false;
    }
    
    if( params[param_name].size() < min_size )
    {
        for(auto item:param)
            vector_str.append( item ).append(" | ");
        
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Parameter array " << param_name << " did not have enough elements (min " << min_size << "). Check the yaml configuration, we will use | " << vector_str << "as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    param.clear();
    for(int i=0; i< params[param_name].size(); i++)
    {
      param.push_back( (std::string) params[param_name][i]);
      vector_str.append( param.back() ).append(" | ");
    }
    
    ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = | " << vector_str);

    return true;
}

bool parseSingleParameter(XmlRpc::XmlRpcValue& params, std::map< std::string, std::string >& param, std::string param_name, std::vector< std::string > names_list)
{
    std::string vector_str;
    
    if( !params.hasMember(param_name) )
    {
        for(auto item:param)
            vector_str.append( item.first ).append(" : ").append( item.second ).append(" | ");
        
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name <<". Check the yaml configuration, we will use | " << vector_str << "as default value.");
        return false;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    std::map< std::string, std::string > param_tmp;
    bool everything_ok(true);
    
    for(int i=0; i< names_list.size(); i++)
    {
        if( !params[param_name].hasMember(names_list.at(i)) )
        {
            ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "No value for " << param_name << ":" << names_list.at(i) << ". Check the yaml configuration.");
            everything_ok = false;
            continue;
        }
        param_tmp[names_list.at(i)] = (std::string)params[param_name][names_list.at(i)];
        vector_str.append( names_list.at(i) ).append(" : ").append( param_tmp[names_list.at(i)] ).append(" | ");
    }
    
    if(!param_tmp.empty())
    {
        ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read parameter " << param_name << " = | " << vector_str);
        param.swap(param_tmp);
        param_tmp.clear();
    }
    else
    {
        for(auto item:param)
            vector_str.append( item.first ).append(" : ").append( item.second ).append(" | ");
        
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : " << "Read EMPTY parameter " << param_name << ". Check the yaml configuration, we will use | " << vector_str << "as default value.");
        return false;
    }

    return everything_ok;
}

