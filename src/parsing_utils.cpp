#include "../include/dual_manipulation_shared/parsing_utils.h"

// only used for INFO, ASSERT, and WARN functions
#include "ros/ros.h"

void parseSingleParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM("No value for " << param_name <<". Check the yaml configuration, we will use " << (param?"true":"false") << " as default value.");
	return;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
    param = (bool) params[param_name];
    ROS_INFO_STREAM("Read parameter " << param_name << " = " << (param?"true":"false"));

    return;
}

void parseSingleParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM("No value for " << param_name <<". Check the yaml configuration, we will use " << param << " as default value.");
	return;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    param = (double) params[param_name];
    ROS_INFO_STREAM("Read parameter " << param_name << " = " << param);

    return;
}

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name)
{
    if( !params.hasMember(param_name) )
    {
        ROS_WARN_STREAM("No value for " << param_name <<". Check the yaml configuration, we will use " << param << " as default value.");
	return;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeString);
    param = (std::string) params[param_name];
    ROS_INFO_STREAM("Read parameter " << param_name << " = " << param);

    return;
}

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector< double >& param, std::string param_name, int min_size)
{
    std::string vector_str;
    
    if( !params.hasMember(param_name) )
    {
	for(auto item:param)
	  vector_str.append( std::to_string(item) ).append(" | ");
	
        ROS_WARN_STREAM("No value for " << param_name <<". Check the yaml configuration, we will use | " << vector_str << "as default value.");
	return;
    }
    
    if( params[param_name].size() < min_size )
    {
	for(auto item:param)
	  vector_str.append( std::to_string(item) ).append(" | ");
	
        ROS_WARN_STREAM("Parameter array " << param_name << " did not have enough elements (min " << min_size << "). Check the yaml configuration, we will use | " << vector_str << "as default value.");
	return;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    param.clear();
    for(int i=0; i< params[param_name].size(); i++)
    {
      param.push_back( (double) params[param_name][i]);
      vector_str.append( std::to_string(param.back()) ).append(" | ");
    }
    
    ROS_INFO_STREAM("Read parameter " << param_name << " = | " << vector_str);

    return;
}

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector< std::string >& param, std::string param_name, int min_size)
{
    std::string vector_str;
    
    if( !params.hasMember(param_name) )
    {
	for(auto item:param)
	  vector_str.append( item ).append(" | ");
	
        ROS_WARN_STREAM("No value for " << param_name <<". Check the yaml configuration, we will use | " << vector_str << "as default value.");
	return;
    }
    
    if( params[param_name].size() < min_size )
    {
	for(auto item:param)
	  vector_str.append( item ).append(" | ");
	
        ROS_WARN_STREAM("Parameter array " << param_name << " did not have enough elements (min " << min_size << "). Check the yaml configuration, we will use | " << vector_str << "as default value.");
	return;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    param.clear();
    for(int i=0; i< params[param_name].size(); i++)
    {
      param.push_back( (std::string) params[param_name][i]);
      vector_str.append( param.back() ).append(" | ");
    }
    
    ROS_INFO_STREAM("Read parameter " << param_name << " = | " << vector_str);

    return;
}

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::map< std::string, std::string >& param, std::string param_name, std::vector< std::string > names_list)
{
    std::string vector_str;
    
    if( !params.hasMember(param_name) )
    {
	for(auto item:param)
	  vector_str.append( item.first ).append(" : ").append( item.second ).append(" | ");
	
        ROS_WARN_STREAM("No value for " << param_name <<". Check the yaml configuration, we will use | " << vector_str << "as default value.");
	return;
    }

    ROS_ASSERT(params[param_name].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    std::map< std::string, std::string > param_tmp;
    
    for(int i=0; i< names_list.size(); i++)
    {
      if( !params[param_name].hasMember(names_list.at(i)) )
      {
	ROS_WARN_STREAM("No value for " << param_name << ":" << names_list.at(i) << ". Check the yaml configuration.");
	continue;
      }
      param_tmp[names_list.at(i)] = (std::string)params[param_name][names_list.at(i)];
      vector_str.append( names_list.at(i) ).append(" : ").append( param_tmp[names_list.at(i)] ).append(" | ");
    }
    
    if(!param_tmp.empty())
    {
      ROS_INFO_STREAM("Read parameter " << param_name << " = | " << vector_str);
      param.swap(param_tmp);
      param_tmp.clear();
    }
    else
    {
      for(auto item:param)
	  vector_str.append( item.first ).append(" : ").append( item.second ).append(" | ");
      
      ROS_WARN_STREAM("Read EMPTY parameter " << param_name << ". Check the yaml configuration, we will use | " << vector_str << "as default value.");
    }

    return;
}

