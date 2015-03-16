#ifndef PARSING_UTILS_H
#define PARSING_UTILS_H

#include <XmlRpcValue.h>

void parseSingleParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector<double>& param, std::string param_name, int min_size = 0);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector<std::string>& param, std::string param_name, int min_size = 0);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::map< std::string, std::string >& param, std::string param_name, std::vector< std::string > names_list);

#endif //PARSING_UTILS_H