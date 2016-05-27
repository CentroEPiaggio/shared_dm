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

#ifndef PARSING_UTILS_H
#define PARSING_UTILS_H

#include <XmlRpcValue.h>

void parseSingleParameter(XmlRpc::XmlRpcValue& params, bool& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, double& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, int& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::string& param, std::string param_name);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector<double>& param, std::string param_name, int min_size = 0);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::vector<std::string>& param, std::string param_name, int min_size = 0);

void parseSingleParameter(XmlRpc::XmlRpcValue& params, std::map< std::string, std::string >& param, std::string param_name, std::vector< std::string > names_list);

#endif //PARSING_UTILS_H