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

#ifndef STREAM_UTILS_H
#define STREAM_UTILS_H


#include <map>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <forward_list>

template< typename K,typename T>//,typename C, typename Alloc >
std::ostream& operator<<( std::ostream& os, const std::map<K,T>& m )
{
    for (typename std::map<K,T>::const_iterator it=m.begin();it!=m.end();++it)
        os<<it->first <<" "<<it->second <<std::endl;
    os<<"-------"<<std::endl;
    return os;
}

template< typename K,typename T,typename C >
std::ostream& operator<<( std::ostream& os, const std::tuple<K,T,C>& m )
{
    os<<std::get<0>(m)<<" "<<std::get<1>(m)<<" "<<std::get<2>(m);
    os<<std::endl;
    return os;
}

template< typename K,typename T >
std::ostream& operator<<( std::ostream& os, const std::tuple<K,T>& m )
{
    os<<std::get<0>(m)<<" "<<std::get<1>(m);
    os<<std::endl;
    return os;
}

template< typename T>//,typename C, typename Alloc >
std::ostream& operator<<( std::ostream& os, const std::vector<T>& m )
{
    os<<std::endl;
    for (typename std::vector<T>::const_iterator it=m.begin();it!=m.end();++it)
        os<<(*it)<<" ";
    os<<std::endl<<"-------"<<std::endl;
    return os;
}


template< typename T>//,typename C, typename Alloc >
std::ostream& operator<<( std::ostream& os, const std::list<T>& m )
{
    os<<std::endl;
    for (typename std::list<T>::const_iterator it=m.begin();it!=m.end();++it)
        os<<(*it)<<" ";
    os<<std::endl<<"-------"<<std::endl;
    return os;
}

template< typename T>//,typename C, typename Alloc >
std::ostream& operator<<( std::ostream& os, const std::set<T>& m )
{
    for (typename std::set<T>::const_iterator it=m.begin();it!=m.end();++it)
        os<<(*it)<<" ";
    return os;
}

template<typename T, typename K>
std::ostream& operator<<(std::ostream& os, const std::pair<T,K>& m)
{
    os<<m.first<<" "<<m.second<<" ";
    return os;
}
#endif