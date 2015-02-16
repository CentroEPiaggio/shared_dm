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