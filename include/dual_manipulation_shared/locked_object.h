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

#ifndef LOCKED_OBJECT_H_
#define LOCKED_OBJECT_H_

#include <memory>

namespace dual_manipulation
{
namespace shared
{

/**
 * A proxy class to ensure a certain variable is used in mutual exclusion:
 * return an object of this class constructed with the same mutex to ensure such behavior
 * 
 * @example
 * use this proxy as return of a function such the following
 * LockedObject<myClass,std::mutex> getMyLockedObject()
 * {
 *    // my_mutex is a class object
 *    return LockedObject<myClass,std::mutex>(my_object,my_mutex);
 * }
 */
template<class T, class LOCK>
class LockedObject
{
public:
    LockedObject(std::shared_ptr<T>& t, LOCK& lock) : myObj(t), myLock(lock)
    {
        myLock.lock();
    }
    
    ~LockedObject()
    {
        myLock.unlock();
    }
    
    operator const std::shared_ptr<T>&()
    {
        return myObj;
    }
    
    const std::shared_ptr<T>& operator->()
    {
        return myObj;
    }
    
private:
    std::shared_ptr<T> myObj;
    LOCK& myLock;
};

}
}

#endif
