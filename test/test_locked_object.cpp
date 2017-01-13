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

#include "ros/ros.h"
#include "dual_manipulation_shared/locked_object.h"
#include <mutex>
#include <kdl/frames.hpp>
#include <iostream>
#include <thread>

std::mutex m,m2;
std::shared_ptr<KDL::Frame> my_frame(new KDL::Frame(KDL::Vector(0.0,0.0,0.0)));
typedef dual_manipulation::shared::LockedObject<KDL::Frame,std::mutex> my_lock;

my_lock getMyLockedFrame()
{
    return my_lock(my_frame,m);
}

void printMutex(const std::string& s)
{
    std::unique_lock<std::mutex> ul(m2);
    std::cout << s << std::endl;
}

void addToLockedFrame(KDL::Frame& f, int i)
{
    sleep(1);
    printMutex("Thread #" + std::to_string(i) + ": got one! Doing z+=0.1...");
    f.p.z( f.p.z() + 0.1 );
    printMutex("Thread #" + std::to_string(i) + ": z = " + std::to_string(f.p.z()));
}

void threadFcn(int i)
{
    printMutex("Thread #" + std::to_string(i) + ": asking for a locked frame...");
    addToLockedFrame(getMyLockedFrame(),i);
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Dual manipulation| -> test_locked_object "<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "test_serialization");
    
    std::vector<std::thread> t_live;
    for(int i=0; i<10; ++i)
    {
        t_live.push_back(std::thread(std::bind(&threadFcn,i)));
    }
    
    for(auto& t:t_live)
        t.join();
    
    return 0;
}