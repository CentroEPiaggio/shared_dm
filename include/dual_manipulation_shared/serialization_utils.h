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

#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <ros/package.h>
#include "dual_manipulation_shared/grasp_trajectory.h"

#include <ros/serialization.h>
#include <fstream>

// maximum length of a grasp message
// this does not affect older files: can be changed if needed: it's just
#define max_grasp_size 100000

#define OBJ_GRASP_FACTOR 1000

template< typename T> bool serialize_ik(const T & my_ik, std::string filename="unnamed_grasp.txt", std::string package="dual_manipulation_grasp_db", std::string sub_path="grasp_trajectories");

template< typename T> bool deserialize_ik(T & my_ik, std::string filename="unnamed_grasp.txt", std::string package="dual_manipulation_grasp_db", std::string sub_path="grasp_trajectories");

void monotonic_decreasing_distance_filter(std::vector<geometry_msgs::Pose> poses);

void down_sampling(std::vector<geometry_msgs::Pose>& poses, int period);

bool read_grasp_msg(uint obj_id, uint grasp_id, dual_manipulation_shared::grasp_trajectory& grasp_msg);

/**
 * @brief Serialize a grasp on disk, returning the ID to use in the database in order to then be able to read the same grasp using @read_grasp_msg function
 * 
 * @return the ID of the serialized grasp to be written in the database; if ID <= 0, an error occurred.
 */
int write_grasp_msg(uint obj_id, uint grasp_id, const dual_manipulation_shared::grasp_trajectory& grasp_msg);

int compute_grasp_id(uint obj_id, uint grasp_id);
#endif //SERIALIZATION_UTILS_H