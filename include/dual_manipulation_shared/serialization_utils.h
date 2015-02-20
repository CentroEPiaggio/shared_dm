#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

#include "ros/ros.h"
#include "dual_manipulation_shared/ik_service.h"
#include <ros/package.h>

#include <ros/serialization.h>
#include <fstream>

// maximum length of a grasp message
// this does not affect older files: can be changed if needed: it's just
#define max_grasp_size 100000

bool serialize_ik(const dual_manipulation_shared::ik_service::Request & my_ik, std::string filename="unnamed_grasp.txt");

bool deserialize_ik(dual_manipulation_shared::ik_service::Request & my_ik, std::string filename="unnamed_grasp.txt");

#endif //SERIALIZATION_UTILS_H