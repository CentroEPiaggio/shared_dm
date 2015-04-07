# waypoints for the end-effector when grasping (in a frame relative to the object)
geometry_msgs/Pose[] ee_pose
# hand joint trajectory points
trajectory_msgs/JointTrajectory grasp_trajectory
# object identifier - identifies the appropriate entry in the database
uint64 object_db_id
# object to attach to the grasping end-effector
# needed fields are "link_name", "object.header.frame_id", and "object.mesh_poses"
# object.meshes is filled with a mesh read from the database using "object_db_id"
moveit_msgs/AttachedCollisionObject attObject