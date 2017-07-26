#ifndef NODE_TRANSITIONS_H
#define NODE_TRANSITIONS_H
#include <map>
#include <iostream>

/**
 * @brief A class for defining all possible transitions which are allowed between nodes in the graph for planning
 */
namespace dual_manipulation
{
namespace shared
{

static const std::string UNKNOWN_NAME("UNKNOWN");
static const std::string MOVE_NONBLOCKING_NAME("MOVE_NONBLOCKING");
static const std::string GRASP_NAME("GRASP");
static const std::string UNGRASP_NAME("UNGRASP");
static const std::string EXCHANGE_GRASP_NAME("EXCHANGE_GRASP");
static const std::string SLIDE_NAME("SLIDE");
static const std::string TILT_NAME("TILT");
static const std::string LAST_EE_FIXED_NAME("LAST_EE_FIXED");
static const std::string LAST_EE_MOVABLE_NAME("LAST_EE_MOVABLE");

enum class NodeTransitionTypes
{
    UNKNOWN,                    // transition to consider empty or non-existent fields: will be found in the old way, for back-compatibility
    MOVE_NONBLOCKING,           // move with a given end-effector from one point to another, non-blocking transition
    GRASP,                      // from one non-movable to one movable, grasp the object (change on ground)
    UNGRASP,                    // from one movable to one non-movable, ungrasp the object (change on ground)
    EXCHANGE_GRASP,             // both ee are movable, perform integrasp (change above ground)
    SLIDE,                      // both ee are non-movable, use another movable end-effector to slide the object from source to target
    TILT,                       // both ee are non-movable, use another movable end-effector to tilt the object from source to target
    LAST_EE_FIXED,              // ee_id is the last end effector in the path, and is not movable
    LAST_EE_MOVABLE             // ee_id is the last end effector in the path, and is movable
};

class NodeTransitions
{
public:
    NodeTransitions()
    {
        fromName[GRASP_NAME] = NodeTransitionTypes::GRASP;
        fromName[UNGRASP_NAME] = NodeTransitionTypes::UNGRASP;
        fromName[EXCHANGE_GRASP_NAME] = NodeTransitionTypes::EXCHANGE_GRASP;
        fromName[SLIDE_NAME] = NodeTransitionTypes::SLIDE;
        fromName[TILT_NAME] = NodeTransitionTypes::TILT;
    }
    NodeTransitionTypes getTransitionTypeFromName(const std::string& name) const
    {
        if(!fromName.count(name))
            return NodeTransitionTypes::UNKNOWN;
        
        return fromName.at(name);
    }
    
private:
    std::map<std::string,NodeTransitionTypes> fromName;
};

}
}

std::ostream& operator<<(std::ostream &output, const dual_manipulation::shared::NodeTransitionTypes &ntype);

#endif // NODE_TRANSITIONS_H
