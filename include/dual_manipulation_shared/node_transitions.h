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

static const std::string GRASP_NAME("grasp");
static const std::string UNGRASP_NAME("ungrasp");
static const std::string EXCHANGE_GRASP_NAME("exchange_grasp");

enum class NodeTransitionTypes
{
    UNKNOWN,                    // transition to consider empty or non-existent fields: will be found in the old way, for back-compatibility
    MOVE_NONBLOCKING,           // move with a given end-effector from one point to another, non-blocking transition
    GRASP,                      // from one non-movable to one movable, grasp the object (change on ground)
    UNGRASP,                    // from one movable to one non-movable, ungrasp the object (change on ground)
    EXCHANGE_GRASP,             // both ee are movable, perform integrasp (change above ground)
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
