#include <dual_manipulation_shared/node_transitions.h>

std::ostream& operator<<(std::ostream &output, const dual_manipulation::shared::NodeTransitionTypes &ntype)
{
    typedef dual_manipulation::shared::NodeTransitionTypes NodeTransitionTypes;
    
    output << (ntype==NodeTransitionTypes::UNKNOWN?"UNKNOWN":
               ntype==NodeTransitionTypes::MOVE_NONBLOCKING?"MOVE_NONBLOCKING":
               ntype==NodeTransitionTypes::GRASP?"GRASP":
               ntype==NodeTransitionTypes::UNGRASP?"UNGRASP":
               ntype==NodeTransitionTypes::EXCHANGE_GRASP?"EXCHANGE_GRASP":
               ntype==NodeTransitionTypes::SLIDE?"SLIDE":
               ntype==NodeTransitionTypes::LAST_EE_FIXED?"LAST_EE_FIXED":
               ntype==NodeTransitionTypes::LAST_EE_MOVABLE?"LAST_EE_MOVABLE":
               "you forgot to teach me how to write this type of NodeTransitionTypes!");
    
    return output;
}
