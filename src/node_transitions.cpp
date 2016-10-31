#include <dual_manipulation_shared/node_transitions.h>

std::ostream& operator<<(std::ostream &output, const dual_manipulation::shared::NodeTransitionTypes &ntype)
{
    typedef dual_manipulation::shared::NodeTransitionTypes NodeTransitionTypes;
    
    output << (ntype==NodeTransitionTypes::UNKNOWN?dual_manipulation::shared::UNKNOWN_NAME:
               ntype==NodeTransitionTypes::MOVE_NONBLOCKING?dual_manipulation::shared::MOVE_NONBLOCKING_NAME:
               ntype==NodeTransitionTypes::GRASP?dual_manipulation::shared::GRASP_NAME:
               ntype==NodeTransitionTypes::UNGRASP?dual_manipulation::shared::UNGRASP_NAME:
               ntype==NodeTransitionTypes::EXCHANGE_GRASP?dual_manipulation::shared::EXCHANGE_GRASP_NAME:
               ntype==NodeTransitionTypes::SLIDE?dual_manipulation::shared::SLIDE_NAME:
               ntype==NodeTransitionTypes::LAST_EE_FIXED?dual_manipulation::shared::LAST_EE_FIXED_NAME:
               ntype==NodeTransitionTypes::LAST_EE_MOVABLE?dual_manipulation::shared::LAST_EE_MOVABLE_NAME:
               "you forgot to teach me how to write this type of NodeTransitionTypes!");
    
    return output;
}
