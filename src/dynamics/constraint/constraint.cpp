#include "muli3/constraint.h"

namespace muli3
{

Constraint::Constraint(RigidBody* bodyA, RigidBody* bodyB)
    : bodyA{ bodyA }
    , bodyB{ bodyB }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    MuliAssert(bodyA->GetWorld() == bodyB->GetWorld());
}

} // namespace muli3
