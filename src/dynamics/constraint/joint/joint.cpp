#include "muli3/joint.h"
#include "muli3/solver_states.h"
#include "muli3/world.h"

namespace muli3
{

Joint::Joint(Joint::Type type, Body* bodyA, Body* bodyB, float frequency, float dampingRatio)
    : DynamicDispatcher(int32(type))
    , OnDestroy{ nullptr }
    , UserData{ nullptr }
    , bodyA{ bodyA }
    , bodyB{ bodyB }
    , setIndex{ null_index }
    , colorIndex{ null_index }
    , localIndex{ null_index }
    , flagIsland{ false }
{
    MuliAssert(bodyA->GetWorld() == bodyB->GetWorld());

    SetFrequency(frequency);
    SetDampingRatio(dampingRatio);
}

Joint::~Joint()
{
    if (OnDestroy)
    {
        OnDestroy->OnJointDestroy(this);
    }
}

JointState* Joint::GetJointState()
{
    if (colorIndex != null_index)
    {
        return &bodyA->world->constraintGraph.batches[colorIndex].jointStates[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &bodyA->world->solverSets[setIndex].jointStates[localIndex];
    }
}

const JointState* Joint::GetJointState() const
{
    if (colorIndex != null_index)
    {
        return &bodyA->world->constraintGraph.batches[colorIndex].jointStates[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &bodyA->world->solverSets[setIndex].jointStates[localIndex];
    }
}

} // namespace muli3
