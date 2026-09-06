#include "muli3/joint.h"
#include "muli3/solver_states.h"
#include "muli3/world.h"

namespace muli3
{

Joint::Joint(Joint::Type type, Body* bodyA, Body* bodyB)
    : DynamicDispatcher(int32(type))
    , OnDestroy{ nullptr }
    , UserData{ nullptr }
    , bodyA{ bodyA }
    , bodyB{ bodyB }
    , worldIndex{ null_index }
    , bodyIndexA{ null_index }
    , bodyIndexB{ null_index }
    , setIndex{ null_index }
    , colorIndex{ null_index }
    , localIndex{ null_index }
    , flagIsland{ false }
{
    MuliAssert(bodyA->GetWorld() == bodyB->GetWorld());
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
    World* world = bodyA->GetWorld();

    if (colorIndex != null_index)
    {
        return &world->constraintGraph.batches[colorIndex].scalarJoints.states[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &world->solverSets[setIndex].jointStates[localIndex];
    }
}

const JointState* Joint::GetJointState() const
{
    World* world = bodyA->GetWorld();

    if (colorIndex != null_index)
    {
        return &world->constraintGraph.batches[colorIndex].scalarJoints.states[localIndex];
    }
    else
    {
        MuliAssert(setIndex != null_index);
        return &world->solverSets[setIndex].jointStates[localIndex];
    }
}

} // namespace muli3
