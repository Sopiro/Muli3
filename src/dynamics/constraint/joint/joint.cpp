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
    SetParameters(frequency, dampingRatio);
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

void Joint::ComputeBetaAndGamma(float effectiveMass, float dt)
{
    JointState* s = GetJointState();

    // If the frequency is less than or equal to zero, make this joint rigid
    if (frequency <= 0.0f || effectiveMass <= 0.0f)
    {
        s->beta = 1.0f;
        s->gamma = 0.0f;
    }
    else
    {
        float omega = 2.0f * pi * frequency;
        float d = 2.0f * effectiveMass * dampingRatio * omega; // Damping coefficient
        float k = effectiveMass * omega * omega;               // Spring constant
        float h = dt;

        s->beta = h * k / (d + h * k);
        s->gamma = 1.0f / ((d + h * k) * h);
    }
}

} // namespace muli3
