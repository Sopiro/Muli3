#include "muli3/joint.h"
#include "muli3/solver_states.h"
#include "muli3/world.h"

namespace muli3
{

Joint::Joint(Joint::Type type, RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass)
    : DynamicDispatcher(int32(type))
    , OnDestroy{ nullptr }
    , UserData{ nullptr }
    , bodyA{ bodyA }
    , bodyB{ bodyB }
    , setIndex{ null_index }
    , localIndex{ null_index }
    , flagIsland{ false }
{
    MuliAssert(bodyA->GetWorld() == bodyB->GetWorld());
    SetParameters(jointFrequency, jointDampingRatio, jointMass);
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
    return &bodyA->world->solverSets[setIndex].jointStates[localIndex];
}

const JointState* Joint::GetJointState() const
{
    return &bodyA->world->solverSets[setIndex].jointStates[localIndex];
}

void Joint::SetParameters(float newJointFrequency, float newJointDampingRatio, float newJointMass)
{
    // 0 < Frequency
    // 0 <= Damping ratio <= 1
    // 0 < Mass

    if (newJointFrequency > 0.0f)
    {
        jointFrequency = newJointFrequency;
        jointDampingRatio = Clamp(newJointDampingRatio, 0.0f, 1.0f);
        jointMass = Clamp(newJointMass, epsilon, max_float);
    }
    else
    {
        jointFrequency = -1.0f;
        jointDampingRatio = 0.0f;
        jointMass = 0.0f;
    }
}

void Joint::ComputeBetaAndGamma(const Timestep& step)
{
    JointState* s = GetJointState();

    // If the frequency is less than or equal to zero, make this joint solid
    if (jointFrequency <= 0.0f)
    {
        s->beta = 1.0f;
        s->gamma = 0.0f;
    }
    else
    {
        float omega = 2.0f * pi * jointFrequency;
        float d = 2.0f * jointMass * jointDampingRatio * omega; // Damping coefficient
        float k = jointMass * omega * omega;                    // Spring constant
        float h = step.dt;

        s->beta = h * k / (d + h * k);
        s->gamma = 1.0f / ((d + h * k) * h);
    }
}

} // namespace muli3
