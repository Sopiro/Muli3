#pragma once

#include "common.h"

namespace muli3
{

class World;
struct BodyState;
struct ContactState;
struct JointState;

class Island
{
public:
    Island() = default;

    void Prepare(
        ContactState** contacts, BodyState** bodies, JointState** joints, int32 contactCount, int32 bodyCount, int32 jointCount
    );
    void Solve(World* world);

    ContactState** contacts;
    BodyState** bodies;
    JointState** joints;

    int32 contactCount;
    int32 bodyCount;
    int32 jointCount;
};

inline void Island::Prepare(
    ContactState** inContacts,
    BodyState** inBodies,
    JointState** inJoints,
    int32 inContactCount,
    int32 inBodyCount,
    int32 inJointCount
)
{
    contacts = inContacts;
    bodies = inBodies;
    joints = inJoints;
    contactCount = inContactCount;
    bodyCount = inBodyCount;
    jointCount = inJointCount;
}

} // namespace muli3
