#pragma once

#include "common.h"

namespace muli3
{

class World;
class Contact;
class RigidBody;
class Joint;

class Island
{
public:
    Island() = default;

    void Prepare(
        bool sleeping,
        Contact** contacts,
        RigidBody** bodies,
        Joint** joints,
        int32 contactCount,
        int32 bodyCount,
        int32 jointCount
    );
    void Solve(World* world);

    Contact** contacts;
    RigidBody** bodies;
    Joint** joints;

    int32 contactCount;
    int32 bodyCount;
    int32 jointCount;

    bool sleeping;
};

inline void Island::Prepare(
    bool inSleeping,
    Contact** inContacts,
    RigidBody** inBodies,
    Joint** inJoints,
    int32 inContactCount,
    int32 inBodyCount,
    int32 inJointCount
)
{
    sleeping = inSleeping;

    contacts = inContacts;
    bodies = inBodies;
    joints = inJoints;
    contactCount = inContactCount;
    bodyCount = inBodyCount;
    jointCount = inJointCount;
}

} // namespace muli3
