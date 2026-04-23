#pragma once

#include "world.h"

namespace muli3
{

class Island
{
private:
    friend class World;

    Island(World* world, int32 bodyCapacity, int32 contactCapacity);
    ~Island();

    void Add(RigidBody* body);
    void Add(Contact* contact);

    void Solve();
    void Clear();

    World* world;

    RigidBody** bodies;
    Contact** contacts;

    int32 bodyCapacity;
    int32 contactCapacity;
    int32 bodyCount;
    int32 contactCount;

    bool sleeping;
};

inline void Island::Add(RigidBody* body)
{
    MuliAssert(bodyCount < bodyCapacity);
    body->islandIndex = bodyCount;
    bodies[bodyCount++] = body;
}

inline void Island::Add(Contact* contact)
{
    MuliAssert(contactCount < contactCapacity);
    contacts[contactCount++] = contact;
}

inline void Island::Clear()
{
    bodyCount = 0;
    contactCount = 0;
    sleeping = false;
}

} // namespace muli3

