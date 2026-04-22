#pragma once

#include "collision.h"
#include "rigidbody.h"

namespace muli3
{

class Contact
{
public:
    Contact(RigidBody* bodyA, RigidBody* bodyB);
    ~Contact() = default;

    RigidBody* GetBodyA() const;
    RigidBody* GetBodyB() const;

    const ContactManifold& GetContactManifold() const;
    int32 GetContactCount() const;
    const Vec3& GetPoint() const;
    const Vec3& GetNormal() const;
    float GetPenetration() const;

private:
    friend class World;

    bool Update();
    void Solve(float invDt);

    RigidBody* bodyA;
    RigidBody* bodyB;

    ContactManifold manifold;
};

inline Contact::Contact(RigidBody* bodyA, RigidBody* bodyB)
    : bodyA{ bodyA }
    , bodyB{ bodyB }
{
}

inline RigidBody* Contact::GetBodyA() const
{
    return bodyA;
}

inline RigidBody* Contact::GetBodyB() const
{
    return bodyB;
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return manifold;
}

inline int32 Contact::GetContactCount() const
{
    return manifold.contactCount;
}

inline const Vec3& Contact::GetPoint() const
{
    return manifold.contactPoints[0].p;
}

inline const Vec3& Contact::GetNormal() const
{
    return manifold.contactNormal;
}

inline float Contact::GetPenetration() const
{
    return manifold.penetrationDepth;
}

} // namespace muli3
