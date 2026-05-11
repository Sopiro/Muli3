#pragma once

#include "collider.h"
#include "collision.h"
#include "contact_solver.h"
#include "position_solver.h"
#include "rigidbody.h"
#include "settings.h"

namespace muli3
{

class Contact;

struct ContactEdge
{
    RigidBody* other;
    Contact* contact;
    ContactEdge* prev;
    ContactEdge* next;
};

class Contact
{
public:
    enum
    {
        flag_enabled = 1,
        flag_touching = 1 << 1,
        flag_island = 1 << 2,
    };

    Contact(Collider* colliderA, Collider* colliderB);
    ~Contact() = default;

    Collider* GetColliderA() const;
    Collider* GetColliderB() const;
    RigidBody* GetBodyA() const;
    RigidBody* GetBodyB() const;
    RigidBody* GetReferenceBody() const;
    RigidBody* GetIncidentBody() const;

    const Contact* GetNext() const;
    const Contact* GetPrev() const;
    bool IsTouching() const;
    bool IsEnabled() const;
    void SetEnabled(bool enabled);

    const ContactManifold& GetContactManifold() const;
    int32 GetContactCount() const;
    float GetNormalImpulse(int32 index) const;
    float GetTangentImpulse(int32 index) const;

    float GetFriction() const;
    float GetRestitution() const;
    float GetRestitutionTreshold() const;
    Vec2 GetSurfaceSpeed() const;

private:
    friend class World;
    friend class Island;
    friend class ContactGraph;
    friend class BroadPhase;
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
    friend class PositionSolver;

    void Prepare(const Timestep& step);
    void SolveVelocityConstraints(const Timestep& step);
    bool SolvePositionConstraints(const Timestep& step);

    void Update();

    CollideFunction* collideFunction;

    Collider* colliderA;
    Collider* colliderB;
    RigidBody* bodyA;
    RigidBody* bodyB;
    RigidBody* b1;
    RigidBody* b2;

    Contact* prev = nullptr;
    Contact* next = nullptr;

    ContactEdge nodeA;
    ContactEdge nodeB;

    ContactManifold manifold;

    ContactSolverNormal normalSolvers[max_contact_point_count];
    ContactSolverTangent tangent1Solvers[max_contact_point_count];
    ContactSolverTangent tangent2Solvers[max_contact_point_count];
    PositionSolver positionSolvers[max_contact_point_count];

    // Impulse buffer for position correction
    // prefix 'c' stands for corrective
    Vec3 cLinearImpulseA, cLinearImpulseB;
    Vec3 cAngularImpulseA, cAngularImpulseB;

    Mat3 invIA, invIB;

    float friction;
    float restitution;
    float restitutionThreshold;
    Vec2 surfaceSpeed;
    uint16 flag;
};

inline Collider* Contact::GetColliderA() const
{
    return colliderA;
}

inline Collider* Contact::GetColliderB() const
{
    return colliderB;
}

inline RigidBody* Contact::GetBodyA() const
{
    return bodyA;
}

inline RigidBody* Contact::GetBodyB() const
{
    return bodyB;
}

inline RigidBody* Contact::GetReferenceBody() const
{
    return b1;
}

inline RigidBody* Contact::GetIncidentBody() const
{
    return b2;
}

inline const Contact* Contact::GetPrev() const
{
    return prev;
}

inline const Contact* Contact::GetNext() const
{
    return next;
}

inline bool Contact::IsTouching() const
{
    return (flag & flag_touching) == flag_touching;
}

inline bool Contact::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline void Contact::SetEnabled(bool enabled)
{
    if (enabled)
    {
        flag |= flag_enabled;
    }
    else
    {
        flag &= ~flag_enabled;
    }
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return manifold;
}

inline int32 Contact::GetContactCount() const
{
    return manifold.contactCount;
}

inline float Contact::GetNormalImpulse(int32 index) const
{
    MuliAssert(0 <= index && index < max_contact_point_count);
    return normalSolvers[index].impulse;
}

inline float Contact::GetTangentImpulse(int32 index) const
{
    MuliAssert(0 <= index && index < max_contact_point_count);
    return tangent1Solvers[index].impulse;
}

inline float Contact::GetFriction() const
{
    return friction;
}

inline float Contact::GetRestitution() const
{
    return restitution;
}

inline float Contact::GetRestitutionTreshold() const
{
    return restitutionThreshold;
}

inline Vec2 Contact::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

} // namespace muli3
