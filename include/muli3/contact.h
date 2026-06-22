#pragma once

#include "body.h"
#include "collider.h"
#include "collision.h"
#include "contact_solver.h"
#include "solver_states.h"

namespace muli3
{

class Contact;

struct ContactEdge
{
    Body* other;
    Contact* contact;
    ContactEdge* prev;
    ContactEdge* next;
};

class Contact
{
public:
    Contact(Collider* colliderA, Collider* colliderB);
    ~Contact() = default;

    Collider* GetColliderA() const;
    Collider* GetColliderB() const;

    Body* GetBodyA() const;
    Body* GetBodyB() const;

    const Contact* GetNext() const;
    const Contact* GetPrev() const;

    bool IsTouching() const;
    bool IsEnabled() const;
    void SetEnabled(bool enabled);
    int32 GetColorIndex() const;

    const ContactManifold& GetContactManifold() const;
    int32 GetContactCount() const;

    float GetNormalImpulse(int32 index) const;
    Vec2 GetTangentImpulse(int32 index) const;

    float GetFriction() const;
    float GetRestitution() const;
    float GetRestitutionThreshold() const;
    Vec2 GetSurfaceSpeed() const;

private:
    friend class World;
    friend class ConstraintGraph;
    friend class BroadPhase;
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
    friend class PositionSolver;
    friend struct ContactState;

    enum
    {
        flag_enabled = 1,
        flag_touching = 1 << 1,
        flag_was_touching = 1 << 2,
        flag_island = 1 << 3,
        flag_disjoint = 1 << 4,
    };

    void Update();
    void TriggerCallbacks();
    ContactState* GetContactState();
    const ContactState* GetContactState() const;

    CollideFunction* collideFunction;

    Collider* colliderA;
    Collider* colliderB;

    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    int32 id;

    int32 setIndex;
    int32 colorIndex;
    int32 localIndex;

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

inline Body* Contact::GetBodyA() const
{
    return colliderA->GetBody();
}

inline Body* Contact::GetBodyB() const
{
    return colliderB->GetBody();
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

inline int32 Contact::GetColorIndex() const
{
    return colorIndex;
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return GetContactState()->manifold;
}

inline int32 Contact::GetContactCount() const
{
    return GetContactState()->manifold.contactCount;
}

inline float Contact::GetNormalImpulse(int32 index) const
{
    MuliAssert(0 <= index && index < max_contact_point_count);
    return GetContactState()->normalContact[index].impulse;
}

inline Vec2 Contact::GetTangentImpulse(int32 index) const
{
    MuliAssert(0 <= index && index < max_contact_point_count);
    return GetContactState()->tangentContact[index].impulse;
}

inline float Contact::GetFriction() const
{
    return GetContactState()->friction;
}

inline float Contact::GetRestitution() const
{
    return GetContactState()->restitution;
}

inline float Contact::GetRestitutionThreshold() const
{
    return GetContactState()->restitutionThreshold;
}

inline Vec2 Contact::GetSurfaceSpeed() const
{
    return GetContactState()->surfaceSpeed;
}

} // namespace muli3
