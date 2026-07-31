#pragma once

#include "body.h"
#include "collider.h"
#include "collision.h"
#include "contact_solver.h"
#include "solver_states.h"

namespace muli3
{

class Contact
{
public:
    Contact(Collider* colliderA, Collider* colliderB);
    ~Contact() = default;

    Collider* GetColliderA() const;
    Collider* GetColliderB() const;

    Body* GetBodyA() const;
    Body* GetBodyB() const;

    bool IsTouching() const;
    bool IsEnabled() const;
    void SetEnabled(bool enabled);
    int32 GetColorIndex() const;

    int32 GetManifoldCount() const;
    const ContactManifold& GetContactManifold(int32 index) const;

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

    Collider* colliderA;
    Collider* colliderB;

    int32 poolIndex;
    int32 graphIndex;

    int32 bodyIndexA;
    int32 bodyIndexB;

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

inline int32 Contact::GetManifoldCount() const
{
    return GetContactState()->manifolds.size();
}

inline const ContactManifold& Contact::GetContactManifold(int32 index) const
{
    const ContactState* state = GetContactState();
    MuliAssert(0 <= index && index < state->manifolds.size());
    return state->manifolds[index];
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
