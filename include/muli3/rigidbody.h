#pragma once

#include "transform.h"

namespace muli3
{

class Shape;
class World;
class Contact;
struct ContactEdge;

class RigidBody
{
public:
    Transform transform{};
    Vec3 linearVelocity{ 0.0f, 0.0f, 0.0f };
    Vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };
    float invMass = 0.0f;
    float restitution = 0.0f;
    float friction = 0.5f;
    Shape* shape = nullptr;
    Vec3 force{ 0.0f, 0.0f, 0.0f };
    Vec3 torque{ 0.0f, 0.0f, 0.0f };
    Transform transform0{};

    RigidBody() = default;

    bool IsStatic() const;
    void SetEnabled(bool enabled);
    bool IsEnabled() const;
    bool IsSleeping() const;
    void Awake();
    void Sleep();

    int32 GetIslandID() const;
    int32 GetIslandIndex() const;

    RigidBody* GetPrev();
    const RigidBody* GetPrev() const;
    RigidBody* GetNext();
    const RigidBody* GetNext() const;
    World* GetWorld();
    const World* GetWorld() const;

    float GetMass() const;
    Vec3 GetWorldCenterOfMass() const;
    Mat3 GetInverseInertiaTensorLocal() const;
    Mat3 GetInverseInertiaTensorWorld() const;

    void SetMass(float mass);
    void ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse);
    void ApplyLinearImpulse(const Vec3& impulse);
    void ApplyAngularImpulse(const Vec3& impulse);
    Vec3 GetVelocityAtWorldPoint(const Vec3& point) const;
    void Integrate(float dt);

private:
    friend class World;
    friend class Island;
    friend class Contact;
    friend class ContactGraph;
    friend class BroadPhase;

    enum
    {
        flag_enabled = 1 << 0,
        flag_island = 1 << 1,
        flag_sleeping = 1 << 2,
    };

    World* world = nullptr;
    RigidBody* prev = nullptr;
    RigidBody* next = nullptr;

    ContactEdge* contactList = nullptr;
    int32 node = -1;

    int32 islandIndex = 0;
    int32 islandID = 0;
    uint16 flag = flag_enabled;
    float resting = 0.0f;
    bool ownShape = false;
};

inline bool RigidBody::IsStatic() const
{
    return invMass <= epsilon;
}

inline bool RigidBody::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline int32 RigidBody::GetIslandID() const
{
    return islandID;
}

inline int32 RigidBody::GetIslandIndex() const
{
    return islandIndex;
}

inline RigidBody* RigidBody::GetPrev()
{
    return prev;
}

inline const RigidBody* RigidBody::GetPrev() const
{
    return prev;
}

inline RigidBody* RigidBody::GetNext()
{
    return next;
}

inline const RigidBody* RigidBody::GetNext() const
{
    return next;
}

inline World* RigidBody::GetWorld()
{
    return world;
}

inline const World* RigidBody::GetWorld() const
{
    return world;
}

inline bool RigidBody::IsSleeping() const
{
    return (flag & flag_sleeping) == flag_sleeping;
}

inline void RigidBody::Awake()
{
    if (IsStatic())
    {
        return;
    }

    resting = 0.0f;
    flag &= ~flag_sleeping;
}

inline void RigidBody::Sleep()
{
    if (IsStatic())
    {
        return;
    }

    resting = max_float;
    force = Vec3::zero;
    torque = Vec3::zero;
    linearVelocity = Vec3::zero;
    angularVelocity = Vec3::zero;
    flag |= flag_sleeping;
}

inline float RigidBody::GetMass() const
{
    return IsStatic() ? 0.0f : 1.0f / invMass;
}

} // namespace muli3
