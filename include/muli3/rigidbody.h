#pragma once

#include "settings.h"
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
    enum Type
    {
        static_body = 0,
        kinematic_body,
        dynamic_body,
    };

    RigidBody() = default;

    const Transform& GetTransform() const;
    void SetTransform(const Transform& transform);

    const Vec3& GetPosition() const;
    void SetPosition(const Vec3& position);
    void SetPosition(float x, float y, float z);

    const Quat& GetRotation() const;
    void SetRotation(const Quat& rotation);

    RigidBody::Type GetType() const;
    void SetType(RigidBody::Type type);

    bool IsStatic() const;
    void SetEnabled(bool enabled);
    bool IsEnabled() const;
    bool IsSleeping() const;
    void Awake();
    void Sleep();

    const Vec3& GetForce() const;
    void SetForce(const Vec3& force);
    const Vec3& GetTorque() const;
    void SetTorque(const Vec3& torque);

    const Vec3& GetLinearVelocity() const;
    void SetLinearVelocity(const Vec3& linearVelocity);
    void SetLinearVelocity(float vx, float vy, float vz);
    const Vec3& GetAngularVelocity() const;
    void SetAngularVelocity(const Vec3& angularVelocity);
    void SetAngularVelocity(float vx, float vy, float vz);

    void ApplyForce(const Vec3& worldPoint, const Vec3& force, bool awake);
    void ApplyTorque(const Vec3& torque, bool awake);

    Shape* CreateShape(Shape* shape, const Transform& transform = identity, float density = default_density);
    void DestroyShape();
    Shape* CreateSphereShape(float radius, const Transform& transform = identity, float density = default_density);
    Shape* GetShape();
    const Shape* GetShape() const;

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
    Mat3 GetInertiaTensorLocal() const;
    Mat3 GetInertiaTensorWorld() const;
    Mat3 GetInverseInertiaTensorLocal() const;
    Mat3 GetInverseInertiaTensorWorld() const;

    void SetMass(float mass);
    void ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse);
    void ApplyLinearImpulse(const Vec3& impulse);
    void ApplyAngularImpulse(const Vec3& impulse);
    Vec3 GetVelocityAtWorldPoint(const Vec3& point) const;
    void Integrate(float dt);

    Transform transform{};
    Transform transform0{};

    Vec3 linearVelocity{ 0.0f, 0.0f, 0.0f };
    Vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };

    float invMass = 0.0f;

    float restitution = 0.0f;
    float friction = 0.5f;

    Vec3 force{ 0.0f, 0.0f, 0.0f };
    Vec3 torque{ 0.0f, 0.0f, 0.0f };

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

    Type type = dynamic_body;

    World* world = nullptr;
    RigidBody* prev = nullptr;
    RigidBody* next = nullptr;

    Shape* shape = nullptr;
    ContactEdge* contactList = nullptr;
    int32 node = -1;

    int32 islandIndex = 0;
    int32 islandID = 0;
    uint16 flag = flag_enabled;
    float resting = 0.0f;
};

inline const Transform& RigidBody::GetTransform() const
{
    return transform;
}

inline const Vec3& RigidBody::GetPosition() const
{
    return transform.p;
}

inline void RigidBody::SetPosition(const Vec3& position)
{
    SetPosition(position.x, position.y, position.z);
}

inline const Quat& RigidBody::GetRotation() const
{
    return transform.q;
}

inline RigidBody::Type RigidBody::GetType() const
{
    return type;
}

inline bool RigidBody::IsStatic() const
{
    return type == static_body;
}

inline bool RigidBody::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline const Vec3& RigidBody::GetForce() const
{
    return force;
}

inline void RigidBody::SetForce(const Vec3& newForce)
{
    if (type != dynamic_body)
    {
        return;
    }

    force = newForce;
}

inline const Vec3& RigidBody::GetTorque() const
{
    return torque;
}

inline void RigidBody::SetTorque(const Vec3& newTorque)
{
    if (type != dynamic_body)
    {
        return;
    }

    torque = newTorque;
}

inline const Vec3& RigidBody::GetLinearVelocity() const
{
    return linearVelocity;
}

inline void RigidBody::SetLinearVelocity(const Vec3& newLinearVelocity)
{
    SetLinearVelocity(newLinearVelocity.x, newLinearVelocity.y, newLinearVelocity.z);
}

inline void RigidBody::SetLinearVelocity(float vx, float vy, float vz)
{
    if (IsStatic())
    {
        return;
    }

    linearVelocity = Vec3{ vx, vy, vz };
}

inline const Vec3& RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}

inline void RigidBody::SetAngularVelocity(const Vec3& newAngularVelocity)
{
    SetAngularVelocity(newAngularVelocity.x, newAngularVelocity.y, newAngularVelocity.z);
}

inline void RigidBody::SetAngularVelocity(float vx, float vy, float vz)
{
    if (IsStatic())
    {
        return;
    }

    angularVelocity = Vec3{ vx, vy, vz };
}

inline void RigidBody::ApplyForce(const Vec3& worldPoint, const Vec3& inForce, bool awake)
{
    if (type != dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        force += inForce;
        torque += Cross(worldPoint - GetWorldCenterOfMass(), inForce);
    }
}

inline void RigidBody::ApplyTorque(const Vec3& inTorque, bool awake)
{
    if (type != dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        torque += inTorque;
    }
}

inline Shape* RigidBody::GetShape()
{
    return shape;
}

inline const Shape* RigidBody::GetShape() const
{
    return shape;
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
    return invMass <= epsilon ? 0.0f : 1.0f / invMass;
}

} // namespace muli3
