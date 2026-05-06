#pragma once

#include "settings.h"
#include "transform.h"

namespace muli3
{

class Shape;
class World;
class Contact;
class Joint;
struct ContactEdge;
struct JointEdge;

class RigidBody
{
public:
    enum Type
    {
        static_body = 0,
        kinematic_body,
        dynamic_body,
    };

    RigidBody(const Transform& tf, RigidBody::Type type);

    const Transform& GetTransform() const;
    void SetTransform(const Transform& transform);

    const Motion& GetMotion() const;
    const Vec3& GetLocalCenter() const;

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
    float GetLinearDamping() const;
    void SetLinearDamping(float linearDamping);
    float GetAngularDamping() const;
    void SetAngularDamping(float angularDamping);

    void ApplyForce(const Vec3& worldPoint, const Vec3& force, bool awake);
    void ApplyTorque(const Vec3& torque, bool awake);

    Shape* CreateShape(Shape* shape, const Transform& transform = identity, float density = default_density);
    void DestroyShape();
    Shape* CreateSphereShape(float radius, const Transform& transform = identity, float density = default_density);
    Shape* CreateCapsuleShape(float height, float radius, const Transform& transform = identity, float density = default_density);
    Shape* CreateBoxShape(
        float width,
        float height,
        float depth,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density
    );
    Shape* CreateBoxShape(
        const Vec3& size, const Transform& transform = identity, float radius = default_radius, float density = default_density
    );
    Shape* CreateBoxShape(
        float size, const Transform& transform = identity, float radius = default_radius, float density = default_density
    );
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
    const Mat3& GetInertiaTensor() const;
    Mat3 GetInertiaTensorLocalOrigin() const;

    Mat3 GetWorldInertiaTensor() const;
    Mat3 GetWorldInverseInertiaTensor() const;

    void ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse);
    void ApplyLinearImpulse(const Vec3& impulse);
    void ApplyAngularImpulse(const Vec3& impulse);
    Vec3 GetVelocityAtWorldPoint(const Vec3& point) const;
    void Integrate(float dt);

protected:
    friend class World;
    friend class Island;

    friend class BroadPhase;
    friend class ContactGraph;

    friend class Contact;
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
    friend class PositionSolver;

    friend class Joint;
    friend class Constraint;
    friend class BallSocketJoint;
    friend class DistanceJoint;
    friend class GrabJoint;
    friend class WeldJoint;
    friend class LineJoint;
    friend class PrismaticJoint;
    friend class PulleyJoint;
    friend class MotorJoint;

    enum
    {
        flag_enabled = 1 << 0,
        flag_island = 1 << 1,
        flag_sleeping = 1 << 2,
    };

    Type type;

    Transform transform;
    Motion motion;

    Vec3 linearVelocity;
    Vec3 angularVelocity;

    float mass;
    float invMass;
    Mat3 inertia; // Inertia tensor calculated in local frame
    Mat3 invInertia;

    float restitution;
    float friction;
    float linearDamping;
    float angularDamping;

    Vec3 force;
    Vec3 torque;

    int32 islandIndex;
    int32 islandID;

    uint16 flag;

    void ResetMassData();
    void SynchronizeTransform();

private:
    friend class World;

    World* world;
    RigidBody* prev;
    RigidBody* next;

    Shape* shape;
    float shapeDensity;
    ContactEdge* contactList;
    JointEdge* jointList;
    int32 node;

    float resting;
};

inline const Transform& RigidBody::GetTransform() const
{
    return transform;
}

inline const Motion& RigidBody::GetMotion() const
{
    return motion;
}

inline const Vec3& RigidBody::GetLocalCenter() const
{
    return motion.localCenter;
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

inline float RigidBody::GetLinearDamping() const
{
    return linearDamping;
}

inline void RigidBody::SetLinearDamping(float newLinearDamping)
{
    linearDamping = newLinearDamping;
}

inline float RigidBody::GetAngularDamping() const
{
    return angularDamping;
}

inline void RigidBody::SetAngularDamping(float newAngularDamping)
{
    angularDamping = newAngularDamping;
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
        torque += Cross(worldPoint - motion.c, inForce);
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
    return mass;
}

inline const Mat3& RigidBody::GetInertiaTensor() const
{
    return inertia;
}

inline Mat3 RigidBody::GetInertiaTensorLocalOrigin() const
{
    const Vec3& c = motion.localCenter;

    return Mat3(
        inertia.ex + Vec3{ mass * (c.y * c.y + c.z * c.z), -mass * c.x * c.y, -mass * c.x * c.z },
        inertia.ey + Vec3{ -mass * c.y * c.x, mass * (c.x * c.x + c.z * c.z), -mass * c.y * c.z },
        inertia.ez + Vec3{ -mass * c.z * c.x, -mass * c.z * c.y, mass * (c.x * c.x + c.y * c.y) }
    );
}

inline void RigidBody::SynchronizeTransform()
{
    transform.q = motion.q;
    transform.p = motion.c - transform.q.Rotate(motion.localCenter);
    transform.s = Vec3{ 1.0f, 1.0f, 1.0f };
}

inline Mat3 RigidBody::GetWorldInertiaTensor() const
{
    Mat3 rotation{ motion.q };
    return rotation * inertia * rotation.GetTranspose();
}

inline Mat3 RigidBody::GetWorldInverseInertiaTensor() const
{
    Mat3 rotation{ motion.q };
    return rotation * invInertia * rotation.GetTranspose();
}

} // namespace muli3
