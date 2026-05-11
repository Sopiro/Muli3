#pragma once

#include "collision_filter.h"
#include "material.h"
#include "settings.h"
#include "transform.h"

namespace muli3
{

class BodyDestroyCallback;
class Collider;
class RayCastAnyCallback;
class RayCastClosestCallback;
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
    ~RigidBody();

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    const Transform& GetTransform() const;
    void SetTransform(const Transform& transform);

    const Motion& GetMotion() const;
    const Vec3& GetLocalCenter() const;

    const Vec3& GetPosition() const;
    void SetPosition(const Vec3& position);
    void SetPosition(float x, float y, float z);

    const Quat& GetRotation() const;
    void SetRotation(const Quat& rotation);

    float GetMass() const;
    const Mat3& GetInertiaTensor() const;
    Mat3 GetInertiaTensorLocalOrigin() const;

    float GetLinearDamping() const;
    void SetLinearDamping(float linearDamping);
    float GetAngularDamping() const;
    void SetAngularDamping(float angularDamping);

    void SetGyroscopicTorqueEnabled(bool enabled);
    bool GetGyroscopicTorqueEnabled() const;

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
    void ApplyForceLocal(const Vec3& localPoint, const Vec3& force, bool awake);
    void ApplyTorque(const Vec3& torque, bool awake);

    void ApplyLinearImpulse(const Vec3& impulsePoint, const Vec3& impulse, bool awake);
    void ApplyLinearImpulseLocal(const Vec3& localPoint, const Vec3& impulse, bool awake);
    void ApplyAngularImpulse(const Vec3& impulse, bool awake);

    void Translate(const Vec3& delta);
    void Translate(float dx, float dy, float dz);
    void Rotate(const Quat& delta);
    void Rotate(const Vec3& eulerAngles);

    RigidBody::Type GetType() const;
    void SetType(RigidBody::Type type);

    void SetEnabled(bool enabled);
    bool IsEnabled() const;
    bool IsStatic() const;
    void SetSleeping(bool sleeping);
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

    void SetCollisionFilter(const CollisionFilter& filter) const;
    void SetFriction(float friction) const;
    void SetRestitution(float restitution) const;
    void SetRestitutionThreshold(float threshold) const;
    void SetSurfaceSpeed(const Vec2& surfaceSpeed) const;

    bool TestPoint(const Vec3& q) const;
    Vec3 GetClosestPoint(const Vec3& q) const;
    void RayCastAny(const Vec3& from, const Vec3& to, float radius, RayCastAnyCallback* callback) const;
    bool RayCastClosest(const Vec3& from, const Vec3& to, float radius, RayCastClosestCallback* callback) const;

    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        float radius,
        std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
        float radius,
        std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;

    Collider* CreateCollider(
        Shape* shape,
        const Transform& transform = identity,
        float density = default_density,
        const Material& material = default_material
    );
    void DestroyCollider(Collider* collider);

    int32 GetColliderCount() const;
    Collider* GetColliderList();
    const Collider* GetColliderList() const;

    Collider* CreateSphereCollider(
        float radius,
        const Transform& transform = identity,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateCapsuleCollider(
        float height,
        float radius,
        const Transform& transform = identity,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateCapsuleCollider(
        const Vec3& p1,
        const Vec3& p2,
        float radius,
        const Transform& transform = identity,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateBoxCollider(
        float width,
        float height,
        float depth,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateBoxCollider(
        const Vec3& size,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateBoxCollider(
        float size,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateConvexCollider(
        std::span<const Vec3> vertices,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );

    BodyDestroyCallback* OnDestroy;
    void* UserData;

    Mat3 GetWorldInertiaTensor() const;
    Mat3 GetWorldInverseInertiaTensor() const;
    Vec3 GetVelocityAtWorldPoint(const Vec3& point) const;

protected:
    friend class World;
    friend class Island;

    friend class AABBTree;
    friend class BroadPhase;
    friend class ContactGraph;

    friend class Collider;

    friend class Contact;
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
    friend class PositionSolver;

    friend class Joint;
    friend class Constraint;
    friend class FixedRotationJoint;
    friend class ConeSwingJoint;
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
        flag_gyroscopic_torque = 1 << 3,
    };

    Type type;

    Transform transform;
    Motion motion;

    Vec3 linearVelocity;
    Vec3 angularVelocity;

    float mass;
    float invMass;
    Mat3 inertia;
    Mat3 invInertia;

    float linearDamping;
    float angularDamping;

    Vec3 force;
    Vec3 torque;

    int32 islandIndex;
    int32 islandID;

    uint16 flag;

    void ResetMassData();
    void SynchronizeTransform();
    void SynchronizeColliders();

private:
    World* world;
    RigidBody* prev;
    RigidBody* next;

    Collider* colliderList;
    int32 colliderCount;

    ContactEdge* contactList;
    JointEdge* jointList;

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

inline void RigidBody::SetGyroscopicTorqueEnabled(bool enabled)
{
    if (enabled)
    {
        flag |= flag_gyroscopic_torque;
    }
    else
    {
        flag &= ~flag_gyroscopic_torque;
    }

    if (type != static_body)
    {
        Awake();
    }
}

inline bool RigidBody::GetGyroscopicTorqueEnabled() const
{
    return (flag & flag_gyroscopic_torque) == flag_gyroscopic_torque;
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
        torque += Cross(worldPoint - motion.c, inForce);
    }
}

inline void RigidBody::ApplyForceLocal(const Vec3& localPoint, const Vec3& inForce, bool awake)
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
        torque += Cross(localPoint - motion.localCenter, inForce);
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

inline RigidBody::Type RigidBody::GetType() const
{
    return type;
}

inline bool RigidBody::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline bool RigidBody::IsStatic() const
{
    return type == static_body;
}

inline void RigidBody::SetSleeping(bool sleeping)
{
    if (sleeping)
    {
        Sleep();
    }
    else
    {
        Awake();
    }
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

inline Collider* RigidBody::GetColliderList()
{
    return colliderList;
}

inline const Collider* RigidBody::GetColliderList() const
{
    return colliderList;
}

inline int32 RigidBody::GetColliderCount() const
{
    return colliderCount;
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
