#pragma once

#include "collision_filter.h"
#include "material.h"
#include "settings.h"
#include "solver_states.h"
#include "transform.h"

namespace muli3
{

class World;
class Collider;
class Shape;
class Contact;
class Joint;
struct ContactEdge;
struct JointEdge;

class BodyDestroyCallback;
class RayCastAnyCallback;
class RayCastClosestCallback;

class Body
{
public:
    enum Type
    {
        static_body = 0,
        kinematic_body,
        dynamic_body,
    };

    Body(const Transform& tf, Type type);
    ~Body();

    Body(const Body&) = delete;
    Body& operator=(const Body&) = delete;

    const Transform& GetTransform() const;
    void SetTransform(const Transform& transform);

    const Motion& GetMotion() const;
    const Vec3& GetLocalCenter() const;

    const Vec3& GetPosition() const;
    void SetPosition(const Vec3& position);
    void SetPosition(float x, float y, float z);

    const Quat& GetRotation() const;
    void SetRotation(const Quat& rotation);
    void SetRotation(float eulerX, float eulerY, float eulerZ);

    float GetMass() const;
    const Mat3& GetInertiaTensor() const;
    Mat3 GetInertiaTensorCenterOfMass() const;

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

    Type GetType() const;
    void SetType(Type type);

    void SetEnabled(bool enabled);
    bool IsEnabled() const;

    void SetSleeping(bool sleeping);
    bool IsSleeping() const;

    bool IsDynamic() const;
    bool IsKinematic() const;
    bool IsStatic() const;

    void Awake();
    void Sleep();

    int32 GetIslandIndex() const;

    Body* GetPrev();
    const Body* GetPrev() const;
    Body* GetNext();
    const Body* GetNext() const;
    World* GetWorld();
    const World* GetWorld() const;

    void SetCollisionFilter(const CollisionFilter& filter) const;
    void SetFriction(float friction) const;
    void SetRestitution(float restitution) const;
    void SetRestitutionThreshold(float threshold) const;
    void SetSurfaceSpeed(const Vec2& surfaceSpeed) const;

    bool TestPoint(const Vec3& q) const;
    Vec3 GetClosestPoint(const Vec3& q) const;
    void RayCastAny(const Vec3& from, const Vec3& to, RayCastAnyCallback* callback) const;
    bool RayCastClosest(const Vec3& from, const Vec3& to, RayCastClosestCallback* callback) const;

    void RayCastAny(
        const Vec3& from,
        const Vec3& to,
        std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec3& from,
        const Vec3& to,
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
    Collider* CreateTriangleCollider(
        const Vec3& a,
        const Vec3& b,
        const Vec3& c,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateTriangleCollider(
        const Vec3 vertices[3],
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateQuadCollider(
        const Vec3& a,
        const Vec3& b,
        const Vec3& c,
        const Vec3& d,
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateQuadCollider(
        const Vec3 vertices[4],
        const Transform& transform = identity,
        float radius = default_radius,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateHeightFieldCollider(
        int32 sampleCountX,
        int32 sampleCountZ,
        std::span<const float> heightSamples,
        float cellSizeX = 1.0f,
        float cellSizeZ = 1.0f,
        const Vec3& offset = Vec3{ 0.0f },
        int32 blockSize = 8,
        const Transform& transform = identity,
        const Material& material = default_material
    );

    BodyState* GetBodyState();
    const BodyState* GetBodyState() const;

    Mat3 GetWorldInertiaTensor() const;
    Mat3 GetWorldInverseInertiaTensor() const;
    Vec3 GetVelocityAtWorldSpace(const Vec3& point) const;

    BodyDestroyCallback* OnDestroy;
    void* UserData;

private:
    friend class World;
    friend class AABBTree;
    friend class BroadPhase;
    friend class Collider;
    friend class ConstraintGraph;
    friend class Contact;

    friend class Joint;
    friend class FixedRotationJoint;
    friend class ConeSwingJoint;
    friend class RevoluteJoint;
    friend class RevoluteAngleJoint;
    friend class TwistAngleJoint;
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

    void ResetMassData();
    void SynchronizeTransform();
    void SynchronizeColliders();

    World* world;

    Body* prev;
    Body* next;

    Collider* colliderList;
    int32 colliderCount;

    ContactEdge* contactList;
    JointEdge* jointList;

    Type type;
    Transform transform;

    float mass;
    Mat3 inertia;

    int32 setIndex;
    int32 localIndex;

    int32 islandIndex;
    uint32 usedColors;

    uint16 flag;
};

inline const Transform& Body::GetTransform() const
{
    return transform;
}

inline const Motion& Body::GetMotion() const
{
    return GetBodyState()->motion;
}

inline const Vec3& Body::GetLocalCenter() const
{
    return GetBodyState()->motion.localCenter;
}

inline const Vec3& Body::GetPosition() const
{
    return transform.p;
}

inline void Body::SetPosition(const Vec3& position)
{
    SetPosition(position.x, position.y, position.z);
}

inline const Quat& Body::GetRotation() const
{
    return transform.q;
}

inline float Body::GetMass() const
{
    return mass;
}

inline const Mat3& Body::GetInertiaTensor() const
{
    return inertia;
}

inline Mat3 Body::GetInertiaTensorCenterOfMass() const
{
    const BodyState* s = GetBodyState();
    const Vec3& c = s->motion.localCenter;

    return Mat3(
        inertia.ex + Vec3{ mass * (c.y * c.y + c.z * c.z), -mass * c.x * c.y, -mass * c.x * c.z },
        inertia.ey + Vec3{ -mass * c.y * c.x, mass * (c.x * c.x + c.z * c.z), -mass * c.y * c.z },
        inertia.ez + Vec3{ -mass * c.z * c.x, -mass * c.z * c.y, mass * (c.x * c.x + c.y * c.y) }
    );
}

inline float Body::GetLinearDamping() const
{
    return GetBodyState()->linearDamping;
}

inline void Body::SetLinearDamping(float newLinearDamping)
{
    GetBodyState()->linearDamping = newLinearDamping;
}

inline float Body::GetAngularDamping() const
{
    return GetBodyState()->angularDamping;
}

inline void Body::SetAngularDamping(float newAngularDamping)
{
    GetBodyState()->angularDamping = newAngularDamping;
}

inline void Body::SetGyroscopicTorqueEnabled(bool enabled)
{
    if (enabled)
    {
        flag |= flag_gyroscopic_torque;
    }
    else
    {
        flag &= ~flag_gyroscopic_torque;
    }
}

inline bool Body::GetGyroscopicTorqueEnabled() const
{
    return (flag & flag_gyroscopic_torque) == flag_gyroscopic_torque;
}

inline const Vec3& Body::GetForce() const
{
    return GetBodyState()->force;
}

inline void Body::SetForce(const Vec3& newForce)
{
    if (type != dynamic_body)
    {
        return;
    }

    GetBodyState()->force = newForce;
}

inline const Vec3& Body::GetTorque() const
{
    return GetBodyState()->torque;
}

inline void Body::SetTorque(const Vec3& newTorque)
{
    if (type != dynamic_body)
    {
        return;
    }

    GetBodyState()->torque = newTorque;
}

inline const Vec3& Body::GetLinearVelocity() const
{
    return GetBodyState()->linearVelocity;
}

inline void Body::SetLinearVelocity(const Vec3& newLinearVelocity)
{
    SetLinearVelocity(newLinearVelocity.x, newLinearVelocity.y, newLinearVelocity.z);
}

inline void Body::SetLinearVelocity(float vx, float vy, float vz)
{
    if (IsStatic())
    {
        return;
    }

    GetBodyState()->linearVelocity = Vec3{ vx, vy, vz };
}

inline const Vec3& Body::GetAngularVelocity() const
{
    return GetBodyState()->angularVelocity;
}

inline void Body::SetAngularVelocity(const Vec3& newAngularVelocity)
{
    SetAngularVelocity(newAngularVelocity.x, newAngularVelocity.y, newAngularVelocity.z);
}

inline void Body::SetAngularVelocity(float vx, float vy, float vz)
{
    if (IsStatic())
    {
        return;
    }

    GetBodyState()->angularVelocity = Vec3{ vx, vy, vz };
}

inline void Body::ApplyForce(const Vec3& worldPoint, const Vec3& inForce, bool awake)
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
        BodyState* s = GetBodyState();
        s->force += inForce;
        s->torque += Cross(worldPoint - s->motion.c, inForce);
    }
}

inline void Body::ApplyForceLocal(const Vec3& localPoint, const Vec3& inForce, bool awake)
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
        BodyState* s = GetBodyState();
        s->force += inForce;
        s->torque += Cross(localPoint - s->motion.localCenter, inForce);
    }
}

inline void Body::ApplyTorque(const Vec3& inTorque, bool awake)
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
        GetBodyState()->torque += inTorque;
    }
}

inline Body::Type Body::GetType() const
{
    return type;
}

inline bool Body::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline bool Body::IsDynamic() const
{
    return type == dynamic_body;
}

inline bool Body::IsKinematic() const
{
    return type == kinematic_body;
}

inline bool Body::IsStatic() const
{
    return type == static_body;
}

inline void Body::SetSleeping(bool sleeping)
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

inline int32 Body::GetIslandIndex() const
{
    return islandIndex;
}

inline Body* Body::GetPrev()
{
    return prev;
}

inline const Body* Body::GetPrev() const
{
    return prev;
}

inline Body* Body::GetNext()
{
    return next;
}

inline const Body* Body::GetNext() const
{
    return next;
}

inline World* Body::GetWorld()
{
    return world;
}

inline const World* Body::GetWorld() const
{
    return world;
}

inline bool Body::IsSleeping() const
{
    return (flag & flag_sleeping) == flag_sleeping;
}

inline Collider* Body::GetColliderList()
{
    return colliderList;
}

inline const Collider* Body::GetColliderList() const
{
    return colliderList;
}

inline int32 Body::GetColliderCount() const
{
    return colliderCount;
}

inline void Body::SynchronizeTransform()
{
    BodyState* s = GetBodyState();
    transform.q = s->motion.q;
    transform.p = s->motion.c - transform.q.Rotate(s->motion.localCenter);
    transform.s = Vec3(1);
}

inline Mat3 Body::GetWorldInertiaTensor() const
{
    const BodyState* s = GetBodyState();
    Mat3 rotation(s->motion.q);
    return rotation * inertia * rotation.GetTranspose();
}

inline Mat3 Body::GetWorldInverseInertiaTensor() const
{
    const BodyState* s = GetBodyState();
    Mat3 rotation(s->motion.q);
    return rotation * s->invInertia * rotation.GetTranspose();
}

} // namespace muli3
