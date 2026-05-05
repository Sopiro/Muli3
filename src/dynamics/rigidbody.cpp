#include "muli3/rigidbody.h"
#include "muli3/box.h"
#include "muli3/capsule.h"
#include "muli3/shape.h"
#include "muli3/sphere.h"
#include "muli3/world.h"

namespace muli3
{

RigidBody::RigidBody(const Transform& tf, RigidBody::Type type)
    : type{ type }
    , transform{ tf }
    , motion{ tf }
    , linearVelocity{ 0.0f, 0.0f, 0.0f }
    , angularVelocity{ 0.0f, 0.0f, 0.0f }
    , mass{ 0.0f }
    , invMass{ 0.0f }
    , inertia{ 0.0f }
    , invInertia{ 0.0f }
    , restitution{ default_restitution }
    , friction{ default_friction }
    , linearDamping{ default_linear_damping }
    , angularDamping{ default_angular_damping }
    , force{ 0.0f, 0.0f, 0.0f }
    , torque{ 0.0f, 0.0f, 0.0f }
    , islandIndex{ 0 }
    , islandID{ 0 }
    , flag{ flag_enabled }
    , world{ nullptr }
    , prev{ nullptr }
    , next{ nullptr }
    , shape{ nullptr }
    , shapeDensity{ default_density }
    , contactList{ nullptr }
    , node{ -1 }
    , resting{ 0.0f }
{
}

void RigidBody::SetTransform(const Transform& newTransform)
{
    Transform oldTransform = transform;
    transform = newTransform;
    motion.c = Mul(transform, motion.localCenter);
    motion.q = transform.q;
    motion.c0 = motion.c;
    motion.q0 = motion.q;
    motion.alpha0 = 0.0f;

    if (world != nullptr && IsEnabled())
    {
        world->contactGraph.UpdateBody(this, oldTransform, transform);
    }
}

void RigidBody::SetPosition(float x, float y, float z)
{
    Transform oldTransform = transform;
    transform.p = Vec3{ x, y, z };
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;
    motion.alpha0 = 0.0f;

    if (world != nullptr && IsEnabled())
    {
        world->contactGraph.UpdateBody(this, oldTransform, transform);
    }
}

void RigidBody::SetRotation(const Quat& rotation)
{
    Transform oldTransform = transform;
    transform.q = rotation;
    motion.q = transform.q;
    motion.q0 = motion.q;
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;
    motion.alpha0 = 0.0f;

    if (world != nullptr && IsEnabled())
    {
        world->contactGraph.UpdateBody(this, oldTransform, transform);
    }
}

void RigidBody::SetEnabled(bool enabled)
{
    if (enabled == IsEnabled())
    {
        return;
    }

    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return;
    }

    if (enabled)
    {
        flag |= flag_enabled;
        contactList = nullptr;
        world->contactGraph.AddBody(this);
    }
    else
    {
        flag &= ~flag_enabled;
        world->contactGraph.RemoveBody(this);
        islandID = 0;
        islandIndex = 0;
    }
}

void RigidBody::SetType(RigidBody::Type newType)
{
    if (type == newType)
    {
        return;
    }

    type = newType;
    ResetMassData();

    force = Vec3::zero;
    torque = Vec3::zero;

    if (type == static_body)
    {
        linearVelocity = Vec3::zero;
        angularVelocity = Vec3::zero;
        motion.c0 = motion.c;
        motion.q0 = motion.q;
    }

    Awake();

    if (shape != nullptr && world != nullptr)
    {
        world->contactGraph.RemoveBody(this);
        world->contactGraph.AddBody(this);
    }

    islandID = 0;
    islandIndex = 0;
}

Shape* RigidBody::CreateShape(Shape* newShape, const Transform& shapeTransform, float density)
{
    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return nullptr;
    }

    if (newShape == nullptr)
    {
        return nullptr;
    }

    DestroyShape();

    shapeDensity = density;
    shape = world->CloneShape(newShape, shapeTransform);
    ResetMassData();

    if (IsEnabled())
    {
        world->contactGraph.AddBody(this);
    }

    Awake();

    return shape;
}

void RigidBody::DestroyShape()
{
    if (shape == nullptr)
    {
        return;
    }

    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return;
    }

    world->contactGraph.RemoveBody(this);

    Shape* oldShape = shape;

    shape = nullptr;
    ResetMassData();

    world->FreeShape(oldShape);

    islandID = 0;
    islandIndex = 0;
    Awake();
}

Shape* RigidBody::CreateSphereShape(float radius, const Transform& shapeTransform, float density)
{
    Sphere sphere{ radius };
    return CreateShape(&sphere, shapeTransform, density);
}

Shape* RigidBody::CreateCapsuleShape(float height, float radius, const Transform& shapeTransform, float density)
{
    Capsule capsule{ height, radius };
    return CreateShape(&capsule, shapeTransform, density);
}

Shape* RigidBody::CreateBoxShape(
    float width, float height, float depth, const Transform& shapeTransform, float radius, float density
)
{
    Box box{ width, height, depth, radius };
    return CreateShape(&box, shapeTransform, density);
}

Shape* RigidBody::CreateBoxShape(const Vec3& size, const Transform& shapeTransform, float radius, float density)
{
    return CreateBoxShape(size.x, size.y, size.z, shapeTransform, radius, density);
}

Shape* RigidBody::CreateBoxShape(float size, const Transform& shapeTransform, float radius, float density)
{
    return CreateBoxShape(size, size, size, shapeTransform, radius, density);
}

void RigidBody::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
    if (type != dynamic_body)
    {
        return;
    }

    ApplyLinearImpulse(impulse);

    const Vec3 r = impulsePoint - motion.c;
    ApplyAngularImpulse(Cross(r, impulse));
}

void RigidBody::ApplyLinearImpulse(const Vec3& impulse)
{
    if (type != dynamic_body)
    {
        return;
    }

    linearVelocity += impulse * invMass;
}

void RigidBody::ApplyAngularImpulse(const Vec3& impulse)
{
    if (type != dynamic_body)
    {
        return;
    }

    angularVelocity += GetWorldInverseInertiaTensor() * impulse;
}

Vec3 RigidBody::GetVelocityAtWorldPoint(const Vec3& point) const
{
    return linearVelocity + Cross(angularVelocity, point - motion.c);
}

void RigidBody::Integrate(float dt)
{
    if (type == static_body)
    {
        return;
    }

    motion.c += linearVelocity * dt;

    Quat w{ angularVelocity, 0.0f };
    motion.q = motion.q + (w * motion.q) * dt * 0.5f;
    motion.q.Normalize();

    SynchronizeTransform();
}

void RigidBody::ResetMassData()
{
    mass = 0.0f;
    invMass = 0.0f;
    inertia = Mat3::zero;
    invInertia = Mat3::zero;

    if (type != dynamic_body)
    {
        return;
    }

    if (shape == nullptr)
    {
        return;
    }

    MassData massData;
    shape->ComputeMass(shapeDensity, &massData);

    mass = massData.mass;
    if (mass > 0.0f)
    {
        invMass = 1.0f / mass;
    }

    Vec3 oldCenter = motion.c;
    motion.localCenter = massData.centerOfMass;
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;
    motion.alpha0 = 0.0f;

    inertia = massData.inertia;
    const Vec3& c = motion.localCenter;

    inertia.ex -= Vec3{ mass * (c.y * c.y + c.z * c.z), -mass * c.x * c.y, -mass * c.x * c.z };
    inertia.ey -= Vec3{ -mass * c.y * c.x, mass * (c.x * c.x + c.z * c.z), -mass * c.y * c.z };
    inertia.ez -= Vec3{ -mass * c.z * c.x, -mass * c.z * c.y, mass * (c.x * c.x + c.y * c.y) };

    invInertia = inertia.GetInverse();

    linearVelocity += Cross(angularVelocity, motion.c - oldCenter);
}

} // namespace muli3
