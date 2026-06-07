#include "muli3/body.h"
#include "muli3/callbacks.h"
#include "muli3/collider.h"
#include "muli3/shape.h"
#include "muli3/shapes.h"
#include "muli3/world.h"

namespace muli3
{

Body::Body(const Transform& tf, Type type)
    : OnDestroy{ nullptr }
    , UserData{ nullptr }
    , world{ nullptr }
    , prev{ nullptr }
    , next{ nullptr }
    , colliderList{ nullptr }
    , colliderCount{ 0 }
    , contactList{ nullptr }
    , jointList{ nullptr }
    , type{ type }
    , transform{ tf }
    , mass{ 0.0f }
    , inertia{ 0.0f }
    , setIndex{ null_index }
    , localIndex{ null_index }
    , islandIndex{ null_index }
    , usedColors{ 0 }
    , flag{ flag_enabled }
{
}

Body::~Body()
{
    if (OnDestroy)
    {
        OnDestroy->OnBodyDestroy(this);
    }

    world = nullptr;
}

BodyState* Body::GetBodyState()
{
    return &world->solverSets[setIndex].bodyStates[localIndex];
}

const BodyState* Body::GetBodyState() const
{
    return &world->solverSets[setIndex].bodyStates[localIndex];
}

void Body::Awake()
{
    world->WakeIsland(this);
}

void Body::Sleep()
{
    world->SleepBody(this);
}

void Body::SetTransform(const Transform& newTransform)
{
    transform = newTransform;

    BodyState* s = GetBodyState();
    s->motion.c = Mul(transform, s->motion.localCenter);
    s->motion.q = transform.q;
    s->motion.c0 = s->motion.c;
    s->motion.q0 = s->motion.q;
    s->motion.alpha0 = 0.0f;

    SynchronizeColliders();
}

void Body::SetPosition(float x, float y, float z)
{
    transform.p.Set(x, y, z);

    BodyState* s = GetBodyState();
    s->motion.c = Mul(transform, s->motion.localCenter);
    s->motion.c0 = s->motion.c;
    s->motion.alpha0 = 0.0f;

    SynchronizeColliders();
}

void Body::SetRotation(const Quat& rotation)
{
    transform.q = rotation;

    BodyState* s = GetBodyState();
    s->motion.q = transform.q;
    s->motion.q0 = s->motion.q;
    s->motion.c = Mul(transform, s->motion.localCenter);
    s->motion.c0 = s->motion.c;
    s->motion.alpha0 = 0.0f;

    SynchronizeColliders();
}

void Body::SetRotation(float x, float y, float z)
{
    SetRotation(Quat::FromEuler({ x, y, z }));
}

void Body::Translate(const Vec3& delta)
{
    Translate(delta.x, delta.y, delta.z);
}

void Body::Translate(float dx, float dy, float dz)
{
    transform.p += Vec3(dx, dy, dz);

    BodyState* s = GetBodyState();
    s->motion.c = Mul(transform, s->motion.localCenter);
    s->motion.c0 = s->motion.c;
    s->motion.alpha0 = 0.0f;

    SynchronizeColliders();
}

void Body::Rotate(const Quat& delta)
{
    transform.q = delta * transform.q;
    transform.q.Normalize();

    BodyState* s = GetBodyState();
    s->motion.q = transform.q;
    s->motion.q0 = s->motion.q;
    s->motion.c = Mul(transform, s->motion.localCenter);
    s->motion.c0 = s->motion.c;
    s->motion.alpha0 = 0.0f;

    SynchronizeColliders();
}

void Body::Rotate(const Vec3& eulerAngles)
{
    Rotate(Quat::FromEuler(eulerAngles));
}

Collider* Body::CreateCollider(Shape* shape, const Transform& tf, float density, const Material& material)
{
    MuliAssert(world != nullptr);
    if (world == nullptr || shape == nullptr)
    {
        return nullptr;
    }

    // Shape radius(skin) must be greater than or equal to linear_slop * 2.0 for stable CCD
    MuliAssert(shape->GetRadius() >= minimum_radius);

    Collider* collider = new (world->poolAllocator.Allocate<Collider>()) Collider;
    collider->Create(this, shape, tf, density, material);

    collider->next = colliderList;
    colliderList = collider;
    ++colliderCount;

    world->constraintGraph.AddCollider(collider);

    ResetMassData();

    return collider;
}

void Body::DestroyCollider(Collider* collider)
{
    if (collider == nullptr)
    {
        return;
    }

    MuliAssert(collider->body == this);
    MuliAssert(colliderCount > 0);

    Collider** c = &colliderList;
    while (*c)
    {
        if (*c == collider)
        {
            *c = collider->next;
            break;
        }

        c = &(*c)->next;
    }

    world->constraintGraph.RemoveCollider(collider);
    collider->~Collider();
    collider->Destroy(world);
    world->poolAllocator.Free(collider);

    --colliderCount;

    ResetMassData();
}

Collider* Body::CreateSphereCollider(float radius, const Transform& tf, float density, const Material& material)
{
    SphereShape sphere{ radius };
    return CreateCollider(&sphere, tf, density, material);
}

Collider* Body::CreateCapsuleCollider(float height, float radius, const Transform& tf, float density, const Material& material)
{
    CapsuleShape capsule{ height, radius };
    return CreateCollider(&capsule, tf, density, material);
}

Collider* Body::CreateCapsuleCollider(
    const Vec3& p1, const Vec3& p2, float radius, const Transform& tf, float density, const Material& material
)
{
    CapsuleShape capsule{ p1, p2, radius };
    return CreateCollider(&capsule, tf, density, material);
}

Collider* Body::CreateBoxCollider(
    float width, float height, float depth, const Transform& tf, float radius, float density, const Material& material
)
{
    BoxShape box{ width, height, depth, radius };
    return CreateCollider(&box, tf, density, material);
}

Collider* Body::CreateBoxCollider(const Vec3& size, const Transform& tf, float radius, float density, const Material& material)
{
    return CreateBoxCollider(size.x, size.y, size.z, tf, radius, density, material);
}

Collider* Body::CreateBoxCollider(float size, const Transform& tf, float radius, float density, const Material& material)
{
    return CreateBoxCollider(size, size, size, tf, radius, density, material);
}

Collider* Body::CreateConvexCollider(
    std::span<const Vec3> vertices, const Transform& tf, float radius, float density, const Material& material
)
{
    ConvexShape convex{ vertices, radius };
    return CreateCollider(&convex, tf, density, material);
}

bool Body::TestPoint(const Vec3& q) const
{
    MuliAssert(colliderCount > 0);

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        if (collider->TestPoint(q))
        {
            return true;
        }
    }

    return false;
}

Vec3 Body::GetClosestPoint(const Vec3& q) const
{
    MuliAssert(colliderCount > 0);

    Vec3 cp0 = colliderList->GetClosestPoint(q);
    if (cp0 == q)
    {
        return cp0;
    }

    float d0 = Dist2(cp0, q);

    for (Collider* collider = colliderList->next; collider; collider = collider->next)
    {
        Vec3 cp1 = collider->GetClosestPoint(q);
        if (cp1 == q)
        {
            return cp1;
        }

        float d1 = Dist2(cp1, q);
        if (d1 < d0)
        {
            cp0 = cp1;
            d0 = d1;
        }
    }

    return cp0;
}

void Body::RayCastAny(const Vec3& from, const Vec3& to, float radius, RayCastAnyCallback* callback) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.radius = radius;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        if (collider->RayCast(input, &output))
        {
            float fraction = output.fraction;
            Vec3 point = (1.0f - fraction) * input.from + fraction * input.to;
            input.maxFraction = callback->OnHitAny(collider, point, output.normal, fraction);
        }

        if (input.maxFraction <= 0.0f)
        {
            return;
        }
    }
}

bool Body::RayCastClosest(const Vec3& from, const Vec3& to, float radius, RayCastClosestCallback* callback) const
{
    struct TempCallback : RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider = nullptr;
        Vec3 closestPoint;
        Vec3 closestNormal;
        float closestFraction = 1.0f;

        float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float fraction) override
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;
            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void Body::RayCastAny(
    const Vec3& from,
    const Vec3& to,
    float radius,
    std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.radius = radius;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        if (collider->RayCast(input, &output))
        {
            float fraction = output.fraction;
            Vec3 point = (1.0f - fraction) * input.from + fraction * input.to;
            input.maxFraction = callback(collider, point, output.normal, fraction);
        }

        if (input.maxFraction <= 0.0f)
        {
            return;
        }
    }
}

bool Body::RayCastClosest(
    const Vec3& from,
    const Vec3& to,
    float radius,
    std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
) const
{
    struct TempCallback : RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider = nullptr;
        Vec3 closestPoint;
        Vec3 closestNormal;
        float closestFraction = 1.0f;

        float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float fraction) override
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;
            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void Body::SetType(Body::Type newType)
{
    if (type == newType)
    {
        return;
    }

    type = newType;
    flag &= ~flag_sleeping;
    ResetMassData();

    BodyState* s = GetBodyState();
    s->force = Vec3::zero;
    s->torque = Vec3::zero;

    if (type == static_body)
    {
        s->linearVelocity = Vec3::zero;
        s->angularVelocity = Vec3::zero;
        s->motion.c0 = s->motion.c;
        s->motion.q0 = s->motion.q;
        SynchronizeColliders();
    }

    SolverSetIndex setIndex = static_set;
    if (IsEnabled() == false)
    {
        setIndex = disabled_set;
    }
    else if (type != static_body)
    {
        setIndex = IsSleeping() ? sleeping_set : awake_set;
    }

    world->TransferBody(this, setIndex);

    for (JointEdge* je = jointList; je; je = je->next)
    {
        Joint* joint = je->joint;
        Body* bodyA = joint->GetBodyA();
        Body* bodyB = joint->GetBodyB();

        SolverSetIndex targetSet;
        if (bodyA->IsEnabled() == false || bodyB->IsEnabled() == false)
        {
            targetSet = disabled_set;
        }
        else if (bodyA->IsStatic() && bodyB->IsStatic())
        {
            targetSet = static_set;
        }
        else
        {
            bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
            bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
            targetSet = awakeA || awakeB ? awake_set : sleeping_set;
        }

        world->TransferJoint(joint, targetSet);
    }

    ContactEdge* ce = contactList;
    while (ce)
    {
        ContactEdge* ce0 = ce;
        ce = ce->next;
        world->constraintGraph.Destroy(ce0->contact);
    }
    contactList = nullptr;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        world->constraintGraph.broadPhase.Refresh(collider);
    }

    islandIndex = 0;
}

void Body::SetEnabled(bool enabled)
{
    if (enabled == IsEnabled())
    {
        return;
    }

    if (enabled)
    {
        flag |= flag_enabled;
        if (type != static_body)
        {
            flag &= ~flag_sleeping;
        }

        SolverSetIndex setIndex = static_set;
        if (type != static_body)
        {
            setIndex = IsSleeping() ? sleeping_set : awake_set;
        }

        world->TransferBody(this, setIndex);

        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->constraintGraph.AddCollider(collider);
        }

        for (JointEdge* je = jointList; je; je = je->next)
        {
            Joint* joint = je->joint;
            Body* bodyA = joint->GetBodyA();
            Body* bodyB = joint->GetBodyB();

            if (bodyA->IsEnabled() && bodyB->IsEnabled())
            {
                SolverSetIndex targetSet;
                if (bodyA->IsStatic() && bodyB->IsStatic())
                {
                    targetSet = static_set;
                }
                else
                {
                    bool awakeA = bodyA->IsStatic() == false && bodyA->IsSleeping() == false;
                    bool awakeB = bodyB->IsStatic() == false && bodyB->IsSleeping() == false;
                    targetSet = awakeA || awakeB ? awake_set : sleeping_set;
                }

                world->TransferJoint(joint, targetSet);
            }
        }
    }
    else
    {
        flag &= ~flag_enabled;

        ContactEdge* ce = contactList;
        while (ce)
        {
            ContactEdge* ce0 = ce;
            ce = ce->next;
            world->constraintGraph.Destroy(ce0->contact);
        }
        contactList = nullptr;

        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->constraintGraph.RemoveCollider(collider);
        }

        islandIndex = 0;

        for (JointEdge* je = jointList; je; je = je->next)
        {
            world->TransferJoint(je->joint, disabled_set);
        }

        world->TransferBody(this, disabled_set);
    }
}

void Body::SetCollisionFilter(const CollisionFilter& filter) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFilter(filter);
    }
}

void Body::SetFriction(float friction) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFriction(friction);
    }
}

void Body::SetRestitution(float restitution) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitution(restitution);
    }
}

void Body::SetRestitutionThreshold(float threshold) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitutionTreshold(threshold);
    }
}

void Body::SetSurfaceSpeed(const Vec2& surfaceSpeed) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetSurfaceSpeed(surfaceSpeed);
    }
}

void Body::ApplyLinearImpulse(const Vec3& impulsePoint, const Vec3& impulse, bool awake)
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
        s->linearVelocity += impulse * s->invMass;
        s->angularVelocity += GetWorldInverseInertiaTensor() * Cross(impulsePoint - s->motion.c, impulse);
    }
}

void Body::ApplyLinearImpulseLocal(const Vec3& localPoint, const Vec3& impulse, bool awake)
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
        s->linearVelocity += impulse * s->invMass;
        s->angularVelocity += GetWorldInverseInertiaTensor() * Cross(localPoint - s->motion.localCenter, impulse);
    }
}

void Body::ApplyAngularImpulse(const Vec3& impulse, bool awake)
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
        GetBodyState()->angularVelocity += GetWorldInverseInertiaTensor() * impulse;
    }
}

Vec3 Body::GetVelocityAtWorldSpace(const Vec3& point) const
{
    const BodyState* s = GetBodyState();
    return s->linearVelocity + Cross(s->angularVelocity, point - s->motion.c);
}

void Body::ResetMassData()
{
    BodyState* s = GetBodyState();

    mass = 0.0f;
    inertia = Mat3::zero;
    s->invMass = 0.0f;
    s->invInertia = Mat3::zero;

    if (type != dynamic_body)
    {
        return;
    }

    if (colliderCount <= 0)
    {
        return;
    }

    Vec3 localCenter = Vec3::zero;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        MassData massData = collider->GetMassData();
        mass += massData.mass;
        localCenter += massData.mass * massData.centerOfMass;
        inertia = inertia + massData.inertia;
    }

    if (mass > 0.0f)
    {
        s->invMass = 1.0f / mass;
        localCenter *= s->invMass;
    }

    if (mass > 0.0f)
    {
        const Vec3& c = localCenter;
        inertia.ex -= Vec3{ mass * (c.y * c.y + c.z * c.z), -mass * c.x * c.y, -mass * c.x * c.z };
        inertia.ey -= Vec3{ -mass * c.y * c.x, mass * (c.x * c.x + c.z * c.z), -mass * c.y * c.z };
        inertia.ez -= Vec3{ -mass * c.z * c.x, -mass * c.z * c.y, mass * (c.x * c.x + c.y * c.y) };
        s->invInertia = inertia.GetInverse();
    }

    Vec3 oldCenter = s->motion.c;
    s->motion.localCenter = localCenter;
    s->motion.c = Mul(transform, s->motion.localCenter);
    s->motion.c0 = s->motion.c;
    s->motion.alpha0 = 0.0f;

    s->linearVelocity += Cross(s->angularVelocity, s->motion.c - oldCenter);
}

void Body::SynchronizeColliders()
{
    if (world == nullptr || IsEnabled() == false)
    {
        return;
    }

    if (IsSleeping())
    {
        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->constraintGraph.UpdateCollider(collider, transform);
        }
    }
    else
    {
        BodyState* s = GetBodyState();
        Transform transform0;
        s->motion.GetTransform(0.0f, &transform0);

        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->constraintGraph.UpdateCollider(collider, transform0, transform);
        }
    }
}

} // namespace muli3
