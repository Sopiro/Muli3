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
    , worldIndex{ null_index }
    , colliders{}
    , contacts{}
    , joints{}
    , type{ type }
    , transform{ tf }
    , mass{ 0.0f }
    , inertia{ 0.0f }
    , halfExtent{ 0.0f }
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
    SetRotation(Quat::FromEuler(x, y, z));
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

    MuliAssert(shape->GetRadius() >= 0.0f);

    Collider* collider = world->poolAllocator.New<Collider>();
    collider->Clone(this, shape, tf, density, material);

    collider->bodyIndex = int32(colliders.size());
    colliders.push_back(collider);

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
    MuliAssert(colliders.empty() == false);

    int32 index = collider->bodyIndex;
    MuliAssert(0 <= index && index < int32(colliders.size()));
    MuliAssert(colliders[index] == collider);

    Collider* moved = colliders.back();
    colliders[index] = moved;
    moved->bodyIndex = index;
    colliders.pop_back();

    world->constraintGraph.RemoveCollider(collider);
    Shape* shape = collider->shape;
    world->poolAllocator.Delete(collider);
    world->FreeShape(shape);

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

Collider* Body::CreateTriangleCollider(
    const Vec3& a, const Vec3& b, const Vec3& c, const Transform& tf, float radius, float density, const Material& material
)
{
    TriangleShape triangle{ a, b, c, radius };
    return CreateCollider(&triangle, tf, density, material);
}

Collider* Body::CreateTriangleCollider(
    const Vec3 vertices[3], const Transform& tf, float radius, float density, const Material& material
)
{
    return CreateTriangleCollider(vertices[0], vertices[1], vertices[2], tf, radius, density, material);
}

Collider* Body::CreatePolygonCollider(
    std::span<const Vec3> vertices, const Transform& tf, float radius, float density, const Material& material
)
{
    PolygonShape polygon{ vertices, radius };
    return CreateCollider(&polygon, tf, density, material);
}

Collider* Body::CreateHeightFieldCollider(
    int32 sampleCountX,
    int32 sampleCountZ,
    std::span<const float> heightSamples,
    float cellSizeX,
    float cellSizeZ,
    const Vec3& offset,
    int32 blockSize,
    const Transform& tf,
    const Material& material
)
{
    HeightFieldShape* heightField = world->poolAllocator.New<HeightFieldShape>(
        sampleCountX, sampleCountZ, heightSamples, cellSizeX, cellSizeZ, offset, blockSize, tf
    );

    Collider* collider = world->poolAllocator.New<Collider>();
    collider->Create(this, heightField, 0.0f, material);

    collider->bodyIndex = int32(colliders.size());
    colliders.push_back(collider);

    world->constraintGraph.AddCollider(collider);

    ResetMassData();

    return collider;
}

Collider* Body::CreateMeshCollider(
    std::span<const Vec3> vertices, std::span<const int32> indices, const Transform& tf, const Material& material
)
{
    MeshShape* mesh = world->poolAllocator.New<MeshShape>(vertices, indices, tf);

    Collider* collider = world->poolAllocator.New<Collider>();
    collider->Create(this, mesh, 0.0f, material);

    collider->bodyIndex = int32(colliders.size());
    colliders.push_back(collider);

    world->constraintGraph.AddCollider(collider);

    ResetMassData();

    return collider;
}

bool Body::TestPoint(const Vec3& q) const
{
    MuliAssert(colliders.empty() == false);

    for (Collider* collider : colliders)
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
    MuliAssert(colliders.empty() == false);

    Vec3 cp0 = colliders[0]->GetClosestPoint(q);
    if (cp0 == q)
    {
        return cp0;
    }

    float d0 = Dist2(cp0, q);

    for (size_t i = 1; i < colliders.size(); ++i)
    {
        Collider* collider = colliders[i];
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

void Body::RayCastAny(const Vec3& from, const Vec3& to, RayCastAnyCallback* callback) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;

    for (Collider* collider : colliders)
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

bool Body::RayCastClosest(const Vec3& from, const Vec3& to, RayCastClosestCallback* callback) const
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

    RayCastAny(from, to, &tempCallback);

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
    const Vec3& from, const Vec3& to, std::function<float(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;

    for (Collider* collider : colliders)
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
    const Vec3& from, const Vec3& to, std::function<void(Collider* collider, Vec3 point, Vec3 normal, float fraction)> callback
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

    RayCastAny(from, to, &tempCallback);

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

    while (contacts.empty() == false)
    {
        world->constraintGraph.Destroy(contacts.back());
    }

    bool dynamicTransition = IsDynamic() != (newType == dynamic_body);
    if (dynamicTransition)
    {
        // Recolor joints when the body starts or stops participating in graph coloring.
        for (Joint* joint : joints)
        {
            world->TransferJoint(joint, disabled_set);
        }
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

    SolverSetIndex newIndex = static_set;
    if (IsEnabled() == false)
    {
        newIndex = disabled_set;
    }
    else if (type != static_body)
    {
        newIndex = IsSleeping() ? sleeping_set : awake_set;
    }

    world->TransferBody(this, newIndex);

    for (Joint* joint : joints)
    {
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

    for (Collider* collider : colliders)
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

        SolverSetIndex newIndex = static_set;
        if (type != static_body)
        {
            newIndex = IsSleeping() ? sleeping_set : awake_set;
        }

        world->TransferBody(this, newIndex);

        for (Collider* collider : colliders)
        {
            if (collider->IsEnabled())
            {
                world->constraintGraph.AddCollider(collider);
            }
        }

        for (Joint* joint : joints)
        {
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

        while (contacts.empty() == false)
        {
            world->constraintGraph.Destroy(contacts.back());
        }

        for (Collider* collider : colliders)
        {
            world->constraintGraph.RemoveCollider(collider);
        }

        islandIndex = 0;

        for (Joint* joint : joints)
        {
            world->TransferJoint(joint, disabled_set);
        }

        world->TransferBody(this, disabled_set);
    }
}

void Body::SetCollisionFilter(const CollisionFilter& filter) const
{
    for (Collider* collider : colliders)
    {
        collider->SetFilter(filter);
    }
}

void Body::SetFriction(float friction) const
{
    for (Collider* collider : colliders)
    {
        collider->SetFriction(friction);
    }
}

void Body::SetRestitution(float restitution) const
{
    for (Collider* collider : colliders)
    {
        collider->SetRestitution(restitution);
    }
}

void Body::SetRestitutionThreshold(float threshold) const
{
    for (Collider* collider : colliders)
    {
        collider->SetRestitutionThreshold(threshold);
    }
}

void Body::SetSurfaceSpeed(const Vec2& surfaceSpeed) const
{
    for (Collider* collider : colliders)
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
    halfExtent = Vec3::zero;
    s->invMass = 0.0f;
    s->invInertia = Mat3::zero;

    if (colliders.empty())
    {
        return;
    }

    Vec3 localCenter = Vec3::zero;

    AABB localBounds;
    for (Collider* collider : colliders)
    {
        AABB bounds;
        collider->shape->ComputeAABB(identity, &bounds);
        localBounds = AABB::Union(localBounds, bounds);
    }

    halfExtent = Max(Abs(localBounds.min - localCenter), Abs(localBounds.max - localCenter));

    if (type != dynamic_body)
    {
        return;
    }

    for (Collider* collider : colliders)
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
        for (Collider* collider : colliders)
        {
            if (collider->IsEnabled())
            {
                world->constraintGraph.UpdateCollider(collider, transform);
            }
        }
    }
    else
    {
        BodyState* s = GetBodyState();
        Transform transform0;
        s->motion.GetTransform(0.0f, &transform0);

        for (Collider* collider : colliders)
        {
            if (collider->IsEnabled())
            {
                world->constraintGraph.UpdateCollider(collider, transform0, transform);
            }
        }
    }
}

} // namespace muli3
