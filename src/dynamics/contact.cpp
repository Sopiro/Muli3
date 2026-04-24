#include "muli3/contact.h"
#include "muli3/settings.h"

namespace muli3
{

void Contact::Update()
{
    flag |= flag_enabled;

    if (!bodyA->shape || !bodyB->shape)
    {
        flag &= ~flag_touching;
        return;
    }

    bool touching = Collide(bodyA->shape, bodyA->transform, bodyB->shape, bodyB->transform, &manifold);
    if (touching)
    {
        flag |= flag_touching;

        if (manifold.featureFlipped)
        {
            b1 = bodyB;
            b2 = bodyA;
        }
        else
        {
            b1 = bodyA;
            b2 = bodyB;
        }
    }
    else
    {
        flag &= ~flag_touching;
    }
}

void Contact::Prepare(const Timestep& step)
{
    MuliNotUsed(step);

    friction = SafeSqrt(bodyA->friction * bodyB->friction);
    restitution = Min(bodyA->restitution, bodyB->restitution);
    restitutionThreshold = restitution_slop;
    surfaceSpeed = 0.0f;

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        normalImpulses[i] = 0.0f;
        tangentImpulses[i] = 0.0f;
    }
}

void Contact::SolveVelocityConstraints(const Timestep& step)
{
    float invMassSum = bodyA->invMass + bodyB->invMass;
    if (invMassSum <= epsilon)
    {
        return;
    }

    Vec3 centerOfMassA = bodyA->GetWorldCenterOfMass();
    Vec3 centerOfMassB = bodyB->GetWorldCenterOfMass();
    Mat3 inverseInertiaA = bodyA->GetInverseInertiaTensorWorld();
    Mat3 inverseInertiaB = bodyB->GetInverseInertiaTensorWorld();

    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        Vec3 point = manifold.contactPoints[i].p;
        Vec3 ra = point - centerOfMassA;
        Vec3 rb = point - centerOfMassB;

        Vec3 velocityA = bodyA->GetVelocityAtWorldPoint(point);
        Vec3 velocityB = bodyB->GetVelocityAtWorldPoint(point);
        Vec3 relativeVelocity = velocityB - velocityA;
        float velocityAlongNormal = Dot(relativeVelocity, manifold.contactNormal);

        float restitutionBias = restitution * Min(velocityAlongNormal + restitutionThreshold, 0.0f);
        float penetrationBias = -baumgarte * step.inv_dt * Max(manifold.penetrationDepth - linear_slop, 0.0f);
        float bias = restitutionBias + penetrationBias;

        Vec3 angularA = Cross(inverseInertiaA * Cross(ra, manifold.contactNormal), ra);
        Vec3 angularB = Cross(inverseInertiaB * Cross(rb, manifold.contactNormal), rb);
        float normalMass = invMassSum + Dot(angularA + angularB, manifold.contactNormal);
        if (normalMass <= epsilon)
        {
            continue;
        }

        float impulseMagnitude = -(velocityAlongNormal + bias) / normalMass;
        if (impulseMagnitude <= 0.0f)
        {
            continue;
        }

        normalImpulses[i] += impulseMagnitude;

        Vec3 impulse = manifold.contactNormal * impulseMagnitude;
        bodyA->ApplyImpulse(point, -impulse);
        bodyB->ApplyImpulse(point, impulse);

        Vec3 postVelocityA = bodyA->GetVelocityAtWorldPoint(point);
        Vec3 postVelocityB = bodyB->GetVelocityAtWorldPoint(point);
        Vec3 tangent =
            postVelocityB - postVelocityA - manifold.contactNormal * Dot(postVelocityB - postVelocityA, manifold.contactNormal);
        float tangentLength = Length(tangent);
        if (tangentLength <= epsilon)
        {
            continue;
        }

        tangent /= tangentLength;
        Vec3 tangentAngularA = Cross(inverseInertiaA * Cross(ra, tangent), ra);
        Vec3 tangentAngularB = Cross(inverseInertiaB * Cross(rb, tangent), rb);
        float tangentMass = invMassSum + Dot(tangentAngularA + tangentAngularB, tangent);
        if (tangentMass <= epsilon)
        {
            continue;
        }

        float frictionMagnitude = -Dot(postVelocityB - postVelocityA, tangent) / tangentMass;
        float frictionLimit = impulseMagnitude * friction;
        frictionMagnitude = Clamp(frictionMagnitude, -frictionLimit, frictionLimit);

        tangentImpulses[i] += frictionMagnitude;

        Vec3 frictionImpulse = tangent * frictionMagnitude;
        bodyA->ApplyImpulse(point, -frictionImpulse);
        bodyB->ApplyImpulse(point, frictionImpulse);
    }

    MuliNotUsed(step);
}

} // namespace muli3
