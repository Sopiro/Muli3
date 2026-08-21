#include "muli3/constraint.h"
#include "muli3/settings.h"

#include "wide_math.h"

namespace muli3
{

struct VelocityW
{
    Vec3W linear;
    Vec3W angular;
};

struct PositionW
{
    Vec3W center;
    QuatW rotation;
};

struct BodyMassW
{
    FloatW invMass;
    Mat3W localInvInertia;
    Vec3W center;
    QuatW rotation;
    VelocityW velocity;
};

static Vec3W Gather(const Vec3 values[simd_width])
{
    Float x[simd_width], y[simd_width], z[simd_width];

    for (int32 lane = 0; lane < simd_width; ++lane)
    {
        x[lane] = values[lane].x;
        y[lane] = values[lane].y;
        z[lane] = values[lane].z;
    }

    return { LoadW(x), LoadW(y), LoadW(z) };
}

static QuatW Gather(const Quat values[simd_width])
{
    Float x[simd_width], y[simd_width], z[simd_width], w[simd_width];

    for (int32 lane = 0; lane < simd_width; ++lane)
    {
        x[lane] = values[lane].x;
        y[lane] = values[lane].y;
        z[lane] = values[lane].z;
        w[lane] = values[lane].w;
    }

    return { LoadW(x), LoadW(y), LoadW(z), LoadW(w) };
}

static Mat3W Gather(const Mat3 values[simd_width])
{
    Vec3 ex[simd_width], ey[simd_width], ez[simd_width];

    for (int32 i = 0; i < simd_width; ++i)
    {
        ex[i] = values[i].ex;
        ey[i] = values[i].ey;
        ez[i] = values[i].ez;
    }

    return { Gather(ex), Gather(ey), Gather(ez) };
}

static BodyMassW GatherBodyMass(BlockContactArray* contacts, int32 block, bool bodyA)
{
    const PtrBlock<BodyState>& bodies = bodyA ? contacts->constraint.bodyA[block] : contacts->constraint.bodyB[block];

    Float invMass[simd_width];
    Mat3 invInertia[simd_width];
    Vec3 center[simd_width];
    Quat rotation[simd_width];
    Vec3 linearVelocity[simd_width];
    Vec3 angularVelocity[simd_width];

    for (int32 lane = 0; lane < simd_width; ++lane)
    {
        const BodyState* body = bodies.lane[lane];
        invMass[lane] = body->invMass;
        invInertia[lane] = body->invInertia;
        center[lane] = body->motion.c;
        rotation[lane] = body->motion.q;
        linearVelocity[lane] = body->linearVelocity;
        angularVelocity[lane] = body->angularVelocity;
    }

    return {
        LoadW(invMass), Gather(invInertia), Gather(center), Gather(rotation), { Gather(linearVelocity), Gather(angularVelocity) },
    };
}

static VelocityW GatherVelocity(BlockContactArray* contacts, int32 block, bool bodyA)
{
    const PtrBlock<BodyState>& bodies = bodyA ? contacts->constraint.bodyA[block] : contacts->constraint.bodyB[block];

    Vec3 linear[simd_width];
    Vec3 angular[simd_width];

    for (int32 lane = 0; lane < simd_width; ++lane)
    {
        const BodyState* body = bodies.lane[lane];
        linear[lane] = body->linearVelocity;
        angular[lane] = body->angularVelocity;
    }

    return { Gather(linear), Gather(angular) };
}

static void ScatterVelocity(BlockContactArray* contacts, int32 block, bool bodyA, const VelocityW& velocity)
{
    const PtrBlock<BodyState>& bodies = bodyA ? contacts->constraint.bodyA[block] : contacts->constraint.bodyB[block];
    int32 laneCount = Min(simd_width, contacts->Count() - block * simd_width);

    FloatBlock linearX, linearY, linearZ;
    FloatBlock angularX, angularY, angularZ;

    StoreW(&linearX, velocity.linear.x);
    StoreW(&linearY, velocity.linear.y);
    StoreW(&linearZ, velocity.linear.z);
    StoreW(&angularX, velocity.angular.x);
    StoreW(&angularY, velocity.angular.y);
    StoreW(&angularZ, velocity.angular.z);

    for (int32 lane = 0; lane < laneCount; ++lane)
    {
        BodyState* body = bodies.lane[lane];
        if (body->invMass == 0.0f)
        {
            continue;
        }

        body->linearVelocity = { linearX.lane[lane], linearY.lane[lane], linearZ.lane[lane] };
        body->angularVelocity = { angularX.lane[lane], angularY.lane[lane], angularZ.lane[lane] };
    }
}

static PositionW GatherPosition(BlockContactArray* contacts, int32 block, bool bodyA)
{
    const PtrBlock<BodyState>& bodies = bodyA ? contacts->constraint.bodyA[block] : contacts->constraint.bodyB[block];

    Vec3 center[simd_width];
    Quat rotation[simd_width];

    for (int32 lane = 0; lane < simd_width; ++lane)
    {
        const BodyState* body = bodies.lane[lane];
        center[lane] = body->motion.c;
        rotation[lane] = body->motion.q;
    }

    return { Gather(center), Gather(rotation) };
}

static void ScatterPosition(BlockContactArray* contacts, int32 block, bool bodyA, const PositionW& position)
{
    const PtrBlock<BodyState>& bodies = bodyA ? contacts->constraint.bodyA[block] : contacts->constraint.bodyB[block];
    int32 laneCount = Min(simd_width, contacts->Count() - block * simd_width);

    Vec3Block center;
    QuatBlock rotation;
    StoreW(&center, position.center);
    StoreW(&rotation, position.rotation);

    for (int32 lane = 0; lane < laneCount; ++lane)
    {
        BodyState* body = bodies.lane[lane];
        if (body->invMass == 0.0f)
        {
            continue;
        }

        body->motion.c = { center.x.lane[lane], center.y.lane[lane], center.z.lane[lane] };
        body->motion.q = { rotation.x.lane[lane], rotation.y.lane[lane], rotation.z.lane[lane], rotation.w.lane[lane] };
    }
}

void PrepareContactBlock(BlockContactArray* contacts, SolverSet* solverSets, int32 block)
{
    BlockContactState& state = contacts->state;
    BlockContactConstraint& constraint = contacts->constraint;
    int32 laneCount = Min(simd_width, contacts->Count() - block * simd_width);

    for (int32 lane = 0; lane < laneCount; ++lane)
    {
        constraint.bodyA[block].lane[lane] =
            &solverSets[state.bodySetA[block].lane[lane]].bodyStates[state.bodyIndexA[block].lane[lane]];
        constraint.bodyB[block].lane[lane] =
            &solverSets[state.bodySetB[block].lane[lane]].bodyStates[state.bodyIndexB[block].lane[lane]];
    }
    for (int32 lane = laneCount; lane < simd_width; ++lane)
    {
        // Padding lanes are read during fixed-width gathers but never scattered.
        constraint.bodyA[block].lane[lane] = constraint.bodyA[block].lane[0];
        constraint.bodyB[block].lane[lane] = constraint.bodyB[block].lane[0];
    }

    BodyMassW bodyA = GatherBodyMass(contacts, block, true);
    BodyMassW bodyB = GatherBodyMass(contacts, block, false);

    Mat3W invIA = ComputeWorldInvInertia(bodyA.rotation, bodyA.localInvInertia);
    Mat3W invIB = ComputeWorldInvInertia(bodyB.rotation, bodyB.localInvInertia);

    Vec3W normal = LoadW(state.normal[block]);
    Vec3W tangent1, tangent2;
    CoordinateSystemW(normal, &tangent1, &tangent2);

    StoreW(&constraint.invMassA[block], bodyA.invMass);
    StoreW(&constraint.invMassB[block], bodyB.invMass);
    StoreW(&constraint.invIA[block], invIA);
    StoreW(&constraint.invIB[block], invIB);
    StoreW(&constraint.localInvIA[block], bodyA.localInvInertia);
    StoreW(&constraint.localInvIB[block], bodyB.localInvInertia);
    StoreW(&constraint.localNormal[block], RotateInv(bodyA.rotation, normal));
    StoreW(&constraint.tangent1[block], tangent1);
    StoreW(&constraint.tangent2[block], tangent2);

    FloatW pointCount = LoadW(state.pointCount[block]);
    FloatBlock invCount;
    for (int32 lane = 0; lane < simd_width; ++lane)
    {
        int32 count = int32(state.pointCount[block].lane[lane]);
        invCount.lane[lane] = count > 0 ? Float(1) / Float(count) : Float(0);
    }

    Vec3W ra = { ZeroW(), ZeroW(), ZeroW() };
    Vec3W rb = { ZeroW(), ZeroW(), ZeroW() };
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        FloatW pointMask = GreaterThanW(pointCount, SplatW(Float(i)));
        FloatW pointWeight = AndW(SplatW(1.0f), pointMask);
        Vec3W armA = LoadW(state.anchorA[i][block]) - bodyA.center;
        Vec3W armB = LoadW(state.anchorB[i][block]) - bodyB.center;
        ra = MulAdd(ra, pointWeight, armA);
        rb = MulAdd(rb, pointWeight, armB);
    }
    ra = LoadW(invCount) * ra;
    rb = LoadW(invCount) * rb;

    Vec3W wa1 = Cross(ra, tangent1);
    Vec3W wa2 = Cross(ra, tangent2);
    Vec3W wb1 = Cross(rb, tangent1);
    Vec3W wb2 = Cross(rb, tangent2);
    StoreW(&constraint.frictionWA1[block], wa1);
    StoreW(&constraint.frictionWA2[block], wa2);
    StoreW(&constraint.frictionWB1[block], wb1);
    StoreW(&constraint.frictionWB2[block], wb2);

    FloatW k11 = AddW(AddW(bodyA.invMass, Dot(wa1, Mul(invIA, wa1))), AddW(bodyB.invMass, Dot(wb1, Mul(invIB, wb1))));
    FloatW k12 = AddW(Dot(wa1, Mul(invIA, wa2)), Dot(wb1, Mul(invIB, wb2)));
    FloatW k22 = AddW(AddW(bodyA.invMass, Dot(wa2, Mul(invIA, wa2))), AddW(bodyB.invMass, Dot(wb2, Mul(invIB, wb2))));
    StoreW(&constraint.linearMass[block], InverseW(k11, k12, k22));
    StoreW(&constraint.tangentBias[block], -LoadW(state.surfaceSpeed[block]));

    FloatW zero = ZeroW();
    FloatW one = SplatW(1.0f);
    FloatW twistK = Dot(normal, Mul(invIA, normal) + Mul(invIB, normal));
    FloatW validTwist = GreaterThanW(twistK, zero);
    FloatW safeTwistK = BlendW(one, twistK, validTwist);
    StoreW(&constraint.angularMass[block], BlendW(zero, DivW(one, safeTwistK), validTwist));

    Vec3W oldLinearImpulse = LoadW(state.linearImpulse[block]);
    StoreW(&constraint.tangentImpulse[block], Vec2W{ Dot(oldLinearImpulse, tangent1), Dot(oldLinearImpulse, tangent2) });

    FloatW restitution = LoadW(state.restitution[block]);
    FloatW threshold = LoadW(state.restitutionThreshold[block]);
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        FloatW pointMask = GreaterThanW(pointCount, SplatW(Float(i)));
        Vec3W armA = LoadW(state.anchorA[i][block]) - bodyA.center;
        Vec3W armB = LoadW(state.anchorB[i][block]) - bodyB.center;
        Vec3W normalWA = Cross(armA, normal);
        Vec3W normalWB = Cross(armB, normal);
        FloatW k = AddW(
            AddW(bodyA.invMass, Dot(normalWA, Mul(invIA, normalWA))), AddW(bodyB.invMass, Dot(normalWB, Mul(invIB, normalWB)))
        );
        FloatW validMass = AndW(GreaterThanW(k, zero), pointMask);
        FloatW safeK = BlendW(one, k, validMass);
        FloatW mass = BlendW(zero, DivW(one, safeK), validMass);

        Vec3W relativeVelocity = (bodyB.velocity.linear + Cross(bodyB.velocity.angular, armB)) -
                                 (bodyA.velocity.linear + Cross(bodyA.velocity.angular, armA));
        FloatW normalVelocity = Dot(normal, relativeVelocity);
        FloatW bounce = AndW(GreaterThanW(NegW(normalVelocity), threshold), pointMask);
        FloatW bias = BlendW(zero, MulW(restitution, normalVelocity), bounce);

        StoreW(&constraint.normalWA[i][block], normalWA);
        StoreW(&constraint.normalWB[i][block], normalWB);
        StoreW(&constraint.normalMass[i][block], mass);
        StoreW(&constraint.normalBias[i][block], bias);
        StoreW(&constraint.localPointA[i][block], RotateInv(bodyA.rotation, armA));
        StoreW(&constraint.localPointB[i][block], RotateInv(bodyB.rotation, armB));

        Vec3W centeredArm = armA - ra;
        FloatW distance = SqrtW(Dot(centeredArm, centeredArm));
        StoreW(&constraint.leverArm[i][block], BlendW(zero, distance, pointMask));
    }
}

void WarmStartContactBlock(BlockContactArray* contacts, int32 block)
{
    BlockContactState& state = contacts->state;
    BlockContactConstraint& constraint = contacts->constraint;
    VelocityW bodyA = GatherVelocity(contacts, block, true);
    VelocityW bodyB = GatherVelocity(contacts, block, false);
    FloatW invMassA = LoadW(constraint.invMassA[block]);
    FloatW invMassB = LoadW(constraint.invMassB[block]);
    Mat3W invIA = LoadW(constraint.invIA[block]);
    Mat3W invIB = LoadW(constraint.invIB[block]);
    Vec3W normal = LoadW(state.normal[block]);

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        FloatW impulse = LoadW(state.normalImpulse[i][block]);
        Vec3W wa = LoadW(constraint.normalWA[i][block]);
        Vec3W wb = LoadW(constraint.normalWB[i][block]);
        bodyA.linear = MulSub(bodyA.linear, MulW(invMassA, impulse), normal);
        bodyA.angular = MulSub(bodyA.angular, invIA, impulse * wa);
        bodyB.linear = MulAdd(bodyB.linear, MulW(invMassB, impulse), normal);
        bodyB.angular = MulAdd(bodyB.angular, invIB, impulse * wb);
    }

    Vec3W tangent1 = LoadW(constraint.tangent1[block]);
    Vec3W tangent2 = LoadW(constraint.tangent2[block]);
    Vec3W wa1 = LoadW(constraint.frictionWA1[block]);
    Vec3W wa2 = LoadW(constraint.frictionWA2[block]);
    Vec3W wb1 = LoadW(constraint.frictionWB1[block]);
    Vec3W wb2 = LoadW(constraint.frictionWB2[block]);
    Vec2W tangentImpulse = LoadW(constraint.tangentImpulse[block]);
    FloatW angularImpulse = LoadW(state.angularImpulse[block]);
    Vec3W linearImpulse = tangentImpulse.x * tangent1 + tangentImpulse.y * tangent2;
    Vec3W angularImpulseA = tangentImpulse.x * wa1 + tangentImpulse.y * wa2 + angularImpulse * normal;
    Vec3W angularImpulseB = tangentImpulse.x * wb1 + tangentImpulse.y * wb2 + angularImpulse * normal;

    bodyA.linear = MulSub(bodyA.linear, invMassA, linearImpulse);
    bodyA.angular = MulSub(bodyA.angular, invIA, angularImpulseA);
    bodyB.linear = MulAdd(bodyB.linear, invMassB, linearImpulse);
    bodyB.angular = MulAdd(bodyB.angular, invIB, angularImpulseB);

    ScatterVelocity(contacts, block, true, bodyA);
    ScatterVelocity(contacts, block, false, bodyB);
}

void SolveContactVelocityBlock(BlockContactArray* contacts, int32 block)
{
    BlockContactState& state = contacts->state;
    BlockContactConstraint& constraint = contacts->constraint;

    VelocityW bodyA = GatherVelocity(contacts, block, true);
    VelocityW bodyB = GatherVelocity(contacts, block, false);

    FloatW invMassA = LoadW(constraint.invMassA[block]);
    FloatW invMassB = LoadW(constraint.invMassB[block]);

    Mat3W invIA = LoadW(constraint.invIA[block]);
    Mat3W invIB = LoadW(constraint.invIB[block]);

    Vec3W normal = LoadW(state.normal[block]);

    const FloatW zero = ZeroW();
    const FloatW one = SplatW(1.0f);

    FloatW totalNormalImpulse = zero;
    FloatW totalTwistLimit = zero;

    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        Vec3W wa = LoadW(constraint.normalWA[i][block]);
        Vec3W wb = LoadW(constraint.normalWB[i][block]);
        FloatW mass = LoadW(constraint.normalMass[i][block]);
        FloatW bias = LoadW(constraint.normalBias[i][block]);
        FloatW velocity = SubW(
            AddW(Dot(normal, bodyB.linear), Dot(wb, bodyB.angular)), AddW(Dot(normal, bodyA.linear), Dot(wa, bodyA.angular))
        );
        FloatW lambda = NegW(MulW(mass, AddW(velocity, bias)));
        FloatW oldImpulse = LoadW(state.normalImpulse[i][block]);
        FloatW impulse = MaxW(zero, AddW(oldImpulse, lambda));
        lambda = SubW(impulse, oldImpulse);
        StoreW(&state.normalImpulse[i][block], impulse);

        bodyA.linear = MulSub(bodyA.linear, MulW(invMassA, lambda), normal);
        bodyA.angular = MulSub(bodyA.angular, invIA, lambda * wa);
        bodyB.linear = MulAdd(bodyB.linear, MulW(invMassB, lambda), normal);
        bodyB.angular = MulAdd(bodyB.angular, invIB, lambda * wb);
        totalNormalImpulse = AddW(totalNormalImpulse, impulse);
        totalTwistLimit = AddW(totalTwistLimit, MulW(LoadW(constraint.leverArm[i][block]), impulse));
    }

    FloatW friction = LoadW(state.friction[block]);
    FloatW angularMass = LoadW(constraint.angularMass[block]);
    FloatW twistSpeed = Dot(normal, bodyB.angular - bodyA.angular);
    FloatW maxTwistFriction = MulW(friction, totalTwistLimit);
    FloatW twistLambda = NegW(MulW(angularMass, twistSpeed));
    FloatW oldAngularImpulse = LoadW(state.angularImpulse[block]);
    FloatW angularImpulse = ClampW(AddW(oldAngularImpulse, twistLambda), NegW(maxTwistFriction), maxTwistFriction);
    twistLambda = SubW(angularImpulse, oldAngularImpulse);
    StoreW(&state.angularImpulse[block], angularImpulse);
    Vec3W twistImpulse = twistLambda * normal;
    bodyA.angular = MulSub(bodyA.angular, invIA, twistImpulse);
    bodyB.angular = MulAdd(bodyB.angular, invIB, twistImpulse);

    Vec3W tangent1 = LoadW(constraint.tangent1[block]);
    Vec3W tangent2 = LoadW(constraint.tangent2[block]);
    Vec3W wa1 = LoadW(constraint.frictionWA1[block]);
    Vec3W wa2 = LoadW(constraint.frictionWA2[block]);
    Vec3W wb1 = LoadW(constraint.frictionWB1[block]);
    Vec3W wb2 = LoadW(constraint.frictionWB2[block]);
    FloatW tangentVelocity1 = SubW(
        AddW(Dot(tangent1, bodyB.linear), Dot(wb1, bodyB.angular)), AddW(Dot(tangent1, bodyA.linear), Dot(wa1, bodyA.angular))
    );
    FloatW tangentVelocity2 = SubW(
        AddW(Dot(tangent2, bodyB.linear), Dot(wb2, bodyB.angular)), AddW(Dot(tangent2, bodyA.linear), Dot(wa2, bodyA.angular))
    );
    Vec2W tangentBias = LoadW(constraint.tangentBias[block]);
    Vec2W tangentVelocity{ AddW(tangentVelocity1, tangentBias.x), AddW(tangentVelocity2, tangentBias.y) };
    Vec2W oldImpulse = LoadW(constraint.tangentImpulse[block]);
    Vec2W newImpulse = oldImpulse - Mul(LoadW(constraint.linearMass[block]), tangentVelocity);
    FloatW maxFriction = MulW(friction, totalNormalImpulse);
    FloatW impulse2 = Dot(newImpulse, newImpulse);
    FloatW clampMask = GreaterThanW(impulse2, MulW(maxFriction, maxFriction));
    FloatW length = SqrtW(impulse2);
    FloatW safeLength = BlendW(one, length, clampMask);
    FloatW scale = BlendW(one, DivW(maxFriction, safeLength), clampMask);
    newImpulse = scale * newImpulse;
    StoreW(&constraint.tangentImpulse[block], newImpulse);

    Vec2W deltaImpulse = newImpulse - oldImpulse;
    Vec3W linearImpulse = deltaImpulse.x * tangent1 + deltaImpulse.y * tangent2;
    Vec3W angularImpulseA = deltaImpulse.x * wa1 + deltaImpulse.y * wa2;
    Vec3W angularImpulseB = deltaImpulse.x * wb1 + deltaImpulse.y * wb2;
    bodyA.linear = MulSub(bodyA.linear, invMassA, linearImpulse);
    bodyA.angular = MulSub(bodyA.angular, invIA, angularImpulseA);
    bodyB.linear = MulAdd(bodyB.linear, invMassB, linearImpulse);
    bodyB.angular = MulAdd(bodyB.angular, invIB, angularImpulseB);

    StoreW(&state.linearImpulse[block], newImpulse.x * tangent1 + newImpulse.y * tangent2);
    ScatterVelocity(contacts, block, true, bodyA);
    ScatterVelocity(contacts, block, false, bodyB);
}

void SolveContactPositionBlock(BlockContactArray* contacts, int32 block)
{
    BlockContactState& state = contacts->state;
    BlockContactConstraint& constraint = contacts->constraint;

    PositionW bodyA = GatherPosition(contacts, block, true);
    PositionW bodyB = GatherPosition(contacts, block, false);

    FloatW invMassA = LoadW(constraint.invMassA[block]);
    FloatW invMassB = LoadW(constraint.invMassB[block]);

    Mat3W localInvIA = LoadW(constraint.localInvIA[block]);
    Mat3W localInvIB = LoadW(constraint.localInvIB[block]);

    Vec3W localNormal = LoadW(constraint.localNormal[block]);

    const FloatW zero = ZeroW();
    const FloatW one = SplatW(1.0f);
    FloatW pointCount = LoadW(state.pointCount[block]);

    FloatW dynamicA = GreaterThanW(invMassA, zero);
    FloatW dynamicB = GreaterThanW(invMassB, zero);
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        FloatW pointMask = GreaterThanW(pointCount, SplatW(Float(i)));
        Vec3W ra = Rotate(bodyA.rotation, LoadW(constraint.localPointA[i][block]));
        Vec3W rb = Rotate(bodyB.rotation, LoadW(constraint.localPointB[i][block]));
        Vec3W normal = Rotate(bodyA.rotation, localNormal);
        FloatW separation = Dot((bodyB.center - bodyA.center) + rb - ra, normal);
        FloatW penetration = AndW(LessThanW(separation, SplatW(-linear_slop)), pointMask);
        if (MoveMaskW(penetration) == 0)
        {
            continue;
        }

        Vec3W ran = Cross(ra, normal);
        Vec3W rbn = Cross(rb, normal);
        Vec3W localRan = RotateInv(bodyA.rotation, ran);
        Vec3W localRbn = RotateInv(bodyB.rotation, rbn);
        Vec3W angularA = Mul(localInvIA, localRan);
        Vec3W angularB = Mul(localInvIB, localRbn);
        FloatW k = AddW(AddW(invMassA, invMassB), AddW(Dot(localRan, angularA), Dot(localRbn, angularB)));
        FloatW correction = MinW(zero, MulW(SplatW(position_correction), AddW(separation, SplatW(linear_slop))));
        FloatW validMass = AndW(GreaterThanW(k, zero), pointMask);
        FloatW safeK = BlendW(one, k, validMass);
        FloatW lambda = BlendW(zero, DivW(NegW(correction), safeK), validMass);
        bodyA.center = MulSub(bodyA.center, MulW(invMassA, lambda), normal);
        bodyB.center = MulAdd(bodyB.center, MulW(invMassB, lambda), normal);

        FloatW correctionMask = GreaterThanW(lambda, zero);
        FloatW rotationMaskA = AndW(dynamicA, correctionMask);
        FloatW rotationMaskB = AndW(dynamicB, correctionMask);
        Vec3W angularCorrectionA = Rotate(bodyA.rotation, NegW(lambda) * angularA);
        Vec3W angularCorrectionB = Rotate(bodyB.rotation, lambda * angularB);
        QuatW deltaA{ angularCorrectionA.x, angularCorrectionA.y, angularCorrectionA.z, zero };
        QuatW deltaB{ angularCorrectionB.x, angularCorrectionB.y, angularCorrectionB.z, zero };
        QuatW rotationA = Normalize(bodyA.rotation + SplatW(0.5f) * (deltaA * bodyA.rotation));
        QuatW rotationB = Normalize(bodyB.rotation + SplatW(0.5f) * (deltaB * bodyB.rotation));
        bodyA.rotation = Blend(bodyA.rotation, rotationA, rotationMaskA);
        bodyB.rotation = Blend(bodyB.rotation, rotationB, rotationMaskB);
    }

    ScatterPosition(contacts, block, true, bodyA);
    ScatterPosition(contacts, block, false, bodyB);
}

} // namespace muli3
