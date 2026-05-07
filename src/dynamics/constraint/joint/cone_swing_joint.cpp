#include "muli3/cone_swing_joint.h"
#include "muli3/frame.h"

namespace muli3
{

static constexpr float max_joint_angular_correction = 10.0f * pi / 180.0f;

ConeSwingJoint::ConeSwingJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec3& worldAxis,
    float jointMaxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(cone_swing_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , maxAngle{ Clamp(jointMaxAngle, 0.0f, pi) }
    , currentAngle{ 0.0f }
    , m{ 0.0f }
    , bias{ 0.0f }
    , impulseSum{ 0.0f }
    , activeLimit{ false }
{
    Vec3 axis = Length2(worldAxis) > epsilon ? Normalize(worldAxis) : y_axis;

    localAxisA = bodyA->GetRotation().RotateInv(axis);
    localAxisB = bodyB->GetRotation().RotateInv(axis);
}

void ConeSwingJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    Vec3 axisA = bodyA->GetRotation().Rotate(localAxisA);
    Vec3 axisB = bodyB->GetRotation().Rotate(localAxisB);

    float dot = Clamp(Dot(axisA, axisB), -1.0f, 1.0f);
    currentAngle = std::acos(dot);

    Vec3 axis = Cross(axisA, axisB);
    float axisLength = axis.Normalize();

    activeLimit = currentAngle > maxAngle;
    if (activeLimit == false)
    {
        bias = 0.0f;
        impulseSum = 0.0f;
        swingAxis = Vec3::zero;
        m = 0.0f;
        return;
    }

    // The swing correction acts around the axis perpendicular to both body axes.
    if (axisLength == 0.0f)
    {
        CoordinateSystem(axisA, &swingAxis);
    }
    else
    {
        swingAxis = axis;
    }

    Mat3 invIA = bodyA->GetWorldInverseInertiaTensor();
    Mat3 invIB = bodyB->GetWorldInverseInertiaTensor();

    float k = Dot(swingAxis, invIA * swingAxis) + Dot(swingAxis, invIB * swingAxis) + gamma;
    m = k != 0.0f ? 1.0f / k : 0.0f;
    float error = Min(currentAngle - maxAngle, max_joint_angular_correction);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void ConeSwingJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    if (activeLimit == false)
    {
        return;
    }

    float jv = Dot(swingAxis, bodyB->angularVelocity - bodyA->angularVelocity);
    float lambda = m * -(jv + bias + impulseSum * gamma);
    float newImpulseSum = Min(impulseSum + lambda, 0.0f);
    lambda = newImpulseSum - impulseSum;
    impulseSum = newImpulseSum;

    ApplyImpulse(lambda);
}

void ConeSwingJoint::ApplyImpulse(float lambda)
{
    Vec3 p = swingAxis * lambda;

    bodyA->angularVelocity -= bodyA->GetWorldInverseInertiaTensor() * p;
    bodyB->angularVelocity += bodyB->GetWorldInverseInertiaTensor() * p;
}

} // namespace muli3
