// IWYU pragma: always_keep
#pragma once

#include "joint.h"

namespace muli3
{

// Point-to-point constraint against a world-space target
// 3 DOF constraint (constrains 3 translational DOFs)
class GrabJoint : public Joint
{
public:
    GrabJoint(Body* body, const Vec3& anchor, const Vec3& target, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchor() const;

    const Vec3& GetTarget() const;
    void SetTarget(const Vec3& newTarget);

private:
    float frequency;
    float dampingRatio;

    Vec3 localAnchor;
    Vec3 target;

    Vec3 r;
    Mat3 m;

    Vec3 bias;
    Vec3 impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(const Vec3& lambda);
};

// Orientation constraint against a world-space target orientation
// 3 DOF constraint (constrains 3 rotational DOFs)
class FixedRotationJoint : public Joint
{
public:
    FixedRotationJoint(Body* body, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Quat& GetTargetOrientation() const;
    void SetTargetOrientation(const Quat& newTargetOrientation);

private:
    float frequency;
    float dampingRatio;

    Quat targetOrientation;

    Mat3 m;
    Vec3 bias;
    Vec3 impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(const Vec3& lambda);
};

// Point-to-point constraint: constrains two anchor points to coincide
// 3 DOF constraint (constrains 3 translational DOFs)
class BallSocketJoint : public Joint
{
public:
    BallSocketJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

private:
    float frequency;
    float dampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;

    Vec3 ra;
    Vec3 rb;
    Mat3 m;

    Vec3 bias;
    Vec3 impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(const Vec3& lambda);
};

// Cone limit constraint: constrains two axes to stay within a maximum swing angle
// 1 DOF angular limit constraint (does not constrain twist around the axis)
class ConeSwingJoint : public Joint
{
public:
    ConeSwingJoint(Body* bodyA, Body* bodyB, const Vec3& axis, float maxAngle, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;

    float GetJointAngle() const;
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

private:
    float frequency;
    float dampingRatio;

    Vec3 localAxisA;
    Vec3 localAxisB;
    float maxAngle;
    float currentAngle;

    Vec3 swingAxis;
    float m;
    float bias;
    float impulseSum;
    float beta;
    float gamma;
    int32 limitState;

    void ApplyImpulse(float lambda);
};

// Distance constraint: constrains the separation between two anchor points
// 1 DOF constraint (equal distance or min/max distance limit)
class DistanceJoint : public Joint
{
public:
    DistanceJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        float minLength,
        float maxLength,
        float frequency,
        float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

    float GetJointLength() const;
    void SetJointLength(float newLength);

    float GetJointMinLength() const;
    void SetJointMinLength(float newMinLength);
    float GetJointMaxLength() const;
    void SetJointMaxLength(float newMaxLength);

private:
    float frequency;
    float dampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;
    float minLength, maxLength;

    Vec3 ra;
    Vec3 rb;
    Vec3 d;
    float m;

    float bias;
    float impulseSum;
    float beta;
    float gamma;
    int32 limitState;

    void ApplyImpulse(float lambda);
};

// Line constraint: constrains one anchor point to slide along an axis
// 2 DOF constraint (constrains 2 translational DOFs perpendicular to the axis)
class LineJoint : public Joint
{
public:
    LineJoint(Body* bodyA, Body* bodyB, const Vec3& anchor, const Vec3& dir, float frequency, float dampingRatio);

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

private:
    float frequency;
    float dampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxis;

    Vec3 t1, t2; // Two perpendicular vectors to the sliding axis
    Vec3 sa1, sa2;
    Vec3 sb1, sb2;

    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(const Vec2& lambda);
};

// Prismatic(Slider) constraint: line constraint + relative orientation constraint
// 5 DOF constraint (constrains 2 translational DOFs perpendicular to the axis and 3 rotational DOFs)
class PrismaticJoint : public Joint
{
public:
    PrismaticJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        const Vec3& dir,
        float linearFrequency,
        float linearDampingRatio,
        float angularFrequency,
        float angularDampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetLinearFrequency() const;
    void SetLinearFrequency(float newFrequency);
    float GetLinearDampingRatio() const;
    void SetLinearDampingRatio(float newDampingRatio);
    float GetAngularFrequency() const;
    void SetAngularFrequency(float newFrequency);
    float GetAngularDampingRatio() const;
    void SetAngularDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    const Quat& GetOrientationOffset() const;

private:
    float linearFrequency;
    float linearDampingRatio;
    float angularFrequency;
    float angularDampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxis;
    Quat orientationOffset;

    // For linear part (2 DOF perpendicular to axis)
    Vec3 t1, t2;
    Vec3 sa1, sa2;
    Vec3 sb1, sb2;
    Mat2 linearM;
    Vec2 linearBias;
    Vec2 linearImpulseSum;
    float linearBeta;
    float linearGamma;

    // For angular part (3 DOF)
    Mat3 angularM;
    Vec3 angularBias;
    Vec3 angularImpulseSum;
    float angularBeta;
    float angularGamma;

    void ApplyImpulse(const Vec2& linearLambda, const Vec3& angularLambda);
};

// Rigid attachment constraint: BallSocketJoint + relative orientation constraint
// 6 DOF constraint (constrains 3 translational DOFs and 3 rotational DOFs)
class WeldJoint : public Joint
{
public:
    WeldJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        float linearFrequency,
        float linearDampingRatio,
        float angularFrequency,
        float angularDampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetLinearFrequency() const;
    void SetLinearFrequency(float newFrequency);
    float GetLinearDampingRatio() const;
    void SetLinearDampingRatio(float newDampingRatio);
    float GetAngularFrequency() const;
    void SetAngularFrequency(float newFrequency);
    float GetAngularDampingRatio() const;
    void SetAngularDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;

    const Quat& GetOrientationOffset() const;

private:
    float linearFrequency;
    float linearDampingRatio;
    float angularFrequency;
    float angularDampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;

    Quat orientationOffset;

    Vec3 ra;
    Vec3 rb;

    // Effective mass for linear part (3x3) and angular part (3x3) solved separately
    Mat3 linearM;
    Mat3 angularM;

    Vec3 linearBias;
    Vec3 angularBias;

    Vec3 linearImpulseSum;
    Vec3 angularImpulseSum;
    float linearBeta;
    float linearGamma;
    float angularBeta;
    float angularGamma;

    void ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda);
};

// Angular part of a revolute joint: keeps hinge axes aligned and controls twist around the axis
// 2 DOF angular alignment constraint + optional 1 DOF angular limit and motor constraint
class RevoluteAngleJoint : public Joint
{
public:
    RevoluteAngleJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float swingFrequency,
        float swingDampingRatio,
        float angleFrequency,
        float angleDampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetSwingFrequency() const;
    void SetSwingFrequency(float newFrequency);
    float GetSwingDampingRatio() const;
    void SetSwingDampingRatio(float newDampingRatio);
    float GetAngleFrequency() const;
    void SetAngleFrequency(float newFrequency);
    float GetAngleDampingRatio() const;
    void SetAngleDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;

    float GetJointAngleOffset() const;
    float GetJointAngle() const;
    void SetJointAngle(float newAngle);

    bool IsLimitEnabled() const;
    void SetLimitEnabled(bool enabled);
    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

    bool IsMotorEnabled() const;
    void SetMotorEnabled(bool enabled);
    float GetMotorSpeed() const;
    void SetMotorSpeed(float speed);
    float GetMaxMotorTorque() const;
    void SetMaxMotorTorque(float torque);

private:
    float swingFrequency;
    float swingDampingRatio;
    float angleFrequency;
    float angleDampingRatio;

    Vec3 localAxisA;
    Vec3 localAxisB;
    Vec3 localNormalAxisA;
    Vec3 localNormalAxisB;

    float angleOffset;
    float minAngle;
    float maxAngle;
    float currentAngle;
    bool limitEnabled;

    Vec3 swingAxis1;
    Vec3 swingAxis2;
    Mat2 swingM;
    Vec2 swingBias;
    Vec2 swingImpulseSum;
    float swingBeta;
    float swingGamma;

    Vec3 twistAxis;
    float angleM;
    float angleBias;
    float angleImpulseSum;
    float angleBeta;
    float angleGamma;
    int32 limitState;

    bool motorEnabled;
    float motorSpeed;
    float maxMotorTorque;
    float motorM;
    float motorImpulseSum;

    void ApplySwingImpulse(const Vec2& lambda);
    void ApplyTwistImpulse(float lambda);
};

// Twist limit constraint: constrains the relative angle around a shared reference axis
// 1 DOF angular limit constraint
class TwistAngleJoint : public Joint
{
public:
    TwistAngleJoint(
        Body* bodyA, Body* bodyB, const Vec3& axis, float minAngle, float maxAngle, float frequency, float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;

    float GetJointAngleOffset() const;
    float GetJointAngle() const;
    void SetJointAngle(float newAngle);

    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

private:
    float frequency;
    float dampingRatio;

    Vec3 localAxisA;
    Vec3 localAxisB;
    Vec3 localNormalAxisA;
    Vec3 localNormalAxisB;

    float angleOffset;
    float minAngle;
    float maxAngle;
    float currentAngle;

    Vec3 twistAxis;
    float angleM;
    float angleBias;
    float angleImpulseSum;
    float beta;
    float gamma;
    int32 limitState;

    void ApplyAngleImpulse(float lambda);
};

// Revolute(Hinge) constraint: BallSocketJoint + hinge axis alignment with twist control
// 5 DOF constraint + optional 1 DOF angular limit and motor constraint
class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        const Vec3& axis,
        float minAngle,
        float maxAngle,
        float linearFrequency,
        float linearDampingRatio,
        float swingFrequency,
        float swingDampingRatio,
        float angleFrequency,
        float angleDampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetLinearFrequency() const;
    void SetLinearFrequency(float newFrequency);
    float GetLinearDampingRatio() const;
    void SetLinearDampingRatio(float newDampingRatio);
    float GetSwingFrequency() const;
    void SetSwingFrequency(float newFrequency);
    float GetSwingDampingRatio() const;
    void SetSwingDampingRatio(float newDampingRatio);
    float GetAngleFrequency() const;
    void SetAngleFrequency(float newFrequency);
    float GetAngleDampingRatio() const;
    void SetAngleDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;
    const Vec3& GetLocalNormalAxisA() const;
    const Vec3& GetLocalNormalAxisB() const;

    float GetJointAngleOffset() const;
    float GetJointAngle() const;
    void SetJointAngle(float newAngle);

    bool IsLimitEnabled() const;
    void SetLimitEnabled(bool enabled);
    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

    bool IsMotorEnabled() const;
    void SetMotorEnabled(bool enabled);
    float GetMotorSpeed() const;
    void SetMotorSpeed(float speed);
    float GetMaxMotorTorque() const;
    void SetMaxMotorTorque(float torque);

private:
    float linearFrequency;
    float linearDampingRatio;
    float swingFrequency;
    float swingDampingRatio;
    float angleFrequency;
    float angleDampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxisA;
    Vec3 localAxisB;
    Vec3 localNormalAxisA;
    Vec3 localNormalAxisB;

    float angleOffset;
    float minAngle;
    float maxAngle;
    float currentAngle;
    bool limitEnabled;

    Vec3 ra;
    Vec3 rb;
    Mat3 linearM;
    Vec3 linearBias;
    Vec3 linearImpulseSum;
    float linearBeta;
    float linearGamma;

    Vec3 swingAxis1;
    Vec3 swingAxis2;
    Mat2 swingM;
    Vec2 swingBias;
    Vec2 swingImpulseSum;
    float swingBeta;
    float swingGamma;

    Vec3 twistAxis;
    float angleM;
    float angleBias;
    float angleImpulseSum;
    float angleBeta;
    float angleGamma;
    int32 limitState;

    bool motorEnabled;
    float motorSpeed;
    float maxMotorTorque;
    float motorM;
    float motorImpulseSum;

    void ApplyLinearImpulse(const Vec3& lambda);
    void ApplySwingImpulse(const Vec2& lambda);
    void ApplyTwistImpulse(float lambda);
};

// Angular part of a universal joint.
// Keeps body-fixed axes A and B perpendicular, leaving steering around A and spin around B free.
class UniversalAngleJoint : public Joint
{
public:
    UniversalAngleJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& axisA,
        const Vec3& axisB,
        float perpFrequency,
        float perpDampingRatio,
        float steerFrequency,
        float steerDampingRatio,
        float spinFrequency,
        float spinDampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetPerpendicularFrequency() const;
    void SetPerpendicularFrequency(float newFrequency);
    float GetPerpendicularDampingRatio() const;
    void SetPerpendicularDampingRatio(float newDampingRatio);

    float GetSteeringFrequency() const;
    void SetSteeringFrequency(float frequency);
    float GetSteeringDampingRatio() const;
    void SetSteeringDampingRatio(float dampingRatio);

    float GetSpinFrequency() const;
    void SetSpinFrequency(float newFrequency);
    float GetSpinDampingRatio() const;
    void SetSpinDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAxisA() const;
    const Vec3& GetLocalAxisB() const;
    const Vec3& GetLocalReferenceAxisA() const;
    const Vec3& GetLocalReferenceAxisB() const;

    float GetSteeringAngle() const;
    bool IsSteeringMotorEnabled() const;
    void SetSteeringMotorEnabled(bool enabled);
    float GetTargetSteeringAngle() const;
    void SetTargetSteeringAngle(float angle);
    float GetMaxSteeringTorque() const;
    void SetMaxSteeringTorque(float torque);

    bool IsSteeringLimitEnabled() const;
    void SetSteeringLimitEnabled(bool enabled);
    float GetSteeringMinAngle() const;
    void SetSteeringMinAngle(float angle);
    float GetSteeringMaxAngle() const;
    void SetSteeringMaxAngle(float angle);

    bool IsSpinMotorEnabled() const;
    void SetSpinMotorEnabled(bool enabled);
    float GetSpinAngle() const;
    float GetSpinSpeed() const;
    void SetSpinSpeed(float speed);
    float GetMaxSpinTorque() const;
    void SetMaxSpinTorque(float torque);

    bool IsSpinLimitEnabled() const;
    void SetSpinLimitEnabled(bool enabled);
    float GetSpinMinAngle() const;
    void SetSpinMinAngle(float angle);
    float GetSpinMaxAngle() const;
    void SetSpinMaxAngle(float angle);

private:
    float perpFrequency;
    float perpDampingRatio;
    float steerFrequency;
    float steerDampingRatio;
    float spinFrequency;
    float spinDampingRatio;

    Vec3 localAxisA;    // Steering axis fixed to body A
    Vec3 localAxisB;    // Spin axis fixed to body B
    Vec3 localRefAxisA; // Zero-steering direction fixed to body A
    Vec3 localRefAxisB; // Zero-spin direction fixed to body B

    // Perpendicular constraint removes rotation around cross(axisB, axisA).
    Vec3 perpAxis;
    float perpM;
    float perpBias;
    float perpImpulseSum;
    float perpBeta;
    float perpGamma;

    // Steering servo and limit act around the angular gradient of the measured steering angle.
    Vec3 steeringAxis;
    float steeringAngle;
    bool steeringMotorEnabled;
    float targetSteeringAngle;
    float maxSteeringTorque;
    float steeringM;
    float steeringBias;
    float steeringImpulseSum;
    float steeringBeta;
    float steeringGamma;

    bool steeringLimitEnabled;
    float steeringMinAngle;
    float steeringMaxAngle;
    float steeringLimitBias;
    float steeringLimitImpulseSum;
    int32 steeringLimitState;

    // Spin motor and limit act around body B's spin axis.
    Vec3 spinAxis;
    float spinAngle;
    bool spinMotorEnabled;
    float spinSpeed;
    float maxSpinTorque;
    float spinM;
    float spinImpulseSum;

    bool spinLimitEnabled;
    float spinMinAngle;
    float spinMaxAngle;
    float spinLimitM;
    float spinLimitBias;
    float spinLimitImpulseSum;
    float spinBeta;
    float spinGamma;
    int32 spinLimitState;

    void ApplyAngularImpulse(const Vec3& axis, float lambda);
};

// Servo constraint: drives the relative anchor positions and orientations toward target offsets
// Up to 6 DOF motorized constraint (3 translational DOFs and 3 rotational DOFs, limited by max force/torque)
class MotorJoint : public Joint
{
public:
    MotorJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchor,
        float maxJointForce,
        float maxJointTorque,
        float linearFrequency,
        float linearDampingRatio,
        float angularFrequency,
        float angularDampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetLinearFrequency() const;
    void SetLinearFrequency(float newFrequency);
    float GetLinearDampingRatio() const;
    void SetLinearDampingRatio(float newDampingRatio);
    float GetAngularFrequency() const;
    void SetAngularFrequency(float newFrequency);
    float GetAngularDampingRatio() const;
    void SetAngularDampingRatio(float newDampingRatio);

    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    float GetMaxForce() const;
    void SetMaxForce(float maxForce);
    float GetMaxTorque() const;
    void SetMaxTorque(float maxTorque);
    const Vec3& GetLinearOffset() const;
    void SetLinearOffset(const Vec3& linearOffset);
    const Vec3& GetAngularOffset() const;
    void SetAngularOffset(const Vec3& angularOffset);

private:
    float linearFrequency;
    float linearDampingRatio;
    float angularFrequency;
    float angularDampingRatio;

    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Quat orientationOffset;

    Vec3 linearOffset;
    Vec3 angularOffset;

    float maxForce;
    float maxTorque;

    Vec3 ra;
    Vec3 rb;
    Mat3 linearM;
    Mat3 angularM;

    Vec3 linearBias;
    Vec3 angularBias;

    Vec3 linearImpulseSum;
    Vec3 angularImpulseSum;
    float linearBeta;
    float linearGamma;
    float angularBeta;
    float angularGamma;

    void ApplyImpulse(const Vec3& linearLambda, const Vec3& angularLambda);
};

// Pulley constraint: constrains the combined rope length between two body anchors and two ground anchors
// 1 DOF constraint (constrains the scalar pulley length)
class PulleyJoint : public Joint
{
public:
    PulleyJoint(
        Body* bodyA,
        Body* bodyB,
        const Vec3& anchorA,
        const Vec3& anchorB,
        const Vec3& groundAnchorA,
        const Vec3& groundAnchorB,
        float ratio,
        float frequency,
        float dampingRatio
    );

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    float GetFrequency() const;
    void SetFrequency(float newFrequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float newDampingRatio);

    const Vec3& GetGroundAnchorA() const;
    const Vec3& GetGroundAnchorB() const;
    const Vec3& GetLocalAnchorA() const;
    const Vec3& GetLocalAnchorB() const;
    float GetPulleyLength() const;
    void SetPulleyLength(float newLength);

private:
    float frequency;
    float dampingRatio;

    Vec3 groundAnchorA;
    Vec3 groundAnchorB;
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    float length;
    float ratio;

    Vec3 ra;
    Vec3 rb;
    Vec3 ua;
    Vec3 ub;
    float m;

    float bias;
    float impulseSum;
    float beta;
    float gamma;

    void ApplyImpulse(float lambda);
};

inline void Joint::Prepare(const Timestep& step)
{
    Dispatch([&](auto joint) { joint->Prepare(step); });
}

inline void Joint::WarmStart()
{
    Dispatch([](auto joint) { joint->WarmStart(); });
}

inline void Joint::SolveVelocityConstraints(const Timestep& step)
{
    Dispatch([&](auto joint) { joint->SolveVelocityConstraints(step); });
}

} // namespace muli3
