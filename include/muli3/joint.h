#pragma once

#include "body.h"
#include "dynamic_dispatcher.h"

namespace muli3
{

class Joint;
class JointDestroyCallback;

using Joints = TypePack<
    class GrabJoint,
    class FixedRotationJoint,
    class ConeSwingJoint,
    class RevoluteJoint,
    class RevoluteAngleJoint,
    class UniversalAngleJoint,
    class TwistAngleJoint,
    class BallSocketJoint,
    class DistanceJoint,
    class WeldJoint,
    class LineJoint,
    class PrismaticJoint,
    class PulleyJoint,
    class MotorJoint>;

class Joint : public DynamicDispatcher<Joints>
{
    /*
     * Equation of motion for the damped harmonic oscillator
     * a = d²x/dt²
     * v = dx/dt
     *
     * ma + cv + kx = 0
     *
     * c = damping coefficient for springy motion
     * m = mass
     * k = spring constant
     *
     * a + 2ζωv + ω²x = 0
     *
     * ζ = damping ratio
     * ω = angular frequecy
     *
     * 2ζω = c / m
     * ω² = k / m
     *
     * Constraint equation
     * J·v + (β/h)·C(x) + (γ/h)·λ = 0
     *
     * h = dt
     * C(x) = Posiitonal error
     * λ = Corrective impulse
     *
     * β = hk / (c + hk)
     * γ = 1 / (c + hk)
     *
     * More reading:
     * https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
     * https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
     */

public:
    using Types = Joints;

    enum Type
    {
        // Order should match with Joints type pack
        grab_joint,
        fixed_rotation_joint,
        cone_swing_joint,
        revolute_joint,
        revolute_angle_joint,
        universal_angle_joint,
        twist_angle_joint,
        ball_socket_joint,
        distance_joint,
        weld_joint,
        line_joint,
        prismatic_joint,
        pulley_joint,
        motor_joint,
    };

    ~Joint();

    void Prepare(const Timestep& step);
    void WarmStart();
    void SolveVelocityConstraints(const Timestep& step);

    Body* GetBodyA() const;
    Body* GetBodyB() const;

    Type GetType() const;

    bool IsEnabled() const;

    JointDestroyCallback* OnDestroy;
    void* UserData;

protected:
    Joint(Type type, Body* bodyA, Body* bodyB);

    JointState* GetJointState();
    const JointState* GetJointState() const;

    Body* bodyA;
    Body* bodyB;

private:
    friend class World;
    friend class ConstraintGraph;

    int32 worldIndex;

    int32 bodyIndexA;
    int32 bodyIndexB;

    int32 setIndex;
    int32 colorIndex;
    int32 localIndex;

    bool flagIsland;
};

inline void ComputeBetaAndGamma(
    float* outBeta, float* outGamma, float frequency, float dampingRatio, float effectiveMass, float dt
)
{
    // The velocity solver uses K = J * M^-1 * J^T and solves
    // (K + gamma * I) * deltaLambda = -(J * V + beta / dt * C + gamma * accumulatedLambda).
    // beta and gamma are the implicit spring-damper coefficients obtained by
    // discretizing m * Cddot + d * Cdot + k * C = 0 over one time step.

    // If the frequency is less than or equal to zero, make this joint rigid
    if (frequency <= 0.0f || effectiveMass <= 0.0f)
    {
        *outBeta = 1.0f;
        *outGamma = 0.0f;
    }
    else
    {
        float omega = 2.0f * pi * frequency;
        float d = 2.0f * effectiveMass * dampingRatio * omega; // Damping coefficient
        float k = effectiveMass * omega * omega;               // Spring constant
        float h = dt;

        *outBeta = h * k / (d + h * k);
        *outGamma = 1.0f / ((d + h * k) * h);
    }
}

inline Joint::Type Joint::GetType() const
{
    return Joint::Type(type_index);
}

inline bool Joint::IsEnabled() const
{
    return bodyA->IsEnabled() && bodyB->IsEnabled();
}

inline Body* Joint::GetBodyA() const
{
    return bodyA;
}

inline Body* Joint::GetBodyB() const
{
    return bodyB;
}

} // namespace muli3
