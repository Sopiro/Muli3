#pragma once

#include "common.h"
#include "dynamic_dispatcher.h"
#include "rigidbody.h"

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
    class TwistAngleJoint,
    class BallSocketJoint,
    class DistanceJoint,
    class WeldJoint,
    class LineJoint,
    class PrismaticJoint,
    class PulleyJoint,
    class MotorJoint>;

struct JointEdge
{
    RigidBody* other;
    Joint* joint;
    JointEdge* prev;
    JointEdge* next;
};

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
    friend class World;

public:
    using Types = Joints;

    enum Type
    {
        grab_joint,
        fixed_rotation_joint,
        cone_swing_joint,
        revolute_joint,
        revolute_angle_joint,
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
    bool SolvePositionConstraints(const Timestep& step);

    RigidBody* GetBodyA() const;
    RigidBody* GetBodyB() const;

    float GetJointFrequency() const;
    void SetJointFrequency(float jointFrequency);
    float GetJointDampingRatio() const;
    void SetJointDampingRatio(float jointDampingRatio);
    float GetJointMass() const;
    void SetJointMass(float jointMass);

    void SetParameters(float jointFrequency, float jointDampingRatio, float jointMass);

    bool IsSolid() const;
    Joint::Type GetType() const;

    Joint* GetPrev();
    const Joint* GetPrev() const;
    Joint* GetNext();
    const Joint* GetNext() const;

    bool IsEnabled() const;

    JointDestroyCallback* OnDestroy;
    void* UserData;

protected:
    // clang-format off
    Joint(
        Joint::Type type,
        RigidBody* bodyA,
        RigidBody* bodyB,
        float jointFrequency,
        float jointDampingRatio,
        float jointMass
    );
    // clang-format on

    JointState* GetJointState();
    const JointState* GetJointState() const;
    void ComputeBetaAndGamma(const Timestep& step);

    RigidBody* bodyA;
    RigidBody* bodyB;

private:
    // Following parameters are used to soften the joint
    // Frequency values less than or equal to zero make joints rigid
    float jointFrequency;    // 0 < Frequency
    float jointDampingRatio; // 0 <= Damping ratio <= 1
    float jointMass;         // 0 < Mass

    Joint* prev;
    Joint* next;

    JointEdge nodeA;
    JointEdge nodeB;

    int32 setIndex;
    int32 localIndex;

    bool flagIsland;
};

inline float Joint::GetJointFrequency() const
{
    return jointFrequency;
}

inline void Joint::SetJointFrequency(float newJointFrequency)
{
    SetParameters(newJointFrequency, jointDampingRatio, jointMass);
}

inline float Joint::GetJointDampingRatio() const
{
    return jointDampingRatio;
}

inline void Joint::SetJointDampingRatio(float newJointDampingRatio)
{
    SetParameters(jointFrequency, newJointDampingRatio, jointMass);
}

inline float Joint::GetJointMass() const
{
    return jointMass;
}

inline void Joint::SetJointMass(float newJointMass)
{
    SetParameters(jointFrequency, jointDampingRatio, newJointMass);
}

inline bool Joint::IsSolid() const
{
    return jointFrequency <= 0.0f;
}

inline Joint::Type Joint::GetType() const
{
    return Joint::Type(type_index);
}

inline Joint* Joint::GetPrev()
{
    return prev;
}

inline const Joint* Joint::GetPrev() const
{
    return prev;
}

inline Joint* Joint::GetNext()
{
    return next;
}

inline const Joint* Joint::GetNext() const
{
    return next;
}

inline bool Joint::IsEnabled() const
{
    return bodyA->IsEnabled() || bodyB->IsEnabled();
}

inline RigidBody* Joint::GetBodyA() const
{
    return bodyA;
}

inline RigidBody* Joint::GetBodyB() const
{
    return bodyB;
}

} // namespace muli3
