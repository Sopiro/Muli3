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

    float GetFrequency() const;
    void SetFrequency(float frequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float dampingRatio);

    void SetParameters(float frequency, float dampingRatio);

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
    Joint(Joint::Type type, RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio);

    void ComputeBetaAndGamma(float effectiveMass, float dt);

    JointState* GetJointState();
    const JointState* GetJointState() const;

    RigidBody* bodyA;
    RigidBody* bodyB;

private:
    friend class World;
    friend class ConstraintGraph;

    // Following parameters are used to soften the joint
    // Frequency values less than or equal to zero make joints rigid
    float frequency;    // 0 < Frequency
    float dampingRatio; // 0 <= Damping Ratio <= 1

    Joint* prev;
    Joint* next;

    JointEdge nodeA;
    JointEdge nodeB;

    int32 setIndex;
    int32 colorIndex;
    int32 localIndex;

    bool flagIsland;
};

inline float Joint::GetFrequency() const
{
    return frequency;
}

inline void Joint::SetFrequency(float newJointFrequency)
{
    SetParameters(newJointFrequency, dampingRatio);
}

inline float Joint::GetDampingRatio() const
{
    return dampingRatio;
}

inline void Joint::SetDampingRatio(float newJointDampingRatio)
{
    SetParameters(frequency, newJointDampingRatio);
}

inline void Joint::SetParameters(float newFrequency, float newDampingRatio)
{
    if (newFrequency > 0.0f)
    {
        frequency = newFrequency;
        dampingRatio = std::clamp(newDampingRatio, 0.0f, 1.0f);
    }
    else
    {
        frequency = -1.0f;
        dampingRatio = 0.0f;
    }
}

inline bool Joint::IsSolid() const
{
    return frequency <= 0.0f;
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
