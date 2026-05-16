#pragma once

#include "rigidbody.h"

namespace muli3
{

struct Timestep;

class Constraint
{
public:
    Constraint(RigidBody* bodyA, RigidBody* bodyB);
    virtual ~Constraint() = default;

    Constraint(const Constraint&) = delete;
    Constraint& operator=(const Constraint&) = delete;

    /*
     * C: Constraint equation
     * C = J * v = 0
     * J depends on constraint
     *
     * Compute Jacobian J and effective mass W
     * W = K^-1 = (J * M^-1 * J^t)^-1
     */
    virtual void Prepare(const Timestep& step) = 0;

    /*
     * Solve velocity constraint, calculate corrective impulse for current iteration
     * Pc: Corrective impulse
     * lambda: lagrangian multiplier
     *
     * Pc = J^t * lambda
     * lambda = (J * M^-1 * J^t)^-1 * -(Jv + (beta / h) * C(x)) where C(x): positional error
     *
     * with soft constraint,
     * lambda = (J * M^-1 * J^t + gamma * I)^-1 *
     *     -(Jv + (beta / h) * C(x) + (gamma / h) * lambda') where I = identity matrix
     *
     * More reading:
     * https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
     */
    virtual void SolveVelocityConstraints(const Timestep& step) = 0;
    virtual bool SolvePositionConstraints(const Timestep& step) = 0;

    RigidBody* GetBodyA() const;
    RigidBody* GetBodyB() const;

protected:
    RigidBody* bodyA;
    RigidBody* bodyB;

    Mat3 invIA;
    Mat3 invIB;

    float beta;
    float gamma;
};

inline RigidBody* Constraint::GetBodyA() const
{
    return bodyA;
}

inline RigidBody* Constraint::GetBodyB() const
{
    return bodyB;
}

} // namespace muli3
