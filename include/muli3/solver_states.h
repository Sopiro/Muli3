#pragma once

#include "collision.h"
#include "contact_solver.h"
#include "position_solver.h"
#include "transform.h"

namespace muli3
{

class Contact;
class Joint;
class RigidBody;
struct Timestep;

struct BodyState
{
    RigidBody* body;

    Transform transform;
    Motion motion;

    Vec3 linearVelocity;
    Vec3 angularVelocity;

    float mass;
    float invMass;
    Mat3 inertia;
    Mat3 invInertia;

    float linearDamping;
    float angularDamping;

    Vec3 force;
    Vec3 torque;

    float resting;
};

struct ContactState
{
    void Update();
    void Prepare(const Timestep& step);
    void SolveVelocityConstraints(const Timestep& step);
    bool SolvePositionConstraints(const Timestep& step);

    Contact* contact;

    BodyState* s1;
    BodyState* s2;

    ContactManifold manifold;

    ContactSolverNormal normalSolvers[max_contact_point_count];
    ContactSolverTangent tangent1Solvers[max_contact_point_count];
    ContactSolverTangent tangent2Solvers[max_contact_point_count];
    PositionSolver positionSolvers[max_contact_point_count];

    Vec3 cLinearImpulseA;
    Vec3 cLinearImpulseB;
    Vec3 cAngularImpulseA;
    Vec3 cAngularImpulseB;

    Mat3 invIA;
    Mat3 invIB;

    float friction;
    float restitution;
    float restitutionThreshold;
    Vec2 surfaceSpeed;
};

struct JointState
{
    void Prepare(const Timestep& step);
    void SolveVelocityConstraints(const Timestep& step);
    bool SolvePositionConstraints(const Timestep& step);

    Joint* joint;

    Mat3 invIA;
    Mat3 invIB;

    float beta;
    float gamma;
};

enum SolverSetIndex
{
    static_set = 0,
    disabled_set,
    awake_set,
    sleeping_set,
    solver_set_count,
};

struct SolverSet
{
    std::vector<BodyState> bodyStates;
    std::vector<ContactState> contactStates;
    std::vector<JointState> jointStates;
};

} // namespace muli3
