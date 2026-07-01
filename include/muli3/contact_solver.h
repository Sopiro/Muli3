#pragma once

#include "math.h"

namespace muli3
{

class Contact;
struct ContactState;
struct JointState;
struct Timestep;

struct SolverNormalContact
{
    Vec3 n;        // normal
    Vec3 wa;       // Cross(ra, normal)
    Vec3 wb;       // Cross(rb, normal)

    float m;       // Effective mass
    float bias;    // Bias
    float impulse; // Impulse sum
};

struct SolverTangentContact
{
    Vec3 t1, t2;   // tangent
    Vec3 wa1, wa2; // Cross(ra, tangent)
    Vec3 wb1, wb2; // Cross(rb, tangent)

    Mat2 m;        // Effective mass
    Vec2 bias;     // Bias
    Vec2 impulse;  // Impulse sum
};

struct SolverPosition
{
    Vec3 localPointA;
    Vec3 localPointB;
    Vec3 localNormal;
};

void PrepareContact(ContactState* s);
void WarmStartContact(ContactState* s);
void SolveContactVelocityConstraints(ContactState* s);
bool SolveContactPositionConstraints(ContactState* s);

void PrepareJoint(JointState* s, const Timestep& step);
void WarmStartJoint(JointState* s);
void SolveJointVelocityConstraints(JointState* s, const Timestep& step);
bool SolveJointPositionConstraints(JointState* s, const Timestep& step);

} // namespace muli3
