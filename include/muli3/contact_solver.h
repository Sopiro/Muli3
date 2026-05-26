#pragma once

#include "math.h"

namespace muli3
{

class Contact;
struct ContactState;
struct JointState;
struct Timestep;

struct ContactJacobian
{
    Vec3 va; // -dir
    Vec3 wa; // -Cross(ra, dir)
    Vec3 vb; //  dir
    Vec3 wb; //  Cross(rb, dir)
};

struct SolverContact
{
    ContactJacobian j;

    // effective mass
    float m;
    float bias;

    // impulse sum
    float impulse = 0.0f;
    float impulseSave = 0.0f;
};

struct SolverPosition
{
    Vec3 localPlanePoint;
    Vec3 localClipPoint;
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
