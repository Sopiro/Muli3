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
    struct Jacobian
    {
        Vec3 va;   // -dir
        Vec3 wa;   // -Cross(ra, dir)
        Vec3 vb;   //  dir
        Vec3 wb;   //  Cross(rb, dir)
    } j;

    float m;       // effective mass
    float bias;
    float impulse; // impulse sum
};

struct SolverTangentContact
{
    struct Jacobian
    {
        Vec3 va;
        Vec3 wa;
        Vec3 vb;
        Vec3 wb;
    } j1, j2;

    Mat2 m;       // effective mass
    Vec2 bias;
    Vec2 impulse; // impulse sum
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
