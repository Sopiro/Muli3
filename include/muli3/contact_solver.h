#pragma once

#include "collision.h"

namespace muli3
{

class Contact;
struct ContactState;
struct JointState;
struct Timestep;

struct NormalConstraint
{
    Vec3 n;     // Normal
    Vec3 wa;    // Cross(ra, normal)
    Vec3 wb;    // Cross(rb, normal)

    float m;    // Effective mass
    float bias; // Bias
};

struct FrictionConstraint
{
    Vec3 t1, t2;     // Tangent frame
    Vec3 ra, rb;     // Centered contact arms

    Vec3 wa1, wa2;   // Cross(ra, tangent)
    Vec3 wb1, wb2;   // Cross(rb, tangent)

    Mat2 linearMass; // Linear effective mass
    Vec2 bias;       // Linear bias
    float twistMass; // Angular effective mass
};

struct PositionConstraint
{
    Vec3 localPointA;
    Vec3 localPointB;
    Vec3 localNormal;
};

struct ContactConstraint
{
    NormalConstraint normalContact[max_contact_point_count];
    FrictionConstraint frictionContact;
    PositionConstraint positionContact[max_contact_point_count];
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
