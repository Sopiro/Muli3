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
    Vec3 t1, t2;                       // Tangent frame
    Vec3 ra, rb;                       // Centered contact arms

    Vec3 wa1, wa2;                     // Cross(ra, tangent)
    Vec3 wb1, wb2;                     // Cross(rb, tangent)

    Mat2 linearMass;                   // Linear effective mass
    Vec2 bias;                         // Linear bias
    float angularMass;                 // Angular effective mass

    float da[max_contact_point_count]; // Friction lever arms
};

struct PositionConstraint
{
    Vec3 localPointA;
    Vec3 localPointB;
};

struct ContactConstraint
{
    NormalConstraint normalContact[max_contact_point_count];
    FrictionConstraint frictionContact;
    PositionConstraint positionContact[max_contact_point_count];
    Vec3 localNormal; // Shared position constraint normal
};

using ContactConstraintSet = GrowableStack<ContactConstraint, 1>;

void PrepareContact(ContactState* contact);
void WarmStartContact(ContactState* contact);
void SolveContactVelocityConstraints(ContactState* contact);
bool SolveContactPositionConstraints(ContactState* contact);

void PrepareJoint(JointState* joint, const Timestep& step);
void WarmStartJoint(JointState* joint);
void SolveJointVelocityConstraints(JointState* joint, const Timestep& step);
bool SolveJointPositionConstraints(JointState* joint, const Timestep& step);

} // namespace muli3
