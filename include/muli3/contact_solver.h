#pragma once

#include "math.h"

namespace muli3
{

class Contact;
struct Timestep;

struct ContactJacobian
{
    Vec3 va;  // -dir
    Vec3 wa;  // -Cross(ra, dir)
    Vec3 vb;  //  dir
    Vec3 wb;  //  Cross(rb, dir)
};

class ContactSolverNormal
{
public:
    void Prepare(Contact* contact, int32 index, const Timestep& step);
    void Solve(Contact* contact);

private:
    friend class Contact;
    friend class ContactSolverTangent;

    ContactJacobian j;

    float m;              // effective mass
    float bias;

    float impulse = 0.0f; // impulse sum
    float impulseSave = 0.0f;
};

class ContactSolverTangent
{
public:
    void Prepare(Contact* contact, const Vec3& tangent, int32 index, const Timestep& step);
    void Solve(Contact* contact, const ContactSolverNormal* normalSolver);

private:
    friend class Contact;

    ContactJacobian j;

    float m;              // effective mass
    float bias;

    float impulse = 0.0f; // impulse sum
    float impulseSave = 0.0f;
};

} // namespace muli3
