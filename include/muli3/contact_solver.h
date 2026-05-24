#pragma once

#include "math.h"

namespace muli3
{

class Contact;
struct ContactState;
struct Timestep;

struct ContactJacobian
{
    Vec3 va; // -dir
    Vec3 wa; // -Cross(ra, dir)
    Vec3 vb; //  dir
    Vec3 wb; //  Cross(rb, dir)
};

class ContactSolverNormal
{
public:
    void Prepare(ContactState* s, int32 index, const Timestep& step);
    void Solve(ContactState* s);

private:
    friend class Contact;
    friend class ContactSolverTangent;
    friend struct ContactState;

    ContactJacobian j;

    float m;              // effective mass
    float bias;

    float impulse = 0.0f; // impulse sum
    float impulseSave = 0.0f;
};

class ContactSolverTangent
{
public:
    void Prepare(ContactState* s, const Vec3& tangent, uint8 tangentIndex, int32 index, const Timestep& step);
    void Solve(ContactState* s, const ContactSolverNormal* normalSolver);

private:
    friend class Contact;
    friend struct ContactState;

    ContactJacobian j;

    float m;              // effective mass
    float bias;

    float impulse = 0.0f; // impulse sum
    float impulseSave = 0.0f;
};

} // namespace muli3
