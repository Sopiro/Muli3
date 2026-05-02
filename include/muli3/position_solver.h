#pragma once

#include "math.h"

namespace muli3
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* contact, int32 index);
    bool Solve(Contact* contact);

private:
    friend class Contact;

    Vec3 localPlanePoint;
    Vec3 localClipPoint;
    Vec3 localNormal;
};

} // namespace muli3
