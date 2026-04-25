#pragma once

#include "math.h"

namespace muli3
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* contact, int32 index);
    bool Solve();

private:
    friend class Contact;

    Contact* contact;

    Vec3 localPlanePoint;
    Vec3 localClipPoint;
    Vec3 localNormal;
};

} // namespace muli3
