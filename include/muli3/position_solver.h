#pragma once

#include "math.h"

namespace muli3
{

struct ContactState;

class PositionSolver
{
public:
    void Prepare(ContactState* s, int32 index);
    bool Solve(ContactState* s);

private:
    friend class Contact;

    Vec3 localPlanePoint;
    Vec3 localClipPoint;
    Vec3 localNormal;
};

} // namespace muli3
