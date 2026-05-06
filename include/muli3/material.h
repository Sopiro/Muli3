#pragma once

#include "math.h"
#include "settings.h"

namespace muli3
{

struct Material
{
    float friction = default_friction;
    float restitution = default_restitution;
    float restitutionTreshold = default_restitution_treshold;
    float surfaceSpeed = default_surface_speed;
};

constexpr Material default_material{};

inline float MixFriction(float frictionA, float frictionB)
{
    return SafeSqrt(frictionA * frictionB);
}

inline float MixRestitution(float restitutionA, float restitutionB)
{
    return Max(restitutionA, restitutionB);
}

inline float MixRestitutionTreshold(float tresholdA, float tresholdB)
{
    return Min(tresholdA, tresholdB);
}

} // namespace muli3
