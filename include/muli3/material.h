#pragma once

#include "math.h"
#include "settings.h"

namespace muli3
{

struct Material
{
    float friction = default_friction;
    float restitution = default_restitution;
    float restitutionThreshold = default_restitution_threshold;
    Vec2 surfaceSpeed = default_surface_speed;
};

constexpr inline Material default_material{};

inline float MixFriction(float frictionA, float frictionB)
{
    return SafeSqrt(frictionA * frictionB);
}

inline float MixRestitution(float restitutionA, float restitutionB)
{
    return Max(restitutionA, restitutionB);
}

inline float MixRestitutionThreshold(float thresholdA, float thresholdB)
{
    return Min(thresholdA, thresholdB);
}

} // namespace muli3
