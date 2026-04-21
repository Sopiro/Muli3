#pragma once

#include "bounding_box.h"

namespace muli3
{

struct Timestep
{
    int32 velocity_iterations = 3;

    float dt = 0.0f;
    float inv_dt = 0.0f;
};

struct WorldSettings
{
    bool apply_gravity = true;
    Vec3 gravity{ 0.0f, -10.0f, 0.0f };

    AABB world_bounds{ Vec3{ -1e6f }, Vec3{ 1e6f } };

    mutable Timestep step;
};

} // namespace muli3
