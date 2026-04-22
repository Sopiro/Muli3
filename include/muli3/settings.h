#pragma once

#include "bounding_box.h"

namespace muli3
{

inline constexpr float linear_slop = 0.004f;
inline constexpr float position_correction = 0.2f;
inline constexpr float max_position_correction = 0.1f;

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
