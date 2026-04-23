#pragma once

#include "bounding_box.h"

namespace muli3
{

inline constexpr float linear_slop = 0.004f;
inline constexpr float position_correction = 0.2f;
inline constexpr float max_position_correction = 0.1f;
inline constexpr float aabb_margin = 0.1f;
inline constexpr float aabb_multiplier = 2.0f;

struct Timestep
{
    int32 velocity_iterations = 3;
    int32 position_iterations = 3;

    float dt = 0.0f;
    float inv_dt = 0.0f;
};

struct WorldSettings
{
    bool apply_gravity = true;
    bool sleeping = true;
    Vec3 gravity{ 0.0f, -10.0f, 0.0f };

    AABB world_bounds{ Vec3{ -1e6f }, Vec3{ 1e6f } };
    float sleeping_time = 0.5f;
    float rest_linear_tolerance = 0.01f;
    float rest_angular_tolerance = 0.01f;

    mutable Timestep step;
};

} // namespace muli3
