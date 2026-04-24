#pragma once

#include "bounding_box.h"

namespace muli3
{

inline constexpr float linear_slop = 0.005f;
inline constexpr float restitution_slop = 0.5f;

inline constexpr float baumgarte = 0.2f;

inline constexpr float aabb_margin = 0.1f;
inline constexpr float aabb_multiplier = 2.0f;

inline constexpr float default_density = 1.0f;

struct Timestep
{
    int32 velocity_iterations = 3;

    bool warm_starting = true;
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
    float rest_linear_tolerance = 0.01f * 0.01f;
    float rest_angular_tolerance = (0.5f * pi / 180.0f) * (0.5f * pi / 180.0f);

    mutable Timestep step;
};

} // namespace muli3
