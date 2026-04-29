#pragma once

#include "bounding_box.h"

namespace muli3
{

inline constexpr float linear_slop = 0.01f;
inline constexpr float restitution_slop = 0.5f;

inline constexpr int32 gjk_max_iteration = 20;
inline constexpr float gjk_tolerance = epsilon;

inline constexpr int32 epa_max_iteration = 20;
inline constexpr float epa_tolerance = epsilon;

inline constexpr float contact_merge_threshold = linear_slop * 0.001f;

inline constexpr float position_correction = 0.2f;
inline constexpr float max_position_correction = 0.1f;
inline constexpr float position_solver_threshold = linear_slop * 2.5f;

inline constexpr float aabb_margin = 0.1f;
inline constexpr float aabb_multiplier = 2.0f;

inline constexpr float minimum_radius = linear_slop * 2.0f;
inline constexpr float default_radius = linear_slop * 2.5f;
inline constexpr float default_density = 1.0f;
inline constexpr float default_friction = 0.5f;
inline constexpr float default_restitution = 0.0f;

struct Timestep
{
    int32 velocity_iterations = 3;
    int32 position_iterations = 2;

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
