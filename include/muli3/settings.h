#pragma once

#include "bounding_box.h"

namespace muli3
{

class ThreadPool;

// Broad phase
inline constexpr float aabb_margin = 0.05f;
inline constexpr float aabb_multiplier = 4.0f;

// Narrow phase
inline constexpr int32 gjk_max_iteration = 20;
inline constexpr float gjk_tolerance = epsilon;
inline constexpr int32 epa_max_iteration = 20;
inline constexpr float epa_tolerance = epsilon;

// Collision tolerances
inline constexpr float linear_slop = 0.005f;
inline constexpr float angular_slop = 2.0f * pi / 180.0f;
inline constexpr float restitution_slop = 0.5f;

// Solver tolerances
inline constexpr float position_correction = 0.2f;
inline constexpr float position_solver_threshold = linear_slop * 3.0f;
inline constexpr float max_joint_angular_correction = 10.0f * pi / 180.0f;

// Defaults
inline constexpr float default_radius = linear_slop;
inline constexpr float default_density = 1.0f;
inline constexpr float default_friction = 0.5f;
inline constexpr float default_restitution = 0.0f;
inline constexpr float default_restitution_threshold = 2.0f;
inline constexpr Vec2 default_surface_speed = Vec2{ 0.0f };
inline constexpr float default_linear_damping = 0.0f;
inline constexpr float default_angular_damping = 0.0f;

struct Timestep
{
    uint64 index = 0;
    float dt = 0.0f;
    float inv_dt = 0.0f;
};

struct WorldSettings
{
    ThreadPool* thread_pool = nullptr;

    int32 velocity_iterations = 6;
    int32 position_iterations = 2;

    bool apply_gravity = true;
    Vec3 gravity{ 0.0f, -10.0f, 0.0f };
    AABB world_bounds{ Vec3{ -1e6f }, Vec3{ 1e6f } };

    bool sleeping = true;
    float sleeping_time = 0.5f;
    float rest_linear_tolerance = Sqr(0.03f);
    float rest_angular_tolerance = Sqr(1.0f * pi / 180.0f);
};

} // namespace muli3
