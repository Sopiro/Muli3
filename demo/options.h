#pragma once

namespace muli3
{

enum BodyDrawMode
{
    body_draw_solid,
    body_draw_solid_wireframe,
    body_draw_depth_wireframe,
    body_draw_wireframe,
    body_draw_none,
    body_draw_mode_count,
};

struct DebugOptions
{
    bool pause = false;
    bool step = false;
    BodyDrawMode body_draw_mode = body_draw_solid;
    bool draw_joint = true;
    bool show_bvh = false;
    bool show_aabb = false;
    bool show_profiler = false;
    bool show_contact_point = false;
    bool show_contact_normal = false;
    bool overlay_contact_point = false;
    bool overlay_contact_normal = false;
    bool reset_camera = false;
    bool colorize_island = true;
};

namespace UserFlag
{

enum Flag : size_t
{
    hide_joint = 1 << 0,
};

inline void SetFlag(Joint* joint, Flag flag, bool enabled)
{
    if (enabled)
    {
        joint->UserData = (void*)((size_t)joint->UserData | flag);
    }
    else
    {
        joint->UserData = (void*)((size_t)joint->UserData & ~flag);
    }
}

inline bool IsEnabled(const Joint* joint, Flag flag)
{
    return ((size_t)joint->UserData & flag) == flag;
}

} // namespace UserFlag

} // namespace muli3
