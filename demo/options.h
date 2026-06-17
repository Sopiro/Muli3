#pragma once

namespace muli3
{

struct DebugOptions
{
    bool pause = false;
    bool step = false;
    bool draw_body = true;
    bool draw_joint = true;
    bool draw_outline = false;
    bool show_bvh = false;
    bool show_aabb = false;
    bool show_profiler = true;
    bool show_contact_point = false;
    bool show_contact_normal = false;
    bool reset_camera = false;
    bool colorize_island = true;
};

} // namespace muli3
