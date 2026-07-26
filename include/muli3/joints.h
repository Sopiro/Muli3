#pragma once

#include "joint.h"

#include "ball_socket_joint.h"     // IWYU pragma: export
#include "cone_swing_joint.h"      // IWYU pragma: export
#include "distance_joint.h"        // IWYU pragma: export
#include "fixed_rotation_joint.h"  // IWYU pragma: export
#include "grab_joint.h"            // IWYU pragma: export
#include "line_joint.h"            // IWYU pragma: export
#include "motor_joint.h"           // IWYU pragma: export
#include "prismatic_joint.h"       // IWYU pragma: export
#include "pulley_joint.h"          // IWYU pragma: export
#include "revolute_angle_joint.h"  // IWYU pragma: export
#include "revolute_joint.h"        // IWYU pragma: export
#include "twist_angle_joint.h"     // IWYU pragma: export
#include "universal_angle_joint.h" // IWYU pragma: export
#include "weld_joint.h"            // IWYU pragma: export

namespace muli3
{

inline void Joint::Prepare(const Timestep& step)
{
    Dispatch([&](auto joint) { joint->Prepare(step); });
}

inline void Joint::WarmStart()
{
    Dispatch([](auto joint) { joint->WarmStart(); });
}

inline void Joint::SolveVelocityConstraints(const Timestep& step)
{
    Dispatch([&](auto joint) { joint->SolveVelocityConstraints(step); });
}

} // namespace muli3
