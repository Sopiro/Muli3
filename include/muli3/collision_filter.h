#pragma once

#include "types.h"

namespace muli3
{

struct CollisionFilter
{
    int32 group = 0;
    uint32 bit = 1;
    uint32 mask = 0xffffffff;
};

constexpr inline CollisionFilter default_collision_filter{};

inline bool EvaluateFilter(const CollisionFilter& filterA, const CollisionFilter& filterB)
{
    if (filterA.group == filterB.group && filterA.group != 0)
    {
        return filterA.group > 0;
    }

    return (filterA.mask & filterB.bit) != 0 && (filterB.mask & filterA.bit) != 0;
}

} // namespace muli3
