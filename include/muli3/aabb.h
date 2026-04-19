#pragma once

#include "math.h"

namespace muli3
{

struct AABB
{
    AABB()
        : min{ max_value }
        , max{ -max_value }
    {
    }

    constexpr AABB(const Vec3& min, const Vec3& max)
        : min{ min }
        , max{ max }
    {
    }

    Vec3& operator[](int32 i)
    {
        return i == 0 ? min : max;
    }

    const Vec3& operator[](int32 i) const
    {
        return i == 0 ? min : max;
    }

    Vec3 GetCenter() const
    {
        return (min + max) * 0.5f;
    }

    Vec3 GetExtents() const
    {
        return max - min;
    }

    float GetVolume() const
    {
        Vec3 e = GetExtents();
        return e.x * e.y * e.z;
    }

    float GetSurfaceArea() const
    {
        Vec3 e = GetExtents();
        return 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
    }

    bool Contains(const AABB& other) const
    {
        return min.x <= other.min.x && min.y <= other.min.y && min.z <= other.min.z && max.x >= other.max.x && max.y >= other.max.y &&
               max.z >= other.max.z;
    }

    bool TestPoint(const Vec3& point) const
    {
        if (min.x > point.x || max.x < point.x) return false;
        if (min.y > point.y || max.y < point.y) return false;
        if (min.z > point.z || max.z < point.z) return false;
        return true;
    }

    bool TestOverlap(const AABB& other) const
    {
        if (min.x > other.max.x || max.x < other.min.x) return false;
        if (min.y > other.max.y || max.y < other.min.y) return false;
        if (min.z > other.max.z || max.z < other.min.z) return false;
        return true;
    }

    static AABB Union(const AABB& a, const AABB& b)
    {
        return AABB{ Min(a.min, b.min), Max(a.max, b.max) };
    }

    static AABB Union(const AABB& aabb, const Vec3& point)
    {
        return AABB{ Min(aabb.min, point), Max(aabb.max, point) };
    }

    static AABB Intersection(const AABB& a, const AABB& b)
    {
        return AABB{ Max(a.min, b.min), Min(a.max, b.max) };
    }

    Vec3 min;
    Vec3 max;
};

} // namespace muli3
