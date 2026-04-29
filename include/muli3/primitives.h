#pragma once

#include "math.h"

namespace muli3
{

constexpr int32 max_face_vertices = 4;

struct Point
{
    Vec3 p;
    int32 id;
};

struct Face
{
    int32 count;
    Point points[max_face_vertices];
    Vec3 normal;
};

} // namespace muli3