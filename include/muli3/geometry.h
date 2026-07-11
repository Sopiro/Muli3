#pragma once

#include "math.h"

namespace muli3
{

struct Face;

void ComputeConvexHull(
    std::span<const Vec3> points, std::vector<Vec3>* outVertices, std::vector<int32>* outIndices, std::vector<Face>* outFaces
);
void ComputeConvexHull(std::span<const Vec2> vertices, std::vector<Vec2>* outVertices);

} // namespace muli3
