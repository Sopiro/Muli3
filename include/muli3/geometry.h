#pragma once

#include "primitives.h"

namespace muli3
{

struct ConvexFace
{
    int32 count = 0;
    int32 indices[max_face_vertices]{};
};

void ComputeConvexHull(std::span<const Vec3> points, std::vector<Vec3>* outVertices, std::vector<ConvexFace>* outFaces);

} // namespace muli3
