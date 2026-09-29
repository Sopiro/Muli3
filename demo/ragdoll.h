#pragma once

#include "muli3/muli3.h"

namespace muli3
{
struct Bone
{
    int32 parentIndex;
    Body* body;
    Joint* joint;
};

struct Ragdoll
{
    enum
    {
        index_pelvis = 0,
        index_chest = 1,
        index_head = 2,
        index_upperRightArm = 3,
        index_lowerRightArm = 4,
        index_upperLeftArm = 5,
        index_lowerLeftArm = 6,
        index_upperRightLeg = 7,
        index_lowerRightLeg = 8,
        index_upperLeftLeg = 9,
        index_lowerLeftLeg = 10,
        bone_count = 11,
    };

    Bone bones[bone_count];
    float scale;
};

Ragdoll CreateRagdoll(World* world, const Transform& tf, float scale, int32 gruop, float density = default_density);
void DeleteRagdoll(World* world, const Ragdoll& ragdoll);

} // namespace muli3
