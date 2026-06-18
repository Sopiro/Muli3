#pragma once

#include "options.h"

namespace muli3
{
struct Bone
{
    int32 parentIndex;
    Body* body;
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

inline Ragdoll CreateRagdoll(World* world, Vec3 headPosition, float scale, int32 gruop, float density = default_density)
{
    Ragdoll ragdoll;
    ragdoll.scale = scale;

    CollisionFilter filter;
    filter.group = -gruop;

    float linearDamping = 0.0f;
    float angularDamping = 1.0f;

    float headX = headPosition.x;
    float headY = headPosition.y;

    float headSize = 0.15f * scale;
    float headRadius = 0.2f * scale;

    // Head
    Body* head = world->CreateCapsule(headSize, headRadius, identity, Body::dynamic_body, density);
    head->SetPosition(headPosition);

    float bodyWidth = 0.5f * scale;
    float bodyHeight = 0.85f * scale;
    float neckGap = 0.05f * scale;

    Body* chest = world->CreateCapsule(bodyHeight / 2.0f, bodyWidth / 2.0f, identity, Body::dynamic_body, density);
    chest->SetPosition(headX, headY - headRadius - bodyHeight / 2.0f, 0);

    ragdoll.bones[Ragdoll::index_chest] = Bone{ Ragdoll::index_pelvis, chest };

    float ballSocketFrequency = 60;
    float ballSocketDampingRatio = 1.0f;

    // Chest
    {
        float headFrequency = 20.0f;
        float headDamplingRatio = 1.0f;
        float headAngle = DegToRad(30);
        float headTwistAngle = DegToRad(45);

        BallSocketJoint* j1 = world->CreateBallSocketJoint(
            chest, head, chest->GetPosition() + Vec3{ 0.0f, bodyHeight / 2.0f, 0 }, ballSocketFrequency, ballSocketDampingRatio
        );
        ConeSwingJoint* j2 = world->CreateConeSwingJoint(chest, head, y_axis, headAngle, headFrequency, headDamplingRatio);
        TwistAngleJoint* j3 =
            world->CreateTwistAngleJoint(chest, head, y_axis, -headTwistAngle, headTwistAngle, headFrequency, headDamplingRatio);
        UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
        UserFlag::SetFlag(j2, UserFlag::hide_joint, true);
        UserFlag::SetFlag(j3, UserFlag::hide_joint, true);

        ragdoll.bones[Ragdoll::index_head] = Bone{ Ragdoll::index_chest, head };
    }

    // Arms
    {
        float armRadius = 0.1f * scale;
        float lowerArmRadius = armRadius * 0.95f;
        float armLength = 0.6f * scale;
        float lowerArmLength = armLength * 0.9f;
        float bodyArmGap = -1.5f * armRadius;

        float armGap = 0;
        float armStartX = (bodyWidth / 2.0f + armRadius + bodyArmGap);
        float armStartY = (headRadius + neckGap + armRadius);

        Body* upperRightArm = world->CreateCapsule(
            Vec3{ headX + armStartX, headY - armStartY, 0 }, Vec3{ headX + armStartX + armLength, headY - armStartY, 0 },
            armRadius, identity, Body::dynamic_body, false, density
        );

        Body* lowerRightArm = world->CreateCapsule(
            Vec3{ headX + armStartX + armLength + armGap, headY - armStartY, 0 },
            Vec3{ headX + armStartX + armLength + armGap + lowerArmLength, headY - armStartY, 0 }, lowerArmRadius, identity,
            Body::dynamic_body, false, density
        );

        Body* upperLeftArm = world->CreateCapsule(
            Vec3{ headX - armStartX, headY - armStartY, 0 }, Vec3{ headX - armStartX - armLength, headY - armStartY, 0 },
            armRadius, identity, Body::dynamic_body, false, density
        );

        Body* lowerLeftArm = world->CreateCapsule(
            Vec3{ headX - armStartX - armLength - armGap, headY - armStartY, 0 },
            Vec3{ headX - armStartX - armLength - armGap - lowerArmLength, headY - armStartY, 0 }, lowerArmRadius, identity,
            Body::dynamic_body, false, density
        );

        // Arm joints
        {
            float armFrequency = 20.0f;
            float armDampingRatio = 1.0f;

            float armAngleFrequency = 20.0f;
            float armAngleDampingRatio = 1.0f;

            float armSwingAngle = DegToRad(85.0f);
            float armTwistAngle = DegToRad(15.0f);

            float elbowAngle = DegToRad(80.0f);

            // chest -> upper right arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    chest, upperRightArm, Vec3{ headX + armStartX, headY - armStartY, 0 }, ballSocketFrequency,
                    ballSocketDampingRatio
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    chest, upperRightArm, x_axis, armSwingAngle, armAngleFrequency, armAngleDampingRatio
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    chest, upperRightArm, x_axis, -armTwistAngle, armTwistAngle, armAngleFrequency, armDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j3, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_upperRightArm] = Bone{ Ragdoll::index_chest, upperRightArm };
            }

            // upper right arm -> lower right arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperRightArm, lowerRightArm, Vec3{ headX + armStartX + armLength + armGap, headY - armStartY, 0 },
                    ballSocketFrequency, ballSocketDampingRatio
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperRightArm, lowerRightArm, -y_axis, 0, elbowAngle, armAngleFrequency, armAngleDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_lowerRightArm] = Bone{ Ragdoll::index_upperRightArm, lowerRightArm };
            }

            // chest -> upper left arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    chest, upperLeftArm, Vec3{ headX - armStartX, headY - armStartY, 0 }, ballSocketFrequency,
                    ballSocketDampingRatio
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    chest, upperLeftArm, -x_axis, armSwingAngle, armAngleFrequency, armAngleDampingRatio
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    chest, upperLeftArm, -x_axis, -armTwistAngle, armTwistAngle, armAngleFrequency, armDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j3, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_upperLeftArm] = Bone{ Ragdoll::index_chest, upperLeftArm };
            }

            // upper left arm -> lower left arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperLeftArm, lowerLeftArm, Vec3{ headX - armStartX - armLength - armGap, headY - armStartY, 0 },
                    ballSocketFrequency, ballSocketDampingRatio
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperLeftArm, lowerLeftArm, y_axis, 0, elbowAngle, armAngleFrequency, armAngleDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_lowerLeftArm] = Bone{ Ragdoll::index_upperLeftArm, lowerLeftArm };
            }
        }
    }

    Vec3 pelvisCenter = chest->GetPosition() - Vec3(0, bodyHeight / 2, 0);
    float pelvisWidth = 0.15f * scale;
    float pelvisRadius = 0.225f * scale;
    Vec3 pelvisLeft = pelvisCenter - Vec3{ pelvisWidth / 2, 0, 0 };
    Vec3 pelvisRight = pelvisCenter + Vec3{ pelvisWidth / 2, 0, 0 };
    Vec3 pelvisTop = pelvisCenter + Vec3(0, pelvisRadius / 2, 0);

    // Pelvis
    Body* pelvis = world->CreateCapsule(pelvisLeft, pelvisRight, pelvisRadius, identity, Body::dynamic_body, false, density);
    pelvis->SetCollisionFilter(filter);

    // pelvis -> chest
    {
        float pelvisFrequency = 20.0f;
        float pelvisDampingRatio = 1.0f;
        float pelvisMinAngle = DegToRad(60.0f);
        float pelvisMaxAngle = DegToRad(80.0f);
        float pelvisAngleFrequency = 20.0f;
        float pelvisAngleDampingRatio = 1.0f;

        BallSocketJoint* j1 = world->CreateBallSocketJoint(pelvis, chest, pelvisTop, ballSocketFrequency, ballSocketDampingRatio);
        RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
            pelvis, chest, x_axis, -pelvisMinAngle, pelvisMaxAngle, pelvisAngleFrequency, pelvisAngleDampingRatio
        );
        UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
        UserFlag::SetFlag(j2, UserFlag::hide_joint, true);

        ragdoll.bones[Ragdoll::index_pelvis] = { -1, pelvis };
    }

    // Legs
    {
        float legStartX = 0.15f * scale;
        float legRadius = 0.14f * scale;
        float lowerLegRadius = legRadius * 0.95f;
        float bodyLegGap = -1.5f * legRadius;
        float legLength = 1.0f * scale;
        float lowerLegLength = legLength * 0.95f;
        float legGap = 0.0f;
        float legStartY = (bodyHeight + headRadius + neckGap + legRadius + bodyLegGap);

        Body* upperRightLeg = world->CreateCapsule(
            Vec3{ headX + legStartX, headY - legStartY, 0 }, Vec3{ headX + legStartX, headY - legStartY - legLength, 0 },
            legRadius, identity, Body::dynamic_body, false, density
        );

        Body* lowerRightLeg = world->CreateCapsule(
            Vec3{ headX + legStartX, headY - legStartY - legLength - legGap, 0 },
            Vec3{ headX + legStartX, headY - legStartY - legLength - legGap - lowerLegLength, 0 }, lowerLegRadius, identity,
            Body::dynamic_body, false, density
        );

        Body* upperLeftLeg = world->CreateCapsule(
            Vec3{ headX - legStartX, headY - legStartY, 0 }, Vec3{ headX - legStartX, headY - legStartY - legLength, 0 },
            legRadius, identity, Body::dynamic_body, false, density
        );

        Body* lowerLeftLeg = world->CreateCapsule(
            Vec3{ headX - legStartX, headY - legStartY - legLength - legGap, 0 },
            Vec3{ headX - legStartX, headY - legStartY - legLength - legGap - lowerLegLength, 0 }, lowerLegRadius, identity,
            Body::dynamic_body, false, density
        );

        // Leg joints
        {
            float LegFrequency = 20.0f;
            float LegDampingRatio = 1.0f;

            float legAngleFrequency = 20.0f;
            float legAngleDampingRatio = 1.0f;

            float legAngle = DegToRad(15.0f);
            float legTwistAngle = DegToRad(5.0f);

            float kneeAngle = DegToRad(150.0f);

            // pelvis -> upper right leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    pelvis, upperRightLeg, Vec3{ headX + legStartX, headY - legStartY, 0 }, ballSocketFrequency,
                    ballSocketDampingRatio
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    pelvis, upperRightLeg, -y_axis, legAngle, legAngleFrequency, legAngleDampingRatio
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    pelvis, upperRightLeg, -y_axis, -legTwistAngle, legTwistAngle, legAngleFrequency, legAngleDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j3, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_upperRightLeg] = Bone{ Ragdoll::index_pelvis, upperRightLeg };
            }

            // upper right leg -> lower right leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperRightLeg, lowerRightLeg, Vec3{ headX + legStartX, headY - legStartY - legLength - legGap, 0 },
                    ballSocketFrequency, ballSocketDampingRatio
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperRightLeg, lowerRightLeg, x_axis, 0, kneeAngle, legAngleFrequency, legAngleDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_lowerRightLeg] = Bone{ Ragdoll::index_upperRightLeg, lowerRightLeg };
            }

            // pelvis -> upper left leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    pelvis, upperLeftLeg, Vec3{ headX - legStartX, headY - legStartY, 0 }, ballSocketFrequency,
                    ballSocketDampingRatio
                );
                ConeSwingJoint* j2 =
                    world->CreateConeSwingJoint(pelvis, upperLeftLeg, -y_axis, legAngle, legAngleFrequency, legAngleDampingRatio);
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    pelvis, upperLeftLeg, y_axis, -legTwistAngle, legTwistAngle, legAngleFrequency, legAngleDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j3, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_upperLeftLeg] = Bone{ Ragdoll::index_pelvis, upperLeftLeg };
            }

            // upper left leg -> lower left leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperLeftLeg, lowerLeftLeg, Vec3{ headX - legStartX, headY - legStartY - legLength - legGap, 0 },
                    ballSocketFrequency, ballSocketDampingRatio
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperLeftLeg, lowerLeftLeg, x_axis, 0, kneeAngle, legAngleFrequency, legAngleDampingRatio
                );
                UserFlag::SetFlag(j1, UserFlag::hide_joint, true);
                UserFlag::SetFlag(j2, UserFlag::hide_joint, true);

                ragdoll.bones[Ragdoll::index_lowerLeftLeg] = Bone{ Ragdoll::index_upperLeftLeg, lowerLeftLeg };
            }
        }
    }

    for (int32 i = 0; i < Ragdoll::bone_count; ++i)
    {
        Body* body = ragdoll.bones[i].body;
        body->SetCollisionFilter(filter);
        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);
        body->SetGyroscopicTorqueEnabled(true);
    }

    return ragdoll;
}

inline void DeleteRagdoll(World* world, const Ragdoll& ragdoll)
{
    if (world != ragdoll.bones[0].body->GetWorld())
    {
        return;
    }

    for (int32 i = 0; i < Ragdoll::bone_count; ++i)
    {
        world->BufferDestroy(ragdoll.bones[i].body);
    }
}

} // namespace muli3
