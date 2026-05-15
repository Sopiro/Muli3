#pragma once

namespace muli3
{
struct Bone
{
    int32 parentIndex;
    RigidBody* body;
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

    float headX = headPosition.x;
    float headY = headPosition.y;

    float headSize = 0.2f * scale;
    float headRadius = 0.2f * scale;

    // Head
    RigidBody* head = world->CreateCapsule(headSize, headRadius, identity, RigidBody::dynamic_body, density);
    head->SetPosition(headPosition);
    head->SetCollisionFilter(filter);

    float bodyWidth = 0.5f * scale;
    float bodyHeight = 0.85f * scale;
    float neckGap = 0.05f * scale;

    RigidBody* chest = world->CreateCapsule(bodyHeight / 2.0f, bodyWidth / 2.0f, identity, RigidBody::dynamic_body, density);
    chest->SetPosition(headX, headY - headRadius - bodyHeight / 2.0f, 0);
    chest->SetCollisionFilter(filter);

    ragdoll.bones[Ragdoll::index_chest] = Bone{ Ragdoll::index_pelvis, chest };

    float ballSocketFrequency = -1;

    // Chest
    {
        float headFrequency = 5.0f;
        float headDamplingRatio = 1.0f;
        float headAngle = DegToRad(30);
        BallSocketJoint* j1 = world->CreateBallSocketJoint(
            chest, head, chest->GetPosition() + Vec3{ 0.0f, bodyHeight / 2.0f, 0 }, ballSocketFrequency
        );
        ConeSwingJoint* j2 =
            world->CreateConeSwingJoint(chest, head, y_axis, headAngle, headFrequency, headDamplingRatio, chest->GetMass());
        TwistAngleJoint* j3 =
            world->CreateTwistAngleJoint(chest, head, y_axis, -pi / 2, pi / 2, headFrequency, headDamplingRatio);
        ragdoll.bones[Ragdoll::index_head] = Bone{ Ragdoll::index_chest, head };
    }

    // Arms
    {
        float armRadius = 0.1f * scale;
        float lowerArmRadius = armRadius * 0.95f;
        float armLength = 0.6f * scale;
        float bodyArmGap = -1.5f * armRadius;

        float armGap = 0;
        float armStartX = (bodyWidth / 2.0f + armRadius + bodyArmGap);
        float armStartY = (headRadius + neckGap + armRadius);

        float angularDamping = 1.0f;

        RigidBody* upperRightArm = world->CreateCapsule(
            Vec3{ headX + armStartX, headY - armStartY, 0 }, Vec3{ headX + armStartX + armLength, headY - armStartY, 0 },
            armRadius, identity, RigidBody::dynamic_body, false, density
        );
        upperRightArm->SetCollisionFilter(filter);
        upperRightArm->SetAngularDamping(angularDamping);

        RigidBody* lowerRightArm = world->CreateCapsule(
            Vec3{ headX + armStartX + armLength + armGap, headY - armStartY, 0 },
            Vec3{ headX + armStartX + armLength + armGap + armLength, headY - armStartY, 0 }, lowerArmRadius, identity,
            RigidBody::dynamic_body, false, density
        );
        lowerRightArm->SetCollisionFilter(filter);
        lowerRightArm->SetAngularDamping(angularDamping);

        RigidBody* upperLeftArm = world->CreateCapsule(
            Vec3{ headX - armStartX, headY - armStartY, 0 }, Vec3{ headX - armStartX - armLength, headY - armStartY, 0 },
            armRadius, identity, RigidBody::dynamic_body, false, density
        );
        upperLeftArm->SetCollisionFilter(filter);
        upperLeftArm->SetAngularDamping(angularDamping);

        RigidBody* lowerLeftArm = world->CreateCapsule(
            Vec3{ headX - armStartX - armLength - armGap, headY - armStartY, 0 },
            Vec3{ headX - armStartX - armLength - armGap - armLength, headY - armStartY, 0 }, lowerArmRadius, identity,
            RigidBody::dynamic_body, false, density
        );
        lowerLeftArm->SetCollisionFilter(filter);
        lowerLeftArm->SetAngularDamping(angularDamping);

        // Arm joints
        {
            float armFrequency = 5.0f;
            float armDampingRatio = 1.0f;

            float armAngleFrequency = 5.0f;
            float armAngleDampingRatio = 1.0f;

            float armSwingAngle = DegToRad(85.0f);
            float armTwistAngle = DegToRad(15.0f);

            float elbowAngle = DegToRad(80.0f);

            // chest -> upper right arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    chest, upperRightArm, Vec3{ headX + armStartX, headY - armStartY, 0 }, ballSocketFrequency, armDampingRatio,
                    chest->GetMass()
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    chest, upperRightArm, x_axis, armSwingAngle, armAngleFrequency, armAngleDampingRatio, chest->GetMass()
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    chest, upperRightArm, x_axis, -armTwistAngle, armTwistAngle, armAngleFrequency, armDampingRatio,
                    chest->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperRightArm] = Bone{ Ragdoll::index_chest, upperRightArm };
            }

            // upper right arm -> lower right arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperRightArm, lowerRightArm, Vec3{ headX + armStartX + armLength + armGap, headY - armStartY, 0 },
                    ballSocketFrequency, armDampingRatio, upperRightArm->GetMass()
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperRightArm, lowerRightArm, -y_axis, 0, elbowAngle, armAngleFrequency, armAngleDampingRatio,
                    upperRightArm->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerRightArm] = Bone{ Ragdoll::index_upperRightArm, lowerRightArm };
            }

            // chest -> upper left arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    chest, upperLeftArm, Vec3{ headX - armStartX, headY - armStartY, 0 }, ballSocketFrequency, armDampingRatio,
                    chest->GetMass()
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    chest, upperLeftArm, -x_axis, armSwingAngle, armAngleFrequency, armAngleDampingRatio, chest->GetMass()
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    chest, upperLeftArm, -x_axis, -armTwistAngle, armTwistAngle, armAngleFrequency, armDampingRatio,
                    chest->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperLeftArm] = Bone{ Ragdoll::index_chest, upperLeftArm };
            }

            // upper left arm -> lower left arm
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperLeftArm, lowerLeftArm, Vec3{ headX - armStartX - armLength - armGap, headY - armStartY, 0 },
                    ballSocketFrequency, armDampingRatio, upperLeftArm->GetMass()
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperLeftArm, lowerLeftArm, y_axis, 0, elbowAngle, armAngleFrequency, armAngleDampingRatio,
                    upperLeftArm->GetMass()
                );

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
    RigidBody* pelvis =
        world->CreateCapsule(pelvisLeft, pelvisRight, pelvisRadius, identity, RigidBody::dynamic_body, false, density);
    pelvis->SetCollisionFilter(filter);

    // pelvis -> chest
    {
        float pelvisFrequency = 5.0f;
        float pelvisDampingRatio = 1.0f;
        float pelvisMinAngle = DegToRad(60.0f);
        float pelvisMaxAngle = DegToRad(80.0f);
        float pelvisAngleFrequency = 10.0f;
        float pelvisAngleDampingRatio = 1.0f;

        BallSocketJoint* j1 =
            world->CreateBallSocketJoint(pelvis, chest, pelvisTop, ballSocketFrequency, pelvisDampingRatio, pelvis->GetMass());
        RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
            pelvis, chest, x_axis, -pelvisMinAngle, pelvisMaxAngle, pelvisAngleFrequency, pelvisAngleDampingRatio,
            pelvis->GetMass()
        );

        ragdoll.bones[Ragdoll::index_pelvis] = { -1, pelvis };
    }

    // Legs
    {
        float legStartX = 0.15f * scale;
        float legRadius = 0.14f * scale;
        float lowerLegRadius = legRadius * 0.9f;
        float bodyLegGap = -1.5f * legRadius;
        float legLength = 1.0f * scale;
        float legGap = 0.0f;
        float legStartY = (bodyHeight + headRadius + neckGap + legRadius + bodyLegGap);

        RigidBody* upperRightLeg = world->CreateCapsule(
            Vec3{ headX + legStartX, headY - legStartY, 0 }, Vec3{ headX + legStartX, headY - legStartY - legLength, 0 },
            legRadius, identity, RigidBody::dynamic_body, false, density
        );
        upperRightLeg->SetCollisionFilter(filter);

        RigidBody* lowerRightLeg = world->CreateCapsule(
            Vec3{ headX + legStartX, headY - legStartY - legLength - legGap, 0 },
            Vec3{ headX + legStartX, headY - legStartY - legLength - legGap - legLength, 0 }, lowerLegRadius, identity,
            RigidBody::dynamic_body, false, density
        );
        lowerRightLeg->SetCollisionFilter(filter);

        RigidBody* upperLeftLeg = world->CreateCapsule(
            Vec3{ headX - legStartX, headY - legStartY, 0 }, Vec3{ headX - legStartX, headY - legStartY - legLength, 0 },
            legRadius, identity, RigidBody::dynamic_body, false, density
        );
        upperLeftLeg->SetCollisionFilter(filter);

        RigidBody* lowerLeftLeg = world->CreateCapsule(
            Vec3{ headX - legStartX, headY - legStartY - legLength - legGap, 0 },
            Vec3{ headX - legStartX, headY - legStartY - legLength - legGap - legLength, 0 }, lowerLegRadius, identity,
            RigidBody::dynamic_body, false, density
        );
        lowerLeftLeg->SetCollisionFilter(filter);

        // Leg joints
        {
            float LegFrequency = 30.0f;
            float LegDampingRatio = 1.0f;

            float legAngleFrequency = 10.0f;
            float legAngleDampingRatio = 1.0f;

            float legAngle = DegToRad(15.0f);
            float legTwistAngle = DegToRad(7.5f);

            float kneeAngle = DegToRad(150.0f);

            // pelvis -> upper right leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    pelvis, upperRightLeg, Vec3{ headX + legStartX, headY - legStartY, 0 }, ballSocketFrequency, LegDampingRatio,
                    pelvis->GetMass()
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    pelvis, upperRightLeg, -y_axis, legAngle, legAngleFrequency, legAngleDampingRatio, pelvis->GetMass()
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    pelvis, upperRightLeg, -y_axis, -legTwistAngle, legTwistAngle, legAngleFrequency, legAngleDampingRatio,
                    pelvis->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperRightLeg] = Bone{ Ragdoll::index_pelvis, upperRightLeg };
            }

            // upper right leg -> lower right leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperRightLeg, lowerRightLeg, Vec3{ headX + legStartX, headY - legStartY - legLength - legGap, 0 },
                    ballSocketFrequency, LegDampingRatio, upperRightLeg->GetMass()
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperRightLeg, lowerRightLeg, x_axis, 0, kneeAngle, legAngleFrequency, legAngleDampingRatio,
                    upperRightLeg->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerRightLeg] = Bone{ Ragdoll::index_upperRightLeg, lowerRightLeg };
            }

            // pelvis -> upper left leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    pelvis, upperLeftLeg, Vec3{ headX - legStartX, headY - legStartY, 0 }, ballSocketFrequency, LegDampingRatio,
                    pelvis->GetMass()
                );
                ConeSwingJoint* j2 = world->CreateConeSwingJoint(
                    pelvis, upperLeftLeg, -y_axis, legAngle, legAngleFrequency, legAngleDampingRatio, pelvis->GetMass()
                );
                TwistAngleJoint* j3 = world->CreateTwistAngleJoint(
                    pelvis, upperLeftLeg, y_axis, -legTwistAngle, legTwistAngle, legAngleFrequency, legAngleDampingRatio,
                    pelvis->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperLeftLeg] = Bone{ Ragdoll::index_pelvis, upperLeftLeg };
            }

            // upper left leg -> lower left leg
            {
                BallSocketJoint* j1 = world->CreateBallSocketJoint(
                    upperLeftLeg, lowerLeftLeg, Vec3{ headX - legStartX, headY - legStartY - legLength - legGap, 0 },
                    ballSocketFrequency, LegDampingRatio, upperLeftLeg->GetMass()
                );
                RevoluteAngleJoint* j2 = world->CreateLimitedRevoluteAngleJoint(
                    upperLeftLeg, lowerLeftLeg, x_axis, 0, kneeAngle, legAngleFrequency, legAngleDampingRatio,
                    upperLeftLeg->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerLeftLeg] = Bone{ Ragdoll::index_upperLeftLeg, lowerLeftLeg };
            }
        }
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