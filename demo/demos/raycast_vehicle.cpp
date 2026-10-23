#include "demo.h"
#include "game.h"
#include "noise.h"
#include "renderer.h"
#include "window.h"

namespace muli3
{

static constexpr Vec3 chassis_vertices[12] = {
    Vec3{ -0.95f, -0.45f, -2.0f }, Vec3{ -0.95f, -0.05f, -2.0f }, Vec3{ -0.95f, 0.6f, -0.75f }, Vec3{ -0.95f, 0.6f, 1.3f },
    Vec3{ -0.95f, 0.05f, 2.0f },   Vec3{ -0.95f, -0.45f, 2.0f },  Vec3{ 0.95f, -0.45f, -2.0f }, Vec3{ 0.95f, -0.05f, -2.0f },
    Vec3{ 0.95f, 0.6f, -0.75f },   Vec3{ 0.95f, 0.6f, 1.3f },     Vec3{ 0.95f, 0.05f, 2.0f },   Vec3{ 0.95f, -0.45f, 2.0f }
};
static constexpr float raycast_wheel_radius = 0.32f;
static constexpr Vec3 raycast_wheel_mounts[] = { Vec3{ -1.05f, -0.15f, -1.45f }, Vec3{ 1.05f, -0.15f, -1.45f },
                                                 Vec3{ -1.05f, -0.15f, 1.45f }, Vec3{ 1.05f, -0.15f, 1.45f } };

static float suspensionLength = 0.5f;
static float suspensionFrequency = 3.0f;
static float suspensionDamping = 0.7f;

static float tireGrip = 1.2f;
static float rollingResistance = 0.3f;

static float driveAcceleration = 10.0f;
static float maxSpeed = 30.0f;
static float maxSteeringAngle = 45.0f;

static bool vehicleChaseCamera = true;
static bool drawRays = true;

struct RaycastWheel
{
    float length = suspensionLength;
    float load = 0.0f;
    Vec3 contactPoint{ 0.0f };
    Vec3 contactNormal{ 0.0f };
    bool grounded = false;
};

class WheelRay : public RayCastAnyCallback
{
public:
    explicit WheelRay(Body* chassis, const Vec3& up)
        : chassis{ chassis }
        , up{ up }
    {
    }

    float OnHitAny(Collider* collider, Vec3 point, Vec3 normal, float fraction) override
    {
        // alignment = cos(up, normal); keep it > 0.3 for stable 1/alignment^2.
        if (collider->GetBody() != chassis && Dot(up, normal) > 0.3f && fraction < closestFraction)
        {
            hit = collider;
            contactPoint = point;
            contactNormal = normal;
            closestFraction = fraction;
        }
        return closestFraction;
    }

    Body* chassis;
    Vec3 up;
    Collider* hit = nullptr;
    Vec3 contactPoint{ 0.0f };
    Vec3 contactNormal{ 0.0f };
    float closestFraction = 1.0f;
};

static float WheelInverseMass(const Body& chassis, const Body& ground, const Vec3& point, const Vec3& direction)
{
    // Inverse effective mass along direction d:
    // K_d = 1/m_a + 1/m_b
    //     + (r_a x d)^T * I_a^-1 * (r_a x d)
    //     + (r_b x d)^T * I_b^-1 * (r_b x d)
    // For unit d, an impulse j*d changes relative speed by K_d*j; stopping needs j = -v_d/K_d.

    Vec3 ra = Cross(point - chassis.GetMotion().c, direction);
    Vec3 rb = Cross(point - ground.GetMotion().c, direction);

    return chassis.GetBodyState()->invMass + ground.GetBodyState()->invMass +
           Dot(ra, chassis.GetWorldInverseInertiaTensor() * ra) + Dot(rb, ground.GetWorldInverseInertiaTensor() * rb);
}

static void StepRaycastWheels(
    World& world, Body& chassis, std::span<RaycastWheel, 4> wheels, float throttle, float steering, bool brake, float dt
)
{
    struct TireContact
    {
        Body* ground = nullptr;
        Vec3 forward{ 0.0f };
        Vec3 side{ 0.0f };
        Vec3 gravityVelocity{ 0.0f };
        Vec2 impulseSum{ 0.0f }; // x: longitudinal, y: lateral (N*s)
        float suspensionImpulse = 0.0f;
        float suspensionMass = 0.0f;
        float suspensionBias = 0.0f;
        float suspensionGamma = 0.0f;
        float forwardInverseMass = 0.0f;
        float sideInverseMass = 0.0f;
        float friction = 0.0f;
        float driveImpulse = 0.0f;
        bool holding = false;
    };

    bool active = !chassis.IsSleeping() || throttle != 0.0f || steering != 0.0f;
    Vec3 up = chassis.GetRotation().Rotate(y_axis);

    TireContact tires[4];

    // Equal supported mass per wheel is a tuning approximation, not the contact effective mass 1/K_d.
    float sprungMass = chassis.GetMass() / 4.0f;

    for (int32 i = 0; i < 4; ++i)
    {
        RaycastWheel& wheel = wheels[i];

        // World mount = p + R*localMount; ray direction = -R*Y, with reach L + wheel radius.
        Vec3 origin = Mul(chassis.GetTransform(), raycast_wheel_mounts[i]);
        WheelRay ray{ &chassis, up };
        world.RayCastAny(origin, origin - up * (suspensionLength + raycast_wheel_radius), &ray);

        // Suspension length = hit fraction * ray reach - radius; a miss gives full extension L.
        float length = ray.hit ? Max(0.0f, ray.closestFraction * (suspensionLength + raycast_wheel_radius) - raycast_wheel_radius)
                               : suspensionLength;

        active |= ray.hit && Dot(wheel.contactNormal, ray.contactNormal) < 0.999f;
        active |= wheel.grounded != (ray.hit != nullptr) || Abs(length - wheel.length) > linear_slop;

        wheel.grounded = ray.hit != nullptr;
        wheel.length = length;
        if (!wheel.grounded)
        {
            continue;
        }

        Body& ground = *ray.hit->GetBody();
        active |= !ground.IsStatic() && !ground.IsSleeping();

        wheel.contactPoint = ray.contactPoint;
        wheel.contactNormal = ray.contactNormal;

        tires[i].ground = &ground;
        tires[i].friction = MixFriction(tireGrip, ray.hit->GetFriction());
    }

    // Ray contacts must wake sleeping cars when support changes.
    if (!active)
    {
        return;
    }

    if (chassis.IsSleeping())
    {
        chassis.Awake();
    }

    for (int32 i = 0; i < 4; ++i)
    {
        TireContact& tire = tires[i];
        wheels[i].load = 0.0f;
        if (!tire.ground)
        {
            continue;
        }

        RaycastWheel& wheel = wheels[i];
        Body& ground = *tire.ground;

        float alignment = Dot(up, wheel.contactNormal);

        // a = up dot n; normal displacement C_n = a*(length - L).
        // Preserve spring energy: k_n*C_n^2 = k*(length - L)^2, so k_n = k/a^2 and damping_n = damping/a^2.
        // Since k = m*w^2 and damping = 2*m*zeta*w, pass m/a^2 to scale both without changing frequency or ratio.
        float beta;
        ComputeBetaAndGamma(
            &beta, &tire.suspensionGamma, suspensionFrequency, suspensionDamping, sprungMass / Sqr(alignment), dt
        );
        // With h = dt: beta = h*k_n/(damping_n + h*k_n), gamma = 1/[h*(damping_n + h*k_n)].
        // bias = beta*C_n/h is negative under compression, requesting separation; solver mass = 1/(K_n + gamma).
        tire.suspensionBias = (wheel.length - suspensionLength) * alignment * beta / dt;
        tire.suspensionMass =
            1.0f / (WheelInverseMass(chassis, ground, wheel.contactPoint, wheel.contactNormal) + tire.suspensionGamma);

        // Predict delta-v = (g_chassis - g_ground)*dt; World::Step applies gravity after these impulses.
        if (world.GetSettings().apply_gravity)
        {
            tire.gravityVelocity = world.GetSettings().gravity *
                                   (chassis.GetGravityScale() - (ground.IsDynamic() ? ground.GetGravityScale() : 0.0f)) * dt;
        }

        // Steer the front pair and project onto the surface: f = normalize(f_raw - n*(n dot f_raw)).
        // side = f cross n completes an orthonormal tangent basis for longitudinal/lateral impulses.
        Vec3 forward = chassis.GetRotation().Rotate(Quat{ i < 2 ? steering : 0.0f, y_axis }.Rotate(-z_axis));
        forward -= wheel.contactNormal * Dot(forward, wheel.contactNormal);
        forward.Normalize();

        tire.forward = forward;
        tire.side = Cross(forward, wheel.contactNormal);
        tire.forwardInverseMass = WheelInverseMass(chassis, ground, wheel.contactPoint, tire.forward);
        tire.sideInverseMass = WheelInverseMass(chassis, ground, wheel.contactPoint, tire.side);

        Vec3 relativeVelocity = chassis.GetVelocityAtWorldSpace(wheel.contactPoint) -
                                ground.GetVelocityAtWorldSpace(wheel.contactPoint) + tire.gravityVelocity;
        float speed = Dot(relativeVelocity, forward);
        relativeVelocity -= wheel.contactNormal * Dot(relativeVelocity, wheel.contactNormal);
        // Remove normal velocity before testing |v_t|^2 < 0.2^2. Rest hold is a low-speed approximation.
        // With no throttle it may use full grip; zero rolling resistance disables this correction.
        tire.holding = throttle == 0.0f && rollingResistance > 0.0f && Dot(relativeVelocity, relativeVelocity) < 0.04f;
        // j_drive = (m/4)*acceleration*throttle*max(0, 1 - signedSpeed/maxSpeed)*dt.
        // Compute once per step, not per iteration; four driven wheels sum to roughly m*acceleration at rest.
        tire.driveImpulse = sprungMass * driveAcceleration * throttle * Max(0.0f, 1.0f - speed * throttle / maxSpeed) * dt;
    }

    // Sequential impulses couple all wheels through the same chassis; accumulated impulses reset each step.
    for (int32 iteration = 0; iteration < 8; ++iteration)
    {
        for (int32 i = 0; i < 4; ++i)
        {
            TireContact& tire = tires[i];
            if (!tire.ground)
            {
                continue;
            }
            RaycastWheel& wheel = wheels[i];
            Body& ground = *tire.ground;
            Vec3 relativeVelocity = chassis.GetVelocityAtWorldSpace(wheel.contactPoint) -
                                    ground.GetVelocityAtWorldSpace(wheel.contactPoint) + tire.gravityVelocity;
            // Solve (K_n + gamma)*deltaJ = -(v_n + bias + gamma*J_old), then clamp J_new >= 0 (push only).
            float impulseSum =
                Max(0.0f, tire.suspensionImpulse -
                              tire.suspensionMass * (Dot(relativeVelocity, wheel.contactNormal) + tire.suspensionBias +
                                                     tire.suspensionGamma * tire.suspensionImpulse));
            Vec3 impulse = wheel.contactNormal * (impulseSum - tire.suspensionImpulse);
            tire.suspensionImpulse = impulseSum;
            wheel.load = impulseSum / dt; // Average normal force N = J_n/dt.
            chassis.ApplyLinearImpulse(wheel.contactPoint, impulse, true);
            ground.ApplyLinearImpulse(wheel.contactPoint, -impulse, true);
        }

        for (int32 i = 0; i < 4; ++i)
        {
            TireContact& tire = tires[i];
            if (!tire.ground)
            {
                continue;
            }
            Body& ground = *tire.ground;
            Vec3 relativeVelocity = chassis.GetVelocityAtWorldSpace(wheels[i].contactPoint) -
                                    ground.GetVelocityAtWorldSpace(wheels[i].contactPoint) + tire.gravityVelocity;
            float speed = Dot(relativeVelocity, tire.forward);

            // Accumulated stopping impulse J_new = J_old - v_f/K_f; only its change is applied below.
            float longitudinalImpulse = tire.impulseSum.x - speed / tire.forwardInverseMass;

            if (!brake && !tire.holding)
            {
                if (throttle != 0.0f)
                {
                    // Driving: apply engine impulse without rolling resistance.
                    longitudinalImpulse = tire.driveImpulse;
                }
                else
                {
                    // Coasting: rolling resistance opposes longitudinal motion.
                    float impulseLimit = rollingResistance * tire.suspensionImpulse;

                    longitudinalImpulse = Clamp(longitudinalImpulse, -impulseLimit, impulseLimit);
                }
            }

            // Same zero-slip solve along side: J_side,new = J_side,old - (v_rel dot side)/K_side.
            float lateralImpulse = tire.impulseSum.y - Dot(relativeVelocity, tire.side) / tire.sideInverseMass;

            Vec2 impulseSum{ longitudinalImpulse, lateralImpulse };

            float impulseLength = Length(impulseSum);
            // Shared friction circle: sqrt(J_forward^2 + J_side^2) <= mu*J_n; scale radially when exceeded.
            float frictionLimit = tire.friction * tire.suspensionImpulse;
            if (impulseLength > frictionLimit)
            {
                impulseSum *= frictionLimit / impulseLength;
            }

            // Apply only the change in accumulated impulse.
            Vec2 delta = impulseSum - tire.impulseSum;
            tire.impulseSum = impulseSum;

            // Reconstruct the world impulse; each body gets opposite linear impulse and torque r cross J.
            Vec3 impulse = tire.forward * delta.x + tire.side * delta.y;
            chassis.ApplyLinearImpulse(wheels[i].contactPoint, impulse, true);
            ground.ApplyLinearImpulse(wheels[i].contactPoint, -impulse, true);
        }
    }
}

class RaycastVehicleDemo : public Demo,
                           public BodyDestroyCallback
{
public:
    RaycastVehicleDemo(Game& game)
        : Demo(game)
    {
        int32 terrainSampleCount = 128;
        float terrainCellSize = 2.0f;
        std::vector<float> terrainHeights(terrainSampleCount * terrainSampleCount);
        NoiseParams terrainNoise;
        terrainNoise.frequency = 0.025f;
        terrainNoise.amplitude = 4.0f;
        terrainNoise.octaves = 4;
        terrainNoise.lacunarity = 2.0f;
        terrainNoise.gain = 0.5f;
        terrainNoise.seed = 1234;

        for (int32 z = 0; z < terrainSampleCount; ++z)
        {
            for (int32 x = 0; x < terrainSampleCount; ++x)
            {
                float worldX = x * terrainCellSize - 20.0f;
                float worldZ = z * terrainCellSize - 20.0f;
                terrainHeights[z * terrainSampleCount + x] = HeightNoise2(worldX, worldZ, terrainNoise);
            }
        }

        Body* ground = world->CreateHeightField(
            terrainSampleCount, terrainSampleCount, terrainHeights, terrainCellSize, terrainCellSize, identity,
            Vec3{ -20.0f, 0.0f, -20.0f }
        );
        ground->SetFriction(1.0f);

        CollisionFilter filter;
        filter.group = -1;

        Vec3 chassisPosition{ 0.0f, 2.5f, 0.0f };
        chassis = world->CreateConvex(
            chassis_vertices, Transform{ chassisPosition, Quat{ pi, y_axis } }, Body::dynamic_body, default_radius, 0.63f
        );
        chassis->SetCollisionFilter(filter);
        chassis->OnDestroy = this;

        // Keep the chassis upright while leaving yaw free.
        world->CreateRevoluteAngleJoint(ground, chassis, y_axis, -pi, pi, 0.5f, 1.0f);

        camera.SetPosition(cameraPivot + Vec3{ 0.0f, 4.0f, -8.0f });
        camera.SetRotation(180.0f, -25.0f);
    }

    ~RaycastVehicleDemo() override
    {
        if (chassis)
        {
            chassis->OnDestroy = nullptr;
        }
        if (cameraOrbiting)
        {
            Window::Get()->SetCursorHidden(false);
            ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
        }
    }

    void OnBodyDestroy(Body*) override
    {
        chassis = nullptr;
        for (RaycastWheel& wheel : wheels)
        {
            wheel = RaycastWheel{};
        }
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnableBodyCreate();
        EnableBodyGrab();

        Window* window = Window::Get();
        bool captureMouse = ImGui::GetIO().WantCaptureMouse;

        if (vehicleChaseCamera && !cameraOrbiting && !captureMouse && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT) &&
            Input::IsMouseDown(GLFW_MOUSE_BUTTON_RIGHT))
        {
            cameraOrbiting = true;
            window->SetCursorHidden(true);
            ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
        }
        else if (cameraOrbiting && (!Input::IsMouseDown(GLFW_MOUSE_BUTTON_RIGHT) || !vehicleChaseCamera))
        {
            cameraOrbiting = false;
            window->SetCursorHidden(false);
            ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
        }

        if (cameraOrbiting)
        {
            Vec2 mouseDelta = Input::GetMouseDelta();
            cameraOrbitYaw -= DegToRad(0.2f) * mouseDelta.x;
            cameraOrbitPitch += DegToRad(0.2f) * mouseDelta.y;
            cameraOrbitPitch = Clamp(cameraOrbitPitch, DegToRad(5.0f), DegToRad(80.0f));
        }
        else if (vehicleChaseCamera && !captureMouse)
        {
            float scroll = Input::GetMouseScroll().y;
            cameraDistance = Clamp(cameraDistance * std::exp(-0.12f * scroll), 3.0f, 30.0f);
        }

        if (!vehicleChaseCamera)
        {
            EnableCameraControl();
        }

        if (!vehicleChaseCamera || ImGui::GetIO().WantCaptureKeyboard)
        {
            throttleInput = 0.0f;
            steeringInput = 0.0f;
            brakeInput = false;
            return;
        }

        brakeInput = Input::IsKeyDown(GLFW_KEY_SPACE);
        throttleInput = float(Input::IsKeyDown(GLFW_KEY_W)) - float(Input::IsKeyDown(GLFW_KEY_S));
        steeringInput = float(Input::IsKeyDown(GLFW_KEY_A)) - float(Input::IsKeyDown(GLFW_KEY_D));
    }

    void Step() override
    {
        if (options.pause && !options.step)
        {
            return;
        }
        if (!chassis)
        {
            Demo::Step();
            return;
        }
        // Attenuate steering by 1/(1 + 0.025*|v|^2); clamp angle change to 2 rad/s * dt.
        float targetSteering = DegToRad(maxSteeringAngle) * steeringInput /
                               (1.0f + 0.025f * Dot(chassis->GetLinearVelocity(), chassis->GetLinearVelocity()));
        steeringAngle += Clamp(targetSteering - steeringAngle, -2.0f * dt, 2.0f * dt);
        StepRaycastWheels(*world, *chassis, wheels, throttleInput, steeringAngle, brakeInput, dt);
        Demo::Step();
    }

    void Update(float alpha) override
    {
        if (!chassis)
        {
            return;
        }
        renderTransform = chassis->GetTransform();
        if (!chassis->IsSleeping())
        {
            chassis->GetMotion().GetTransform(alpha, &renderTransform);
        }
        if (!vehicleChaseCamera)
        {
            return;
        }

        float frameDt = game.GetDeltaTime();
        // Exact first-order smoothing: blend = 1 - exp(-rate*dt), with time constant 1/rate.
        float pivotBlend = 1.0f - std::exp(-10.0f * frameDt);
        cameraPivot += (renderTransform.p - cameraPivot) * pivotBlend;

        // Follow yaw only to avoid suspension pitch/roll shaking the camera.
        Vec3 forward = renderTransform.q.Rotate(-z_axis);
        forward.y = 0.0f;
        if (forward.Normalize() == 0.0f)
        {
            forward = -z_axis;
        }
        Vec3 right = Cross(forward, y_axis);
        Vec3 horizontalDirection = -forward * std::cos(cameraOrbitYaw) + right * std::sin(cameraOrbitYaw);
        Vec3 pivot = cameraPivot;
        // Orbit radius splits into horizontal r*cos(pitch) and vertical r*sin(pitch).
        Vec3 desiredPosition = pivot + horizontalDirection * (cameraDistance * std::cos(cameraOrbitPitch)) +
                               y_axis * (cameraDistance * std::sin(cameraOrbitPitch));

        float blend = 1.0f - std::exp(-5.0f * frameDt);
        camera.position += (desiredPosition - camera.position) * blend;

        Vec3 lookDirection = Normalize(pivot - camera.position);
        // Recover yaw/pitch from the unit look direction; camera forward is local -Z.
        float yaw = std::atan2(-lookDirection.x, -lookDirection.z);
        float pitch = std::asin(Clamp(lookDirection.y, -1.0f, 1.0f));
        camera.SetRotation(RadToDeg(yaw), RadToDeg(pitch));
    }

    void Render() override
    {
        if (!chassis || !drawRays)
        {
            return;
        }
        Vec3 up = renderTransform.q.Rotate(y_axis);
        for (int32 i = 0; i < 4; ++i)
        {
            const RaycastWheel& wheel = wheels[i];
            Vec3 origin = Mul(renderTransform, raycast_wheel_mounts[i]);
            renderer.DrawLine(
                origin, origin - up * (suspensionLength + raycast_wheel_radius),
                wheel.grounded ? Vec4{ 0.15f, 0.75f, 0.3f, 1.0f } : Vec4{ 0.9f, 0.25f, 0.15f, 1.0f }
            );
            if (wheel.grounded)
            {
                renderer.DrawPoint(wheel.contactPoint);
                renderer.DrawLine(wheel.contactPoint, wheel.contactPoint + wheel.contactNormal * 0.35f);
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });
        if (ImGui::Begin("Raycast vehicle", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            int32 grounded = 0;
            for (const RaycastWheel& wheel : wheels)
            {
                grounded += wheel.grounded;
            }
            ImGui::Text(
                "Speed: %.1f km/h",
                chassis ? Dot(chassis->GetLinearVelocity(), chassis->GetRotation().Rotate(-z_axis)) * 3.6f : 0.0f
            );
            ImGui::Text("Grounded wheels: %d / 4", grounded);
            ImGui::Text("W/S: forward / reverse | A/D: steering");
            ImGui::Text("Space: brake | RMB: orbit | Scroll: zoom");
            ImGui::Checkbox("Chase camera", &vehicleChaseCamera);
            ImGui::Checkbox("Show suspension rays", &drawRays);
            bool changed = false;
            changed |= ImGui::SliderFloat("Suspension travel", &suspensionLength, 0.3f, 1.0f, "%.2f m");
            changed |= ImGui::SliderFloat("Suspension frequency", &suspensionFrequency, 1.0f, 5.0f, "%.1f Hz");
            changed |= ImGui::SliderFloat("Suspension damping", &suspensionDamping, 0.2f, 1.5f);
            changed |= ImGui::SliderFloat("Tire grip", &tireGrip, 0.2f, 2.0f);
            changed |= ImGui::SliderFloat("Rolling resistance", &rollingResistance, 0.0f, 0.5f, "%.2f");
            changed |= ImGui::SliderFloat("Acceleration", &driveAcceleration, 1.0f, 12.0f, "%.1f m/s2");
            changed |= ImGui::SliderFloat("Maximum speed", &maxSpeed, 5.0f, 30.0f, "%.1f m/s");
            changed |= ImGui::SliderFloat("Steering angle", &maxSteeringAngle, 5.0f, 90.0f, "%.1f deg");
            if (changed && chassis)
            {
                chassis->Awake();
            }
        }
        ImGui::End();
    }

private:
    Body* chassis = nullptr;
    RaycastWheel wheels[4];
    Transform renderTransform{ Vec3{ 0.0f, 2.5f, 0.0f }, Quat{ pi, y_axis } };
    float throttleInput = 0.0f;
    float steeringInput = 0.0f;
    float steeringAngle = 0.0f;
    bool brakeInput = false;
    float cameraDistance = SafeSqrt(80.0f);
    float cameraOrbitYaw = 0.0f;
    float cameraOrbitPitch = std::atan2(4.0f, 8.0f);
    Vec3 cameraPivot{ 0.0f, 2.5f, 0.0f };
    bool cameraOrbiting = false;
};

static Demo* CreateRaycastVehicleDemo(Game& game)
{
    return new RaycastVehicleDemo(game);
}

static int32 raycastVehicle = register_demo("Vehicle", "Raycast vehicle", CreateRaycastVehicleDemo, 20);

} // namespace muli3
