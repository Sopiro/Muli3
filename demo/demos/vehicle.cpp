#include "demo.h"
#include "game.h"
#include "window.h"

#include <muli3/noise.h>

namespace muli3
{

static float vehicleSuspensionFrequency = 3.5f;
static float vehicleSuspensionDampingRatio = 0.7f;
static float vehicleSuspensionRange = 0.4f;
static float vehicleMaxSteeringAngle = 45.0f;
static float vehicleSteeringFrequency = 10.0f;
static float vehicleSteeringDampingRatio = 0.7f;
static float vehicleMaxSteeringTorque = 5.0f;
static float vehicleSpinSpeedDegrees = 2400.0f;
static float vehicleDriveTorque = 5.0f;
static bool vehicleChaseCamera = true;

class VehicleDemo : public Demo
{
public:
    VehicleDemo(Game& game)
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
        Vec3 chassisVertices[12] = {
            Vec3{ -0.95f, -0.45f, -2.0f }, Vec3{ -0.95f, -0.05f, -2.0f }, Vec3{ -0.95f, 0.6f, -0.75f },
            Vec3{ -0.95f, 0.6f, 1.3f },    Vec3{ -0.95f, 0.05f, 2.0f },   Vec3{ -0.95f, -0.45f, 2.0f },
            Vec3{ 0.95f, -0.45f, -2.0f },  Vec3{ 0.95f, -0.05f, -2.0f },  Vec3{ 0.95f, 0.6f, -0.75f },
            Vec3{ 0.95f, 0.6f, 1.3f },     Vec3{ 0.95f, 0.05f, 2.0f },    Vec3{ 0.95f, -0.45f, 2.0f },
        };
        chassis = world->CreateConvex(chassisVertices, Transform{ chassisPosition }, Body::dynamic_body, default_radius, 0.63f);
        chassis->SetCollisionFilter(filter);

        // Keep the chassis upright while leaving yaw free.
        Joint* j = world->CreateRevoluteAngleJoint(ground, chassis, y_axis, 0.5f, 1.0f);

        Vec3 wheelAnchors[4] = {
            Vec3{ -0.8f, -0.5f, -1.5f },
            Vec3{ 0.8f, -0.5f, -1.5f },
            Vec3{ -0.8f, -0.5f, 1.5f },
            Vec3{ 0.8f, -0.5f, 1.5f },
        };

        for (int32 i = 0; i < 4; ++i)
        {
            Wheel& wheel = wheels[i];

            Vec3 wheelCenter = Mul(chassis->GetTransform(), wheelAnchors[i]);

            // Cylinders are created along local Y. Rotate the wheel axis to world X to match axis B.
            Quat wheelRotation{ -0.5f * pi, z_axis };

            // wheel.body = world->CreateSphere(0.4f, Transform{ wheelCenter }, Body::dynamic_body, 2.0f);

            wheel.body = world->CreateCylinder(
                0.3f, 0.4f, 0.4f, 24, Transform{ wheelCenter, wheelRotation }, Body::dynamic_body, 0.01f, 2.0f
            );

            wheel.body->SetCollisionFilter(filter);
            wheel.body->SetFriction(2.0f);

            // The line keeps the wheel center on the chassis suspension axis.
            world->CreateLineJoint(chassis, wheel.body, wheelCenter, y_axis, 30.0f, 1.0f);

            // A soft fixed distance supplies the suspension spring and damping.
            wheel.suspension = world->CreateDistanceJoint(
                chassis, wheel.body, wheelCenter, wheelCenter, 0.0f, vehicleSuspensionFrequency, vehicleSuspensionDampingRatio
            );

            wheel.suspensionLimit = world->CreateLimitedDistanceJoint(
                chassis, wheel.body, wheelCenter, wheelCenter, -vehicleSuspensionRange / 2, vehicleSuspensionRange / 2, -1.0f,
                1.0f
            );

            // Axis A is chassis steering and axis B is wheel spin.
            wheel.angular = world->CreateUniversalAngleJoint(chassis, wheel.body, y_axis, x_axis, -1.0f, 1.0f);

            wheel.steering = i < 2;
            wheel.driven = i >= 2;
            wheel.angular->SetSteeringLimitEnabled(true);

            if (wheel.steering)
            {
                wheel.angular->SetSteeringMotorEnabled(true);
                wheel.angular->SetSteeringMinAngle(DegToRad(-vehicleMaxSteeringAngle));
                wheel.angular->SetSteeringMaxAngle(DegToRad(vehicleMaxSteeringAngle));
                wheel.angular->SetSteeringFrequency(vehicleSteeringFrequency);
                wheel.angular->SetSteeringDampingRatio(vehicleSteeringDampingRatio);
                wheel.angular->SetMaxSteeringTorque(vehicleMaxSteeringTorque);
            }
            else
            {
                // Rear wheels spin freely but cannot steer.
                wheel.angular->SetSteeringMinAngle(0.0f);
                wheel.angular->SetSteeringMaxAngle(0.0f);
            }

            wheel.angular->SetSpinMotorEnabled(wheel.driven);
            wheel.angular->SetMaxSpinTorque(vehicleDriveTorque);
        }

        camera.SetPosition(chassisPosition + Vec3{ 0.0f, 4.0f, 8.0f });
        camera.SetRotation(0.0f, -18.0f);
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnableBodyCreate();
        EnableBodyGrab();

        Window* window = Window::Get();
        bool captureMouse = ImGui::GetIO().WantCaptureMouse;

        if (vehicleChaseCamera && !cameraOrbiting && !captureMouse && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            cameraOrbiting = true;
            window->SetCursorHidden(true);
            ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
        }
        else if (cameraOrbiting && (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_RIGHT) || !vehicleChaseCamera))
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

        if (ImGui::GetIO().WantCaptureKeyboard)
        {
            throttleInput = 0.0f;
            steeringInput = 0.0f;
            return;
        }

        throttleInput = float(Input::IsKeyDown(GLFW_KEY_W)) - float(Input::IsKeyDown(GLFW_KEY_S));
        steeringInput = float(Input::IsKeyDown(GLFW_KEY_A)) - float(Input::IsKeyDown(GLFW_KEY_D));
    }

    void Step() override
    {
        float steeringAngle = DegToRad(vehicleMaxSteeringAngle) * steeringInput;

        for (Wheel& wheel : wheels)
        {
            wheel.suspension->SetParameters(vehicleSuspensionFrequency, vehicleSuspensionDampingRatio);

            wheel.suspensionLimit->SetJointMinLength(-vehicleSuspensionRange / 2);
            wheel.suspensionLimit->SetJointMaxLength(vehicleSuspensionRange / 2);

            if (wheel.steering)
            {
                wheel.angular->SetTargetSteeringAngle(steeringAngle);
                wheel.angular->SetSteeringMinAngle(DegToRad(-vehicleMaxSteeringAngle));
                wheel.angular->SetSteeringMaxAngle(DegToRad(vehicleMaxSteeringAngle));
                wheel.angular->SetSteeringFrequency(vehicleSteeringFrequency);
                wheel.angular->SetSteeringDampingRatio(vehicleSteeringDampingRatio);
                wheel.angular->SetMaxSteeringTorque(vehicleMaxSteeringTorque);
            }

            if (wheel.driven)
            {
                wheel.angular->SetSpinSpeed(-DegToRad(vehicleSpinSpeedDegrees) * throttleInput);
                wheel.angular->SetMaxSpinTorque(vehicleDriveTorque);
            }
        }

        if (throttleInput != 0.0f || steeringInput != 0.0f)
        {
            chassis->Awake();
            for (Wheel& wheel : wheels)
            {
                wheel.body->Awake();
            }
        }

        Demo::Step();
    }

    void Update(float alpha) override
    {
        if (!vehicleChaseCamera)
        {
            return;
        }

        Transform chassisTransform = chassis->GetTransform();
        if (!chassis->IsSleeping())
        {
            chassis->GetMotion().GetTransform(alpha, &chassisTransform);
        }

        float frameDt = game.GetDeltaTime();
        float pivotBlend = 1.0f - std::exp(-10.0f * frameDt);
        cameraPivot += (chassisTransform.p - cameraPivot) * pivotBlend;

        // Follow chassis yaw only so suspension pitch and roll do not shake the camera.
        Vec3 forward = chassisTransform.q.Rotate(-z_axis);
        forward.y = 0.0f;
        if (forward.Normalize() == 0.0f)
        {
            forward = -z_axis;
        }
        Vec3 right = Cross(forward, y_axis);
        Vec3 horizontalDirection = -forward * std::cos(cameraOrbitYaw) + right * std::sin(cameraOrbitYaw);
        Vec3 pivot = cameraPivot;
        Vec3 desiredPosition = pivot + horizontalDirection * (cameraDistance * std::cos(cameraOrbitPitch)) +
                               y_axis * (cameraDistance * std::sin(cameraOrbitPitch));

        float blend = 1.0f - std::exp(-5.0f * frameDt);
        camera.position += (desiredPosition - camera.position) * blend;

        Vec3 lookDirection = Normalize(pivot - camera.position);
        float yaw = std::atan2(-lookDirection.x, -lookDirection.z);
        float pitch = std::asin(Clamp(lookDirection.y, -1.0f, 1.0f));
        camera.SetRotation(RadToDeg(yaw), RadToDeg(pitch));
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Vehicle", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            Vec3 forward = chassis->GetRotation().Rotate(-z_axis);
            float speed = Dot(chassis->GetLinearVelocity(), forward);

            ImGui::Text("Speed: %.2f m/s", speed);
            ImGui::Text("W/S: throttle");
            ImGui::Text("A/D: steering");
            ImGui::Text("RMB: orbit camera");
            ImGui::Text("Scroll: zoom camera");
            ImGui::Checkbox("Chase camera", &vehicleChaseCamera);

            ImGui::Separator();
            ImGui::SliderFloat("Suspension range", &vehicleSuspensionRange, 0.0, 1.0f, "%.2f m");
            ImGui::SliderFloat("Suspension frequency", &vehicleSuspensionFrequency, 0.5f, 15.0f, "%.2f");
            ImGui::SliderFloat("Suspension damping", &vehicleSuspensionDampingRatio, 0.0f, 2.0f, "%.2f");

            ImGui::Separator();
            ImGui::SliderFloat("Max steering angle", &vehicleMaxSteeringAngle, 0.0f, 90.0f, "%.1f deg");
            ImGui::SliderFloat("Steering frequency", &vehicleSteeringFrequency, 0.0f, 10.0f, "%.2f");
            ImGui::SliderFloat("Steering damping", &vehicleSteeringDampingRatio, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("Steering torque", &vehicleMaxSteeringTorque, 0.0f, 20.0f, "%.1f");

            ImGui::Separator();
            ImGui::SliderFloat("Spin speed", &vehicleSpinSpeedDegrees, 0.0f, 3600.0f, "%.1f deg/s");
            ImGui::SliderFloat("Drive torque", &vehicleDriveTorque, 0.0f, 100.0f, "%.1f");
        }
        ImGui::End();
    }

private:
    struct Wheel
    {
        Body* body = nullptr;
        DistanceJoint* suspension = nullptr;
        DistanceJoint* suspensionLimit = nullptr;
        UniversalAngleJoint* angular = nullptr;
        bool steering = false;
        bool driven = false;
    };

    Body* chassis = nullptr;
    Wheel wheels[4];
    float throttleInput = 0.0f;
    float steeringInput = 0.0f;
    float cameraDistance = SafeSqrt(80.0f);
    float cameraOrbitYaw = 0.0f;
    float cameraOrbitPitch = std::atan2(4.0f, 8.0f);
    Vec3 cameraPivot{ 0.0f, 2.5f, 0.0f };
    bool cameraOrbiting = false;
};

static Demo* CreateVehicleDemo(Game& game)
{
    return new VehicleDemo(game);
}

static int32 vehicle = register_demo("Joints", "Vehicle", CreateVehicleDemo, 10);

} // namespace muli3
