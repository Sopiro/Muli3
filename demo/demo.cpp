#include "game.h"

namespace muli3
{

Demo::Demo(Game& game)
    : game{ game }
    , renderer{ game.GetRenderer() }
    , options{ game.GetDebugOptions() }
{
    settings.world_bounds.min.y = -30;
    world = new World(settings);

    camera.Reset();
    camera.position = Vec3{ 0.0f, 5.0f, 10.0f };
    camera.rotation = Vec3{ DegToRad(-20.0f), 0.0f, 0.0f };
    dt = game.GetFixedDeltaTime();
}

Demo::~Demo()
{
    delete world;
}

void Demo::Update(float dt, bool captureMouse)
{
    camera.Update(dt, captureMouse);
}

void Demo::Step()
{
    world->Step(dt);
}

} // namespace muli3
