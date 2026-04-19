#include "demo.h"

#include "game.h"
#include "renderer.h"

namespace muli3
{

Demo::Demo(Game& game)
    : game{ game }
    , renderer{ game.GetRenderer() }
    , world{ new World(settings) }
{
    camera.Reset();
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
