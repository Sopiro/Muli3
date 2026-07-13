![logo](.github/logo.gif)

# Muli3

[![Build](https://github.com/Sopiro/Muli3/actions/workflows/cmake-multi-platform.yml/badge.svg)](https://github.com/Sopiro/Muli3/actions/workflows/cmake-multi-platform.yml)

3D Rigidbody physics engine.

## Features

Same API as [Muli](https://github.com/Sopiro/Muli), with the dynamics expanded to 3D and multi-threading support.

### Collision
  - Shapes: sphere, capsule, box, polygon, polyhedron, height field, and triangle mesh
  - Support for rounded shapes
  - Multiple colliders attached to a single body
  - Dynamic, static, and kinematic bodies
  - Collision filtering
  - Dynamic AABB tree broad phase
  - One-shot contact manifold generation
  - Accelerated raycast, shapecast, and area queries

 ### Physics Simulation
  - PGS solver with a separate position correction (PGS NGS)
  - Persistent constraint graph
  - Graph coloring based solver with multi-threading
  - Constraint islanding and sleeping
  - Deterministic simulation
  - Contact callbacks: begin, touching, end, pre-solve, post-solve, and destroy
  - Physics material: friction, restitution, and surface speed
  - Various joint types

### Others
  - Cross platform library (C++20)
  - Intuitive API design
  - 30+ Demos

## Example

```c++
#include "muli3/muli3.h"

using namespace muli3;

int main()
{
    ThreadPool pool(8);
    
    WorldSettings settings;
    settings.thread_pool = &pool;

    World world(&settings);

    Body* box = world.CreateBox(1.0f);
    box->SetPosition(0.0f, 5.0f, 0.0f);

    // Run simulation for one second
    float dt = 1.0f / 60.0f;
    for(int i = 0; i < 60; ++i)
    {
        world.Step(dt);
    }

    return 0;
}
```

## Building and running the demo
- Install [CMake](https://cmake.org/install/).
- Ensure CMake is in the system `PATH`.
- Clone the repository: `git clone https://github.com/Sopiro/Muli3`
- Run the following commands:
  ```sh
  git clone https://github.com/Sopiro/Muli3
  cd Muli3
  cmake -S . -B build
  cmake --build build --config Release
  ```
- You can find the demo executable in `build/bin`.

## Including the library
You can easily include the library using CMake's `FetchContent` module.

```cmake
include(FetchContent)

FetchContent_Declare(
    muli3
    GIT_REPOSITORY https://github.com/Sopiro/Muli3
)

set(MULI3_BUILD_DEMO OFF)
FetchContent_MakeAvailable(muli3)
```
