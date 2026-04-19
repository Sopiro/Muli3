#pragma once

#include "camera.h"
#include "mesh.h"
#include "shader.h"

namespace muli3
{

class Renderer : NonCopyable
{
public:
    ~Renderer();

    bool Initialize();
    void Shutdown();
    void Render(const World& world, const Camera& camera, float aspectRatio) const;

private:
    bool CreateShadowResources();
    void DestroyShadowResources();

    Shader surfaceShader;
    Shader shadowShader;
    Mesh sphereMesh;
    bool initialized = false;
    GLuint shadowFramebuffer = 0;
    GLuint shadowDepthTexture = 0;
};

} // namespace muli3
