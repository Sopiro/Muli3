#pragma once

#include "camera.h"
#include "mesh.h"
#include "options.h"
#include "shader.h"

namespace muli3
{

class Renderer : NonCopyable
{
public:
    ~Renderer();

    bool Initialize();
    void Shutdown();
    void Render(const World& world, const Camera& camera, float aspectRatio, const DebugOptions& options);

private:
    bool CreateShadowResources();
    bool CreateDebugResources();
    void DestroyShadowResources();
    void DestroyDebugResources();
    void DrawBody(const RigidBody& body, const Shader& shader) const;
    void DrawAABB(const AABB& aabb, std::vector<Vec3>& lines) const;
    void DrawDebugPrimitives(
        const Mat4& view,
        const Mat4& projection,
        GLenum primitive,
        const std::vector<Vec3>& vertices,
        const Vec3& color,
        float pointSize = 6.0f
    );
    void DrawDebug(const World& world, const Mat4& view, const Mat4& projection, const DebugOptions& options);

    Shader surfaceShader;
    Shader shadowShader;
    Shader debugShader;
    Mesh sphereMesh;
    bool initialized = false;
    GLuint shadowFramebuffer = 0;
    GLuint shadowDepthTexture = 0;
    GLuint debugVAO = 0;
    GLuint debugVBO = 0;
};

} // namespace muli3
