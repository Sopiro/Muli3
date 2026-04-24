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
    struct SphereInstance
    {
        Vec4 model0;
        Vec4 model1;
        Vec4 model2;
        Vec4 model3;
        Vec4 color;
    };

    struct DebugVertex
    {
        Vec3 position;
        Vec3 color;
    };

    bool CreateShadowResources();
    bool CreateDebugResources();
    bool CreateBatchResources();
    void DestroyShadowResources();
    void DestroyDebugResources();
    void DestroyBatchResources();
    void DrawBody(const RigidBody& body, const Vec3& color, const Shader& shader);
    void DrawPoint(const Vec3& point, const Vec3& color);
    void DrawLine(const Vec3& p1, const Vec3& p2, const Vec3& color);
    void DrawAABB(const AABB& aabb, const Vec3& color);
    void FlushSpheres(const Shader& shader);
    void FlushPoints(const Mat4& view, const Mat4& projection, float pointSize = 6.0f);
    void FlushLines(const Mat4& view, const Mat4& projection);
    void FlushDebugVertices(const Mat4& view, const Mat4& projection, GLenum primitive, const std::vector<DebugVertex>& vertices, float pointSize);
    void FlushAllDebug(const Mat4& view, const Mat4& projection);
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
    GLuint sphereInstanceVBO = 0;
    std::vector<SphereInstance> sphereInstances;
    std::vector<DebugVertex> points;
    std::vector<DebugVertex> lines;
};

} // namespace muli3
