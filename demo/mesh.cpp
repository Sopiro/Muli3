#include "mesh.h"
#include "muli3/frame.h"
#include "muli3/shapes.h"

namespace muli3
{

static void SetMeshVertexAttributes()
{
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, uv)));
    glEnableVertexAttribArray(2);
}

Mesh::~Mesh()
{
    Destroy();
}

void Mesh::Upload(const std::vector<MeshVertex>& vertices, const std::vector<uint32>& indices, GLenum primitiveType)
{
    Destroy();

    primitive = primitiveType;
    indexCount = (GLsizei)indices.size();

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(vertices.size() * sizeof(MeshVertex)), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(indices.size() * sizeof(uint32)), indices.data(), GL_STATIC_DRAW);

    SetMeshVertexAttributes();

    glBindVertexArray(0);
}

void Mesh::Upload(
    const std::vector<MeshVertex>& vertices, const std::vector<uint32>& triangleIndices, const std::vector<uint32>& lineIndices
)
{
    Upload(vertices, triangleIndices, GL_TRIANGLES);
    outlineIndexCount = (GLsizei)lineIndices.size();

    glGenVertexArrays(1, &outlineVao);
    glGenBuffers(1, &outlineEbo);
    glBindVertexArray(outlineVao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outlineEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(lineIndices.size() * sizeof(uint32)), lineIndices.data(), GL_STATIC_DRAW);
    SetMeshVertexAttributes();

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Mesh::Destroy()
{
    if (outlineEbo != 0)
    {
        glDeleteBuffers(1, &outlineEbo);
        outlineEbo = 0;
    }
    if (outlineVao != 0)
    {
        glDeleteVertexArrays(1, &outlineVao);
        outlineVao = 0;
    }
    if (ebo != 0)
    {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
    if (vbo != 0)
    {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (vao != 0)
    {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }

    indexCount = 0;
    outlineIndexCount = 0;
}

void Mesh::Draw() const
{
    glBindVertexArray(vao);
    glDrawElements(primitive, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::DrawInstanced(GLsizei instanceCount, bool outline, GLuint baseInstance) const
{
    glBindVertexArray(outline ? outlineVao : vao);
    glDrawElementsInstancedBaseInstance(
        outline ? GL_LINES : primitive, outline ? outlineIndexCount : indexCount, GL_UNSIGNED_INT, nullptr, instanceCount,
        baseInstance
    );
    glBindVertexArray(0);
}

GLuint Mesh::GetVAO() const
{
    return vao;
}

GLuint Mesh::GetOutlineVAO() const
{
    return outlineVao;
}

static void BuildSurfaceGridOutline(
    std::vector<uint32>* lineIndices, int32 segments, int32 rows, bool collapseTop, bool collapseBottom
)
{
    MuliAssert(lineIndices != nullptr);

    lineIndices->clear();
    int32 lineCount = segments * (rows - 1 + rows - int32(collapseTop) - int32(collapseBottom));
    lineIndices->reserve(2 * lineCount);

    for (int32 row = 0; row + 1 < rows; ++row)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            uint32 a = uint32(row * (segments + 1) + segment);
            lineIndices->push_back(a);
            lineIndices->push_back(a + uint32(segments + 1));
        }
    }

    int32 firstRow = collapseTop ? 1 : 0;
    int32 lastRow = collapseBottom ? rows - 1 : rows;
    for (int32 row = firstRow; row < lastRow; ++row)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            uint32 a = uint32(row * (segments + 1) + segment);
            lineIndices->push_back(a);
            lineIndices->push_back(a + 1);
        }
    }
}

void BuildSphereMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, std::vector<uint32>* lineIndices, int32 segments, int32 rings
)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);
    MuliAssert(lineIndices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve((size_t)((segments + 1) * (rings + 1)));
    indices->reserve((size_t)(segments * rings * 6));

    for (int32 ring = 0; ring <= rings; ++ring)
    {
        const float v = (float)ring / (float)rings;
        const float theta = v * pi;

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;

            const Vec3 normal{
                std::sin(theta) * std::cos(phi),
                std::cos(theta),
                std::sin(theta) * std::sin(phi),
            };

            vertices->push_back(MeshVertex{ normal, normal, Vec2{ u, v } });
        }
    }

    for (int32 ring = 0; ring < rings; ++ring)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            const uint32 a = (uint32)(ring * (segments + 1) + segment);
            const uint32 b = a + (uint32)(segments + 1);
            const uint32 c = a + 1;
            const uint32 d = b + 1;

            indices->push_back(a);
            indices->push_back(c);
            indices->push_back(b);

            indices->push_back(c);
            indices->push_back(d);
            indices->push_back(b);
        }
    }

    BuildSurfaceGridOutline(lineIndices, segments, rings + 1, true, true);
}

void BuildCapsuleMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings, float height, float inRadius
)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve((size_t)((segments + 1) * (rings * 2 + 2)));
    indices->reserve((size_t)(segments * (rings * 2 + 1) * 6));

    const float halfHeight = height * 0.5f;
    const int32 rowCount = rings * 2 + 2;

    for (int32 ring = 0; ring <= rings; ++ring)
    {
        const float theta = (float)ring / (float)rings * pi * 0.5f;
        const float y = std::cos(theta);
        const float radius = std::sin(theta);

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;
            const Vec3 normal{ radius * std::cos(phi), y, radius * std::sin(phi) };
            vertices->push_back(
                MeshVertex{ Vec3{ inRadius * normal.x, halfHeight + inRadius * normal.y, inRadius * normal.z }, normal,
                            Vec2{ u, (float)ring / (float)(rowCount - 1) } }
            );
        }
    }

    for (int32 segment = 0; segment <= segments; ++segment)
    {
        const float u = (float)segment / (float)segments;
        const float phi = u * 2.0f * pi;
        const Vec3 normal{ std::cos(phi), 0.0f, std::sin(phi) };
        vertices->push_back(
            MeshVertex{ Vec3{ inRadius * normal.x, -halfHeight, inRadius * normal.z }, normal,
                        Vec2{ u, (float)(rings + 1) / (float)(rowCount - 1) } }
        );
    }

    for (int32 ring = 1; ring <= rings; ++ring)
    {
        const float theta = pi * 0.5f + (float)ring / (float)rings * pi * 0.5f;
        const float y = std::cos(theta);
        const float radius = std::sin(theta);

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;
            const Vec3 normal{ radius * std::cos(phi), y, radius * std::sin(phi) };
            vertices->push_back(
                MeshVertex{ Vec3{ inRadius * normal.x, -halfHeight + inRadius * normal.y, inRadius * normal.z }, normal,
                            Vec2{ u, (float)(rings + 1 + ring) / (float)(rowCount - 1) } }
            );
        }
    }

    for (int32 row = 0; row < rowCount - 1; ++row)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            const uint32 a = (uint32)(row * (segments + 1) + segment);
            const uint32 b = a + (uint32)(segments + 1);
            const uint32 c = a + 1;
            const uint32 d = b + 1;

            indices->push_back(a);
            indices->push_back(c);
            indices->push_back(b);

            indices->push_back(c);
            indices->push_back(d);
            indices->push_back(b);
        }
    }
}

void BuildCapsuleTopMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, std::vector<uint32>* lineIndices, int32 segments, int32 rings
)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);
    MuliAssert(lineIndices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve((size_t)((segments + 1) * (rings + 1)));
    indices->reserve((size_t)(segments * rings * 6));

    for (int32 ring = 0; ring <= rings; ++ring)
    {
        const float v = (float)ring / (float)rings;
        const float theta = v * pi * 0.5f;
        const float y = std::cos(theta);
        const float radius = std::sin(theta);

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;
            Vec3 normal{ radius * std::cos(phi), y, radius * std::sin(phi) };
            vertices->push_back(MeshVertex{ normal, normal, Vec2{ u, v } });
        }
    }

    for (int32 ring = 0; ring < rings; ++ring)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            const uint32 a = (uint32)(ring * (segments + 1) + segment);
            const uint32 b = a + (uint32)(segments + 1);
            const uint32 c = a + 1;
            const uint32 d = b + 1;

            indices->push_back(a);
            indices->push_back(c);
            indices->push_back(b);

            indices->push_back(c);
            indices->push_back(d);
            indices->push_back(b);
        }
    }

    BuildSurfaceGridOutline(lineIndices, segments, rings + 1, true, false);
}

void BuildCapsuleBottomMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, std::vector<uint32>* lineIndices, int32 segments, int32 rings
)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);
    MuliAssert(lineIndices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve((size_t)((segments + 1) * (rings + 1)));
    indices->reserve((size_t)(segments * rings * 6));

    for (int32 ring = 0; ring <= rings; ++ring)
    {
        const float v = (float)ring / (float)rings;
        const float theta = pi * 0.5f + v * pi * 0.5f;
        const float y = std::cos(theta);
        const float radius = std::sin(theta);

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;
            Vec3 normal{ radius * std::cos(phi), y, radius * std::sin(phi) };
            vertices->push_back(MeshVertex{ normal, normal, Vec2{ u, v } });
        }
    }

    for (int32 ring = 0; ring < rings; ++ring)
    {
        for (int32 segment = 0; segment < segments; ++segment)
        {
            const uint32 a = (uint32)(ring * (segments + 1) + segment);
            const uint32 b = a + (uint32)(segments + 1);
            const uint32 c = a + 1;
            const uint32 d = b + 1;

            indices->push_back(a);
            indices->push_back(c);
            indices->push_back(b);

            indices->push_back(c);
            indices->push_back(d);
            indices->push_back(b);
        }
    }

    BuildSurfaceGridOutline(lineIndices, segments, rings + 1, false, true);
}

void BuildCapsuleMidMesh(
    std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, std::vector<uint32>* lineIndices, int32 segments
)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);
    MuliAssert(lineIndices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve((size_t)((segments + 1) * 2));
    indices->reserve((size_t)(segments * 6));

    for (int32 yIndex = 0; yIndex < 2; ++yIndex)
    {
        const float y = yIndex == 0 ? 1.0f : -1.0f;
        const float v = (float)yIndex;

        for (int32 segment = 0; segment <= segments; ++segment)
        {
            const float u = (float)segment / (float)segments;
            const float phi = u * 2.0f * pi;
            Vec3 normal{ std::cos(phi), 0.0f, std::sin(phi) };
            vertices->push_back(MeshVertex{ Vec3{ normal.x, y, normal.z }, normal, Vec2{ u, v } });
        }
    }

    for (int32 segment = 0; segment < segments; ++segment)
    {
        const uint32 a = (uint32)segment;
        const uint32 b = a + (uint32)(segments + 1);
        const uint32 c = a + 1;
        const uint32 d = b + 1;

        indices->push_back(a);
        indices->push_back(c);
        indices->push_back(b);

        indices->push_back(c);
        indices->push_back(d);
        indices->push_back(b);
    }

    lineIndices->clear();
    lineIndices->reserve(2 * segments);
    for (int32 segment = 0; segment < segments; ++segment)
    {
        lineIndices->push_back(uint32(segment));
        lineIndices->push_back(uint32(segment + segments + 1));
    }
}

void BuildBoxMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, std::vector<uint32>* lineIndices)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);
    MuliAssert(lineIndices != nullptr);

    *vertices = {
        MeshVertex{ Vec3{ -1.0f, -1.0f, -1.0f }, Vec3{ 0.0f, 0.0f, -1.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, -1.0f, -1.0f }, Vec3{ 0.0f, 0.0f, -1.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 1.0f, -1.0f }, Vec3{ 0.0f, 0.0f, -1.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ -1.0f, 1.0f, -1.0f }, Vec3{ 0.0f, 0.0f, -1.0f }, Vec2{ 0.0f, 1.0f } },

        MeshVertex{ Vec3{ -1.0f, -1.0f, 1.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, -1.0f, 1.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 1.0f, 1.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ -1.0f, 1.0f, 1.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, Vec2{ 0.0f, 1.0f } },

        MeshVertex{ Vec3{ -1.0f, -1.0f, -1.0f }, Vec3{ -1.0f, 0.0f, 0.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ -1.0f, 1.0f, -1.0f }, Vec3{ -1.0f, 0.0f, 0.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ -1.0f, 1.0f, 1.0f }, Vec3{ -1.0f, 0.0f, 0.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ -1.0f, -1.0f, 1.0f }, Vec3{ -1.0f, 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f } },

        MeshVertex{ Vec3{ 1.0f, -1.0f, -1.0f }, Vec3{ 1.0f, 0.0f, 0.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, -1.0f, 1.0f }, Vec3{ 1.0f, 0.0f, 0.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 1.0f, 1.0f }, Vec3{ 1.0f, 0.0f, 0.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ 1.0f, 1.0f, -1.0f }, Vec3{ 1.0f, 0.0f, 0.0f }, Vec2{ 0.0f, 1.0f } },

        MeshVertex{ Vec3{ -1.0f, -1.0f, -1.0f }, Vec3{ 0.0f, -1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ -1.0f, -1.0f, 1.0f }, Vec3{ 0.0f, -1.0f, 0.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, -1.0f, 1.0f }, Vec3{ 0.0f, -1.0f, 0.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ 1.0f, -1.0f, -1.0f }, Vec3{ 0.0f, -1.0f, 0.0f }, Vec2{ 0.0f, 1.0f } },

        MeshVertex{ Vec3{ -1.0f, 1.0f, -1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 1.0f, -1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 1.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ -1.0f, 1.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 1.0f } },
    };

    *indices = {
        0,  2,  1,  0,  3,  2,  4,  5,  6,  4,  6,  7,  8,  10, 9,  8,  11, 10,
        12, 14, 13, 12, 15, 14, 16, 18, 17, 16, 19, 18, 20, 22, 21, 20, 23, 22,
    };

    *lineIndices = {
        0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7,
    };
}

void BuildGridMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 halfExtent, float spacing)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve((size_t)((halfExtent * 2 + 1) * 4));

    for (int32 i = -halfExtent; i <= halfExtent; ++i)
    {
        const float value = (float)i * spacing;

        vertices->push_back(
            MeshVertex{ Vec3{ value, 0.0f, -halfExtent * spacing }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } }
        );
        vertices->push_back(
            MeshVertex{ Vec3{ value, 0.0f, halfExtent * spacing }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } }
        );

        vertices->push_back(
            MeshVertex{ Vec3{ -halfExtent * spacing, 0.0f, value }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } }
        );
        vertices->push_back(
            MeshVertex{ Vec3{ halfExtent * spacing, 0.0f, value }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } }
        );
    }

    const int32 lineCount = halfExtent * 2 + 1;
    indices->reserve((size_t)(lineCount * 4));

    uint32 vertex = 0;
    for (int32 i = 0; i < lineCount; ++i)
    {
        indices->push_back(vertex + 0);
        indices->push_back(vertex + 1);
        indices->push_back(vertex + 2);
        indices->push_back(vertex + 3);
        vertex += 4;
    }
}

void BuildPlaneMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    *vertices = {
        MeshVertex{ Vec3{ -1.0f, 0.0f, -1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, -1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 1.0f, 1.0f } },
        MeshVertex{ Vec3{ -1.0f, 0.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Vec2{ 0.0f, 1.0f } },
    };

    *indices = { 0, 2, 1, 0, 3, 2 };
}

void BuildConvexMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    const ConvexShape& shape
)
{
    struct Edge
    {
        uint32 a;
        uint32 b;
        Vec3 normal;
        bool visible;
    };

    MuliAssert(vertices != nullptr);
    MuliAssert(triangleIndices != nullptr);
    MuliAssert(lineIndices != nullptr);

    vertices->clear();
    triangleIndices->clear();
    lineIndices->clear();

    std::span<const Face> faces = shape.GetFaces();
    std::span<const int32> faceIndices = shape.GetIndices();

    vertices->reserve(shape.GetIndices().size());
    triangleIndices->reserve(shape.GetIndices().size() * 3);
    lineIndices->reserve(shape.GetIndices().size() * 2);
    std::vector<Edge> edges;
    std::unordered_map<uint64, int32> edgeMap;
    edges.reserve(shape.GetIndices().size());
    edgeMap.reserve(shape.GetIndices().size());

    for (int32 i = 0; i < int32(faces.size()); ++i)
    {
        const Face& face = faces[i];
        const Vec3 normal = face.normal;
        uint32 base = uint32(vertices->size());

        for (int32 j = 0; j < face.vertexCount; ++j)
        {
            Vec3 position = shape.GetVertex(faceIndices[face.vertexStart + j]);
            Vec2 uv{ position.x, position.z };
            vertices->push_back(MeshVertex{ position, normal, uv });
        }

        for (int32 j = 1; j + 1 < face.vertexCount; ++j)
        {
            triangleIndices->push_back(base);
            triangleIndices->push_back(base + uint32(j));
            triangleIndices->push_back(base + uint32(j + 1));
        }

        for (int32 j = 0; j < face.vertexCount; ++j)
        {
            int32 next = (j + 1) % face.vertexCount;
            uint32 id0 = uint32(faceIndices[face.vertexStart + j]);
            uint32 id1 = uint32(faceIndices[face.vertexStart + next]);
            uint32 minId = Min(id0, id1);
            uint32 maxId = Max(id0, id1);
            uint64 edge = (uint64(minId) << 32) | maxId;
            auto it = edgeMap.find(edge);
            if (it == edgeMap.end())
            {
                edgeMap.emplace(edge, int32(edges.size()));
                edges.push_back(Edge{ base + uint32(j), base + uint32(next), normal, true });
            }
            else if (Dot(edges[it->second].normal, normal) > 1.0f - 1e-4f)
            {
                edges[it->second].visible = false;
            }
        }
    }

    for (const Edge& edge : edges)
    {
        if (edge.visible)
        {
            lineIndices->push_back(edge.a);
            lineIndices->push_back(edge.b);
        }
    }
}

void BuildPolygonMesh(
    std::vector<MeshVertex>* vertices,
    std::vector<uint32>* triangleIndices,
    std::vector<uint32>* lineIndices,
    const PolygonShape& shape
)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(triangleIndices != nullptr);
    MuliAssert(lineIndices != nullptr);

    vertices->clear();
    triangleIndices->clear();
    lineIndices->clear();

    int32 count = shape.GetVertexCount();
    Vec3 normal = shape.GetNormal();
    Vec3 tangent, bitangent;
    CoordinateSystem(normal, &tangent, &bitangent);

    Vec2 uvMin{ max_float };
    Vec2 uvMax{ -max_float };
    std::vector<Vec2> uvs;
    uvs.reserve(count);
    for (int32 i = 0; i < count; ++i)
    {
        Vec3 vertex = shape.GetVertex(i);
        Vec2 uv{ Dot(vertex, tangent), Dot(vertex, bitangent) };
        uvs.push_back(uv);
        uvMin = Min(uvMin, uv);
        uvMax = Max(uvMax, uv);
    }

    Vec2 uvExtent = uvMax - uvMin;
    uvExtent.x = Max(uvExtent.x, epsilon);
    uvExtent.y = Max(uvExtent.y, epsilon);

    vertices->reserve(2 * count);
    for (int32 i = 0; i < count; ++i)
    {
        Vec2 uv{ (uvs[i].x - uvMin.x) / uvExtent.x, (uvs[i].y - uvMin.y) / uvExtent.y };
        vertices->push_back(MeshVertex{ shape.GetVertex(i), normal, uv });
    }
    for (int32 i = 0; i < count; ++i)
    {
        Vec2 uv{ (uvs[i].x - uvMin.x) / uvExtent.x, (uvs[i].y - uvMin.y) / uvExtent.y };
        vertices->push_back(MeshVertex{ shape.GetVertex(i), -normal, uv });
    }

    triangleIndices->reserve(6 * (count - 2));
    for (int32 i = 1; i + 1 < count; ++i)
    {
        triangleIndices->push_back(0);
        triangleIndices->push_back(i);
        triangleIndices->push_back(i + 1);

        triangleIndices->push_back(count);
        triangleIndices->push_back(count + i + 1);
        triangleIndices->push_back(count + i);
    }

    lineIndices->reserve(2 * count);
    for (int32 i = 0; i < count; ++i)
    {
        lineIndices->push_back(i);
        lineIndices->push_back((i + 1) % count);
    }
}

void BuildTriangleMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, std::vector<uint32>* lineIndices)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);
    MuliAssert(lineIndices != nullptr);

    *vertices = {
        MeshVertex{ Vec3{ 0.0f, 0.0f, 0.0f }, z_axis, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, 0.0f }, z_axis, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 0.0f, 1.0f, 0.0f }, z_axis, Vec2{ 0.0f, 1.0f } },
        MeshVertex{ Vec3{ 0.0f, 0.0f, 0.0f }, -z_axis, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 0.0f, 1.0f, 0.0f }, -z_axis, Vec2{ 0.0f, 1.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, 0.0f }, -z_axis, Vec2{ 1.0f, 0.0f } },
    };

    *indices = { 0, 1, 2, 3, 4, 5 };
    *lineIndices = { 0, 1, 1, 2, 2, 0 };
}

void BuildHeightFieldMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const HeightFieldShape& shape)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    vertices->clear();
    indices->clear();

    int32 cellCountX = shape.GetCellCountX();
    int32 cellCountZ = shape.GetCellCountZ();
    vertices->reserve(size_t(cellCountX) * size_t(cellCountZ) * 6);
    indices->reserve(size_t(cellCountX) * size_t(cellCountZ) * 6);

    float uvScale = 0.4f;

    for (int32 z = 0; z < cellCountZ; ++z)
    {
        for (int32 x = 0; x < cellCountX; ++x)
        {
            for (int32 t = 0; t < 2; ++t)
            {
                Vec3 a, b, c;
                shape.GetTriangle(x, z, t, &a, &b, &c);
                Vec3 normal = Cross(b - a, c - a);
                normal.Normalize();

                uint32 base = uint32(vertices->size());
                vertices->push_back(MeshVertex{ a, normal, Vec2(a.x, a.z) * uvScale });
                vertices->push_back(MeshVertex{ b, normal, Vec2(b.x, b.z) * uvScale });
                vertices->push_back(MeshVertex{ c, normal, Vec2(c.x, c.z) * uvScale });

                indices->push_back(base);
                indices->push_back(base + 1);
                indices->push_back(base + 2);
            }
        }
    }
}

void BuildMeshShapeMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const MeshShape& shape)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    vertices->clear();
    indices->clear();
    vertices->reserve(size_t(shape.GetTriangleCount()) * 3);
    indices->reserve(size_t(shape.GetTriangleCount()) * 3);

    float uvScale = 0.4f;

    for (int32 triangle = 0; triangle < shape.GetTriangleCount(); ++triangle)
    {
        Vec3 a, b, c;
        shape.GetTriangle(triangle, &a, &b, &c);
        Vec3 normal = Normalize(Cross(b - a, c - a));
        uint32 base = uint32(vertices->size());
        vertices->push_back(MeshVertex{ a, normal, Vec2(a.x, a.z) * uvScale });
        vertices->push_back(MeshVertex{ b, normal, Vec2(b.x, b.z) * uvScale });
        vertices->push_back(MeshVertex{ c, normal, Vec2(c.x, c.z) * uvScale });
        indices->push_back(base);
        indices->push_back(base + 1);
        indices->push_back(base + 2);
    }
}

} // namespace muli3
