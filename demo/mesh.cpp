#include "mesh.h"
#include "muli3/convex_shape.h"
#include "muli3/height_field_shape.h"

namespace muli3
{

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

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, position)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, normal)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, uv)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}

void Mesh::Destroy()
{
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
}

void Mesh::Draw() const
{
    glBindVertexArray(vao);
    glDrawElements(primitive, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::DrawInstanced(GLsizei instanceCount) const
{
    glBindVertexArray(vao);
    glDrawElementsInstanced(primitive, indexCount, GL_UNSIGNED_INT, nullptr, instanceCount);
    glBindVertexArray(0);
}

GLuint Mesh::GetVAO() const
{
    return vao;
}

void BuildSphereMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

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

void BuildCapsuleTopMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

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
}

void BuildCapsuleBottomMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments, int32 rings)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

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
}

void BuildCapsuleMidMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, int32 segments)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

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
}

void BuildBoxMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

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

void BuildConvexMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices, const ConvexShape& shape)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    vertices->clear();
    indices->clear();

    std::span<const ConvexFace> faces = shape.GetFaces();
    std::span<const Vec3> normals = shape.GetFaceNormals();

    vertices->reserve(faces.size() * 3);
    indices->reserve(faces.size() * 3);

    for (int32 i = 0; i < int32(faces.size()); ++i)
    {
        const ConvexFace& face = faces[i];
        const Vec3 normal = normals[i];

        for (int32 j = 0; j < face.count; ++j)
        {
            Vec3 position = shape.GetVertex(face.indices[j]);
            Vec2 uv{ position.x, position.z };
            vertices->push_back(MeshVertex{ position, normal, uv });
            indices->push_back(uint32(indices->size()));
        }
    }
}

void BuildTriangleMesh(std::vector<MeshVertex>* vertices, std::vector<uint32>* indices)
{
    MuliAssert(vertices != nullptr);
    MuliAssert(indices != nullptr);

    *vertices = {
        MeshVertex{ Vec3{ 0.0f, 0.0f, 0.0f }, z_axis, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, 0.0f }, z_axis, Vec2{ 1.0f, 0.0f } },
        MeshVertex{ Vec3{ 0.0f, 1.0f, 0.0f }, z_axis, Vec2{ 0.0f, 1.0f } },
        MeshVertex{ Vec3{ 0.0f, 0.0f, 0.0f }, -z_axis, Vec2{ 0.0f, 0.0f } },
        MeshVertex{ Vec3{ 0.0f, 1.0f, 0.0f }, -z_axis, Vec2{ 0.0f, 1.0f } },
        MeshVertex{ Vec3{ 1.0f, 0.0f, 0.0f }, -z_axis, Vec2{ 1.0f, 0.0f } },
    };

    *indices = { 0, 1, 2, 3, 4, 5 };
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

} // namespace muli3
