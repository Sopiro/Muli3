#pragma once

#include "bounding_box.h"

namespace muli3
{

struct MassData
{
    float mass = 0.0f;
    Mat3 inertia = Mat3{ 0.0f };
    Vec3 centerOfMass{ 0.0f, 0.0f, 0.0f };
};

struct Point
{
    Vec3 p{ 0.0f, 0.0f, 0.0f };
    int32 id = 0;
};

struct Face
{
    Point points[4];
    Vec3 normal{ 1.0f, 0.0f, 0.0f };
    int32 count = 0;
    int32 id = 0;
};

enum ShapeType
{
    // Order matters!
    sphere = 0,
    box,
    shape_count,
};

class Shape
{
public:
    Shape(ShapeType type, float radius);
    virtual ~Shape() = default;

    ShapeType GetType() const;

    float GetRadius() const;
    float GetVolume() const;
    const Vec3& GetCenter() const;

    const Vec3& GetCenterOfMass() const;
    virtual void ComputeMass(float density, MassData* outMassData) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;
    virtual Mat3 ComputeLocalInertiaTensor(float mass) const = 0;

    virtual int32 GetVertexCount() const = 0;
    virtual Vec3 GetVertex(int32 id) const = 0;
    virtual int32 GetSupport(const Vec3& localDir) const = 0;
    virtual int32 GetFaceCount() const = 0;
    virtual bool GetFace(int32 id, const Transform& transform, Face* outFace) const = 0;
    virtual bool GetFeaturedFace(const Transform& transform, const Vec3& dir, Face* outFace) const = 0;

    virtual bool TestPoint(const Transform& transform, const Vec3& q) const = 0;
    virtual Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const = 0;

private:
    ShapeType type;

protected:
    Vec3 center{ 0.0f, 0.0f, 0.0f };
    float radius = 0.0f;
    float volume = 0.0f;
};

inline Shape::Shape(ShapeType type, float radius)
    : type{ type }
    , radius{ radius }
{
}

inline ShapeType Shape::GetType() const
{
    return type;
}

inline float Shape::GetRadius() const
{
    return radius;
}

inline float Shape::GetVolume() const
{
    return volume;
}

inline const Vec3& Shape::GetCenter() const
{
    return center;
}

inline const Vec3& Shape::GetCenterOfMass() const
{
    return center;
}

} // namespace muli3
