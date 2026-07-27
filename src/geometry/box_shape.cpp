#include "muli3/box_shape.h"

namespace muli3
{

constexpr static Vec3 boxNormals[6] = {
    Vec3{ -1.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.0f, 0.0f },  Vec3{ 0.0f, -1.0f, 0.0f },
    Vec3{ 0.0f, 1.0f, 0.0f },  Vec3{ 0.0f, 0.0f, -1.0f }, Vec3{ 0.0f, 0.0f, 1.0f },
};

BoxShape::BoxShape(float width, float height, float depth, float inRadius, const Transform& transform)
    : Shape{ Shape::box, inRadius }
    , halfExtents{ width * 0.5f, height * 0.5f, depth * 0.5f }
    , rotation{ transform.q }
{
    center = transform.p;

    float x = halfExtents.x * 2.0f;
    float y = halfExtents.y * 2.0f;
    float z = halfExtents.z * 2.0f;
    volume = x * y * z + 2.0f * (x * y + y * z + z * x) * radius + pi * (x + y + z) * radius * radius +
             4.0f / 3.0f * pi * radius * radius * radius;
}

BoxShape::BoxShape(const BoxShape& other, const Transform& transform)
    : Shape{ Shape::box, other.radius }
    , halfExtents{ other.halfExtents }
    , rotation{ transform.q * other.rotation }
{
    center = Mul(transform, other.center);

    float x = halfExtents.x * 2.0f;
    float y = halfExtents.y * 2.0f;
    float z = halfExtents.z * 2.0f;
    volume = x * y * z + 2.0f * (x * y + y * z + z * x) * radius + pi * (x + y + z) * radius * radius +
             4.0f / 3.0f * pi * radius * radius * radius;
}

void BoxShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    outMassData->mass = density * volume;
    outMassData->centerOfMass = center;

    // BoxShape is the Minkowski sum of the core box and a sphere of radius r.
    //
    // The rounded box is decomposed into non-overlapping pieces:
    // core box, 6 face slabs, 12 edge quarter-cylinders, and 8 corner octants.
    //
    // Accumulate the second moment matrix M2 = \int x*x^T dV about the local box center.
    // Because the rounded box is symmetric around all local axes, off-diagonal terms vanish.
    // Inertia is computed from M2 at the end:
    //
    // I = \int (dot(x,x) Identity - x*x^T) dm = tr(M2) * Identity - M2

    float a = halfExtents.x;
    float b = halfExtents.y;
    float c = halfExtents.z;
    float r = radius;
    float r2 = r * r;
    float r3 = r2 * r;
    float r4 = r2 * r2;
    float r5 = r4 * r;

    // Core box.
    //
    // M2_x = \int_{-a}^{a} \int_{-b}^{b} \int_{-c}^{c} x^2 dz dy dx
    //      = (2a^3 / 3) * (2b) * (2c)
    //
    // The y/z components are the same with axes permuted.
    //
    Vec3 second{
        8.0f / 3.0f * a * a * a * b * c,
        8.0f / 3.0f * a * b * b * b * c,
        8.0f / 3.0f * a * b * c * c * c,
    };

    for (int32 axis = 0; axis < 3; ++axis)
    {
        float h0 = halfExtents[axis];
        float h1 = halfExtents[(axis + 1) % 3];
        float h2 = halfExtents[(axis + 2) % 3];

        // Face slabs for +/- axis.
        //
        // Let x0 be the slab axis and x1/x2 be the two in-plane axes.
        // The two slabs have:
        //
        // x0 in [h0, h0 + r] and [-h0 - r, -h0]
        // x1 in [-h1, h1]
        // x2 in [-h2, h2]
        //
        // M2_0 = 2 * (2h1) * (2h2) * \int_{h0}^{h0+r} x0^2 dx0
        //      = 8h1h2 * ((h0 + r)^3 - h0^3) / 3
        //
        // M2_1 = 2 * (2h2) * r * \int_{-h1}^{h1} x1^2 dx1
        //      = 8h1^3h2r / 3
        //
        // M2_2 is the same with h1/h2 swapped.
        {
            float m0 = 8.0f * h1 * h2 * ((h0 + r) * (h0 + r) * (h0 + r) - h0 * h0 * h0) / 3.0f;
            float m1 = 8.0f / 3.0f * h1 * h1 * h1 * h2 * r;
            float m2 = 8.0f / 3.0f * h1 * h2 * h2 * h2 * r;

            second[axis] += m0;
            second[(axis + 1) % 3] += m1;
            second[(axis + 2) % 3] += m2;
        }

        // Edge quarter-cylinders parallel to axis.
        //
        // There are four edges for the selected axis. For each edge:
        //
        // x0 = s, s in [-h0, h0]
        // x1 = +/-h1 + u
        // x2 = +/-h2 + v
        // (u, v) is inside one quarter disk: u >= 0, v >= 0, u^2 + v^2 <= r^2
        //
        // Over all four edges the x1/x2 signs cancel the odd terms, and:
        //
        // M2_0 = 4 * (pi*r^2 / 4) * \int_{-h0}^{h0} s^2 ds
        //      = 2*pi*r^2*h0^3 / 3
        //
        // M2_1 = length * 4 * \int_quarter (h1 + u)^2 dA
        //      = length * (pi*h1^2*r^2 + 8*h1*r^3/3 + pi*r^4/4)
        //
        // M2_2 is the same with h1/h2 swapped.
        {
            float length = 2.0f * h0;

            float m0 = 2.0f * pi * r2 * h0 * h0 * h0 / 3.0f;
            float m1 = length * (pi * h1 * h1 * r2 + 8.0f / 3.0f * h1 * r3 + pi * r4 / 4.0f);
            float m2 = length * (pi * h2 * h2 * r2 + 8.0f / 3.0f * h2 * r3 + pi * r4 / 4.0f);
            second[axis] += m0;
            second[(axis + 1) % 3] += m1;
            second[(axis + 2) % 3] += m2;
        }
    }

    // Corner octants.
    //
    // The 8 octants combine into one full sphere of radius r, with each octant
    // translated to a box corner. By symmetry:
    //
    // M2_x = a^2 * V_sphere + \int_sphere q_x^2 dV
    //      = a^2 * (4*pi*r^3/3) + 4*pi*r^5/15
    //
    // The y/z components are the same expression with b/c.
    float sphereVolume = 4.0f / 3.0f * pi * r3;
    float sphereSecond = 4.0f * pi * r5 / 15.0f;
    second.x += a * a * sphereVolume + sphereSecond;
    second.y += b * b * sphereVolume + sphereSecond;
    second.z += c * c * sphereVolume + sphereSecond;

    second *= density;

    Mat3 inertiaCenter{
        Vec3{ second.y + second.z, 0.0f, 0.0f },
        Vec3{ 0.0f, second.x + second.z, 0.0f },
        Vec3{ 0.0f, 0.0f, second.x + second.y },
    };
    Mat3 localRotation{ rotation };
    inertiaCenter = localRotation * inertiaCenter * localRotation.GetTranspose();

    float x = center.x;
    float y = center.y;
    float z = center.z;
    float m = outMassData->mass;

    outMassData->inertia = Mat3(
        inertiaCenter.ex + Vec3{ m * (y * y + z * z), -m * x * y, -m * x * z },
        inertiaCenter.ey + Vec3{ -m * y * x, m * (x * x + z * z), -m * y * z },
        inertiaCenter.ez + Vec3{ -m * z * x, -m * z * y, m * (x * x + y * y) }
    );
}

void BoxShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Quat worldOrientation = transform.q * rotation;
    Mat3 worldRotation{ worldOrientation };
    Vec3 worldCenter = Mul(transform, center);

    Vec3 e{
        Abs(worldRotation.ex.x) * halfExtents.x + Abs(worldRotation.ey.x) * halfExtents.y +
            Abs(worldRotation.ez.x) * halfExtents.z,
        Abs(worldRotation.ex.y) * halfExtents.x + Abs(worldRotation.ey.y) * halfExtents.y +
            Abs(worldRotation.ez.y) * halfExtents.z,
        Abs(worldRotation.ex.z) * halfExtents.x + Abs(worldRotation.ey.z) * halfExtents.y +
            Abs(worldRotation.ez.z) * halfExtents.z,
    };

    Vec3 r{ radius, radius, radius };
    *outAABB = AABB{ worldCenter - e - r, worldCenter + e + r };
}

Face BoxShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    Quat worldOrientation = transform.q * rotation;
    Vec3 localDir = worldOrientation.RotateInv(dir);

    int32 axis = 0;
    float maxProjection = Abs(localDir.x);
    if (Abs(localDir.y) > maxProjection)
    {
        axis = 1;
        maxProjection = Abs(localDir.y);
    }
    if (Abs(localDir.z) > maxProjection)
    {
        axis = 2;
    }

    int32 face = axis * 2 + (localDir[axis] > 0.0f ? 1 : 0);

    Face outFace{};
    outFace.vertexStart = uint16(face * 4);
    outFace.vertexCount = 4;
    outFace.normal = worldOrientation.Rotate(boxNormals[face]);
    return outFace;
}

bool BoxShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Transform boxTransform = Mul(transform, Transform{ center, rotation });
    Vec3 localQ = MulT(boxTransform, q);
    Vec3 clamped{
        Clamp(localQ.x, -halfExtents.x, halfExtents.x),
        Clamp(localQ.y, -halfExtents.y, halfExtents.y),
        Clamp(localQ.z, -halfExtents.z, halfExtents.z),
    };
    Vec3 delta = localQ - clamped;

    return Length2(delta) <= radius * radius;
}

Vec3 BoxShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Transform boxTransform = Mul(transform, Transform{ center, rotation });
    Vec3 localQ = MulT(boxTransform, q);
    Vec3 clamped{
        Clamp(localQ.x, -halfExtents.x, halfExtents.x),
        Clamp(localQ.y, -halfExtents.y, halfExtents.y),
        Clamp(localQ.z, -halfExtents.z, halfExtents.z),
    };
    Vec3 delta = localQ - clamped;

    float distance = delta.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(boxTransform, clamped + delta * radius);
}

bool BoxShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Transform boxTransform = Mul(transform, Transform{ center, rotation });
    RayCastInput localInput = input;
    localInput.from = MulT(boxTransform, input.from);
    localInput.to = MulT(boxTransform, input.to);

    Vec3 p1 = localInput.from;
    Vec3 p2 = localInput.to;
    Vec3 d = p2 - p1;

    if (-halfExtents.x <= p1.x && p1.x <= halfExtents.x && -halfExtents.y <= p1.y && p1.y <= halfExtents.y &&
        -halfExtents.z <= p1.z && p1.z <= halfExtents.z)
    {
        return false;
    }

    float near = 0.0f;
    float far = input.maxFraction;
    int32 axis = -1;
    float sign = 0.0f;

    for (int32 i = 0; i < 3; ++i)
    {
        float p = p1[i];
        float di = d[i];
        float min = -halfExtents[i];
        float max = halfExtents[i];

        if (Abs(di) <= epsilon)
        {
            if (p < min || p > max)
            {
                return false;
            }

            continue;
        }

        float invD = 1.0f / di;
        float t0 = (min - p) * invD;
        float t1 = (max - p) * invD;
        float normalSign = -1.0f;

        if (t1 < t0)
        {
            std::swap(t0, t1);
            normalSign = 1.0f;
        }

        if (t0 > near)
        {
            near = t0;
            axis = i;
            sign = normalSign;
        }
        far = Min(far, t1);

        if (far < near)
        {
            return false;
        }
    }

    if (axis < 0 || near < 0.0f || near > input.maxFraction)
    {
        return false;
    }

    Vec3 normal = Vec3::zero;
    normal[axis] = sign;
    output->fraction = near;
    output->normal = boxTransform.q.Rotate(normal);
    return true;
}

} // namespace muli3
