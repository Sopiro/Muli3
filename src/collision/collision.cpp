#include "muli3/collision.h"
#include "muli3/distance.h"
#include "muli3/frame.h"
#include "muli3/ghost.h"
#include "muli3/growable_array.h"
#include "muli3/settings.h"
#include "muli3/shapes.h"

namespace muli3
{

bool detection_function_initialized = false;
CollideFunction* collide_function_map[Shape::shape_count][Shape::shape_count];

void InitializeDetectionFunctionMap();

inline SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec3& dir)
{
    SupportPoint supportPoint;
    supportPoint.pointA.id = a->GetSupport(tfA.q.RotateInv(dir));
    supportPoint.pointB.id = b->GetSupport(tfB.q.RotateInv(-dir));
    supportPoint.pointA.p = Mul(tfA, a->GetVertex(supportPoint.pointA.id));
    supportPoint.pointB.p = Mul(tfB, b->GetVertex(supportPoint.pointB.id));
    supportPoint.point = supportPoint.pointA.p - supportPoint.pointB.p;

    return supportPoint;
}

struct EPAFace
{
    int32 a, b, c;
    Vec3 normal;
    float distance;
    bool removed;
};

struct EPAEdge
{
    int32 a, b;
};

constexpr int32 epa_max_vertex_count = max_simplex_vertex_count + epa_max_iteration;
constexpr int32 epa_max_face_count = 4 + epa_max_iteration * 2;
constexpr int32 epa_max_edge_count = epa_max_face_count * 3;

using EPAFaces = GrowableArray<EPAFace, epa_max_face_count>;
using EPAEdges = GrowableArray<EPAEdge, epa_max_edge_count>;

static bool AddEPAFace(EPAFaces& faces, const SupportPoint* vertices, int32 a, int32 b, int32 c, const Vec3& inside)
{
    Vec3 pa = vertices[a].point;
    Vec3 pb = vertices[b].point;
    Vec3 pc = vertices[c].point;

    Vec3 normal = Cross(pb - pa, pc - pa);
    if (normal.Normalize() == 0.0f)
    {
        return false;
    }

    float distance = Dot(normal, pa);
    if (Dot(normal, inside) - distance > 0.0f)
    {
        std::swap(b, c);
        normal = -normal;
        distance = -distance;
    }

    faces.emplace_back(a, b, c, normal, distance, false);
    return true;
}

static void AddEPAEdge(EPAEdges& edges, int32 a, int32 b)
{
    for (int32 i = 0; i < edges.size(); ++i)
    {
        if (edges[i].a == b && edges[i].b == a)
        {
            std::swap(edges[i], edges.back());
            edges.pop_back();
            return;
        }
    }

    edges.emplace_back(a, b);
}

bool GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, GJKResult* result)
{
    Simplex simplex;

    Vec3 direction = tfB.p - tfA.p;
    SupportPoint support = CSOSupport(a, tfA, b, tfB, direction);
    simplex.AddVertex(support);

    Vec3 save[max_simplex_vertex_count];
    int32 saveCount;

    for (int32 k = 0; k < gjk_max_iteration; ++k)
    {
        simplex.Save(save, &saveCount);
        simplex.Advance(Vec3::zero);

        if (simplex.count == max_simplex_vertex_count)
        {
            break;
        }

        direction = simplex.GetSearchDirection();

        // Simplex contains the origin
        if (Dot(direction, direction) == 0.0f)
        {
            break;
        }

        support = CSOSupport(a, tfA, b, tfB, direction);

        // Check duplicate vertices
        for (int32 i = 0; i < saveCount; ++i)
        {
            if (save[i] == support.point)
            {
                goto end;
            }
        }

        simplex.AddVertex(support);
    }

end:
    Vec3 closest = simplex.GetClosestPoint();
    float distance = Length(closest);

    result->simplex = simplex;
    result->direction = Normalize(direction);
    result->distance = distance;

    return distance < gjk_tolerance;
}

void EPA(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Simplex& simplex, EPAResult* result)
{
    MuliAssert(simplex.count == max_simplex_vertex_count);

    // To properly handle the origin-on-faces case, the center of the tetrahedron is needed
    Vec3 center = Vec3::zero;
    int32 vertexCount = simplex.count;
    SupportPoint vertices[epa_max_vertex_count];
    for (int32 i = 0; i < vertexCount; ++i)
    {
        vertices[i] = simplex.vertices[i];
        center += vertices[i].point;
    }
    center *= 1.0f / max_simplex_vertex_count;

    EPAFaces faces;
    AddEPAFace(faces, vertices, 0, 1, 2, center);
    AddEPAFace(faces, vertices, 1, 2, 3, center);
    AddEPAFace(faces, vertices, 2, 3, 0, center);
    AddEPAFace(faces, vertices, 3, 0, 1, center);

    if (faces.size() != max_simplex_vertex_count)
    {
        // Degenerate tetrahedron case
        result->contactNormal = NormalizeSafe(tfB.p - tfA.p);
        result->penetrationDepth = 0.0f;
        return;
    }

    EPAFace best = faces[0];

    for (int32 k = 0; k < epa_max_iteration; ++k)
    {
        // Utilize heap for min distance query
        int32 bestIndex = -1;
        float bestDistance = max_float;
        for (int32 i = 0; i < faces.size(); ++i)
        {
            if (!faces[i].removed && faces[i].distance < bestDistance)
            {
                bestIndex = i;
                bestDistance = faces[i].distance;
            }
        }

        if (bestIndex == -1)
        {
            break;
        }

        best = faces[bestIndex];

        SupportPoint support = CSOSupport(a, tfA, b, tfB, best.normal);
        float newDistance = Dot(best.normal, support.point);
        if (newDistance - best.distance <= epa_tolerance)
        {
            break;
        }

        if (vertexCount >= epa_max_vertex_count)
        {
            break;
        }

        int32 newIndex = vertexCount;
        vertices[vertexCount++] = support;

        EPAEdges edges;

        for (int32 i = 0; i < faces.size(); ++i)
        {
            EPAFace& face = faces[i];
            if (face.removed)
            {
                continue;
            }

            if (Dot(face.normal, support.point - vertices[face.a].point) > 0.0f)
            {
                face.removed = true;
                AddEPAEdge(edges, face.a, face.b);
                AddEPAEdge(edges, face.b, face.c);
                AddEPAEdge(edges, face.c, face.a);
            }
        }

        for (int32 i = 0; i < edges.size(); ++i)
        {
            if (!AddEPAFace(faces, vertices, edges[i].a, edges[i].b, newIndex, center))
            {
                result->contactNormal = best.normal;
                result->penetrationDepth = best.distance;
                return;
            }
        }
    }

    result->contactNormal = best.normal;
    result->penetrationDepth = best.distance;
}

static inline void TranslateFace(Face* face, Vec3 d)
{
    for (int32 i = 0; i < max_face_vertices; ++i)
    {
        face->points[i].p += d;
    }
}

static Vec3 IntersectPlaneEdge(const Vec3& a, const Vec3& b, float da, float db)
{
    const float denom = da - db;
    if (Abs(denom) < epsilon)
    {
        return a;
    }

    float t = Clamp(da / denom, 0, 1);
    return a + t * (b - a);
}

struct ClippedFace
{
    int32 count = 0;
    Point points[2 * max_face_vertices];
};

static void ClipFace(ClippedFace* out, const ClippedFace& in, const Vec3& p, const Vec3& dir)
{
    out->count = 0;

    if (in.count == 0)
    {
        return;
    }

    Point p0 = in.points[in.count - 1];
    float d0 = Dot(p0.p - p, dir);
    bool inside0 = d0 >= -epsilon;

    for (int32 i = 0; i < in.count; ++i)
    {
        Point p1 = in.points[i];
        float d1 = Dot(p1.p - p, dir);
        bool inside1 = d1 >= -epsilon;

        if (inside0 && inside1)
        {
            out->points[out->count++] = p1;
        }
        else if (inside0 && !inside1)
        {
            Vec3 intersection = IntersectPlaneEdge(p0.p, p1.p, d0, d1);
            out->points[out->count++] = Point{ intersection, p0.id };
        }
        else if (!inside0 && inside1)
        {
            Vec3 intersection = IntersectPlaneEdge(p0.p, p1.p, d0, d1);
            out->points[out->count++] = Point{ intersection, p0.id };
            out->points[out->count++] = p1;
        }

        p0 = p1;
        d0 = d1;
        inside0 = inside1;
    }
}

static void FindContactPoints(
    const Vec3& n, const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold
)
{
    Face faceA = a->GetFeaturedFace(tfA, n);
    Face faceB = b->GetFeaturedFace(tfB, -n);

    TranslateFace(&faceA, faceA.normal * a->GetRadius());
    TranslateFace(&faceB, faceB.normal * b->GetRadius());

    Face ref; // Reference face
    Face inc; // Incident face
    bool flipped;

    float aParallelness = AbsDot(faceA.normal, n);
    float bParallelness = AbsDot(faceB.normal, n);

    if (std::min(faceA.count, faceB.count) < 3 && (faceB.count < faceA.count))
    {
        ref = faceA;
        inc = faceB;
        flipped = false;
    }
    else if (bParallelness > aParallelness)
    {
        ref = faceB;
        inc = faceA;
        flipped = true;
    }
    else
    {
        ref = faceA;
        inc = faceB;
        flipped = false;
    }

    Vec3 planeNormal = ref.normal;
    Vec3 planePoint = ref.points[0].p;

    ClippedFace faces[2];
    faces[0].count = inc.count;
    std::memcpy(faces[0].points, inc.points, inc.count * sizeof(Point));

    // ping pong indices
    int32 input = 0;
    int32 output = 1;

    for (int32 i0 = ref.count - 1, i1 = 0; i1 < ref.count; i0 = i1, ++i1)
    {
        Vec3 edge = ref.points[i1].p - ref.points[i0].p;
        Vec3 inward = Normalize(Cross(planeNormal, edge));

        ClipFace(&faces[output], faces[input], ref.points[i0].p, inward);

        if (faces[output].count == 0)
        {
            manifold->contactCount = 0;
            return;
        }

        std::swap(input, output);
    }

    // Keep only points that under the reference plane.
    faces[output].count = 0;
    for (int32 i = 0; i < faces[input].count; ++i)
    {
        float separation = Dot(faces[input].points[i].p - planePoint, planeNormal);
        if (separation < 0.0f)
        {
            faces[output].points[faces[output].count++] = faces[input].points[i];
        }
    }

    if (faces[output].count == 0)
    {
        manifold->contactCount = 0;
        return;
    }

    ContactPoint candidates[2 * max_face_vertices];
    for (int32 i = 0; i < faces[output].count; ++i)
    {
        Point point = faces[output].points[i];
        float separation = Dot(point.p - planePoint, planeNormal);

        Vec3 anchorA, anchorB;
        if (flipped)
        {
            anchorA = point.p;
            anchorB = point.p - planeNormal * separation;
        }
        else
        {
            anchorA = point.p - planeNormal * separation;
            anchorB = point.p;
        }

        candidates[i].p = (anchorA + anchorB) * 0.5f;
        candidates[i].anchorA = anchorA;
        candidates[i].anchorB = anchorB;
        candidates[i].normal = n;
    }

    if (faces[output].count <= max_contact_point_count)
    {
        for (int32 i = 0; i < faces[output].count; ++i)
        {
            manifold->contactPoints[i] = candidates[i];
            manifold->contactPoints[i].id = faceA.points[i].id;
        }

        manifold->contactCount = faces[output].count;
        return;
    }

    // Reduce contact points

    int32 indices[max_contact_point_count];
    int32 contactCount = 0;

    constexpr float minDepth2 = Sqr(linear_slop);

    Vec3 centerA = Mul(tfA, a->GetCenter());
    Vec3 projected[2 * max_face_vertices];
    float depth2[2 * max_face_vertices];

    // Work in the contact tangent plane around shape A.
    for (int32 i = 0; i < faces[output].count; ++i)
    {
        Vec3 r = candidates[i].anchorA - centerA;
        projected[i] = GramSchmidt(r, n);
        depth2[i] = Max(minDepth2, Length2(candidates[i].anchorB - candidates[i].anchorA));
    }

    // Start with the point that is farthest from the center and deepest
    int32 point1 = 0;
    float value = Max(minDepth2, Length2(projected[0])) * depth2[0];
    for (int32 i = 1; i < faces[output].count; ++i)
    {
        float v = Max(minDepth2, Length2(projected[i])) * depth2[i];
        if (v > value)
        {
            point1 = i;
            value = v;
        }
    }

    // Use the farthest weighted point from point1 as the main patch axis
    int32 point2 = -1;
    value = -max_float;
    for (int32 i = 0; i < faces[output].count; ++i)
    {
        if (i == point1)
        {
            continue;
        }

        float v = Max(minDepth2, Dist2(projected[i], projected[point1])) * depth2[i];
        if (v > value)
        {
            point2 = i;
            value = v;
        }
    }

    int32 point3 = -1;
    int32 point4 = -1;
    float minSide = 0.0f;
    float maxSide = 0.0f;
    Vec3 perp = Cross(projected[point2] - projected[point1], n);

    // Keep one point on each side of the main axis to maximize patch area
    for (int32 i = 0; i < faces[output].count; ++i)
    {
        if (i == point1 || i == point2)
        {
            continue;
        }

        float side = Dot(perp, projected[i] - projected[point1]);
        if (side < minSide)
        {
            point3 = i;
            minSide = side;
        }
        else if (side > maxSide)
        {
            point4 = i;
            maxSide = side;
        }
    }

    // Emit points in polygon order around the selected patch
    indices[contactCount++] = point1;
    if (point3 != -1)
    {
        indices[contactCount++] = point3;
    }
    indices[contactCount++] = point2;
    if (point4 != -1)
    {
        indices[contactCount++] = point4;
    }

    for (int32 i = 0; i < contactCount; ++i)
    {
        manifold->contactPoints[i] = candidates[indices[i]];
        manifold->contactPoints[i].id = faceA.points[i].id;
    }

    manifold->contactCount = contactCount;
}

bool SphereVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    Vec3 pa = Mul(transformA, a->GetCenter());
    Vec3 pb = Mul(transformB, b->GetCenter());
    Vec3 d = pb - pa;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    float distance2 = Length2(d);
    if (distance2 >= radii * radii)
    {
        return false;
    }

    float distance = SafeSqrt(distance2);
    Vec3 normal = distance > epsilon ? d / distance : Vec3{ 1.0f, 0.0f, 0.0f };
    if (distance <= epsilon)
    {
        distance = radii;
    }

    manifold->contactPoints[0].anchorA = pa + normal * ra;
    manifold->contactPoints[0].anchorB = pb - normal * rb;
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool CapsuleVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const CapsuleShape* capsule = (const CapsuleShape*)a;

    Vec3 pa = capsule->GetVertexA();
    Vec3 pb = capsule->GetVertexB();
    Vec3 ab = pb - pa;

    Vec3 centerB = Mul(transformB, b->GetCenter());
    Vec3 localP = MulT(transformA, centerB);

    float ab2 = Dot(ab, ab);
    float t = 0.0f;
    if (ab2 > epsilon)
    {
        t = Clamp(Dot(localP - pa, ab) / ab2, 0.0f, 1.0f);
    }

    Vec3 closest = pa + ab * t;
    Vec3 normal = localP - closest;
    float distance = normal.Normalize();

    if (distance <= epsilon)
    {
        Vec3 axis = NormalizeSafe(ab);
        if (Length2(axis) <= epsilon)
        {
            axis = y_axis;
        }

        normal = GramSchmidt(localP - capsule->GetCenter(), axis);
        if (normal.Normalize() == 0.0f)
        {
            CoordinateSystem(axis, &normal);
        }
    }

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (distance > radii)
    {
        return false;
    }

    normal = transformA.q.Rotate(normal);
    Point supportA;
    supportA.id = t <= linear_slop ? 0 : (t >= 1.0f - linear_slop ? 1 : 0);
    supportA.p = Mul(transformA, closest) + normal * ra;

    manifold->contactPoints[0].anchorA = supportA.p;
    manifold->contactPoints[0].anchorB = centerB - normal * rb;
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

extern Vec2 ClosestSegmentVsSegment(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1);

bool CapsuleVsCapsule(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const CapsuleShape* capsuleA = (const CapsuleShape*)a;
    const CapsuleShape* capsuleB = (const CapsuleShape*)b;

    Vec3 a0 = Mul(transformA, capsuleA->GetVertexA());
    Vec3 a1 = Mul(transformA, capsuleA->GetVertexB());
    Vec3 b0 = Mul(transformB, capsuleB->GetVertexA());
    Vec3 b1 = Mul(transformB, capsuleB->GetVertexB());

    Vec2 st = ClosestSegmentVsSegment(a0, a1, b0, b1);

    Vec3 pa = a0 + (a1 - a0) * st[0];
    Vec3 pb = b0 + (b1 - b0) * st[1];
    Vec3 normal = pb - pa;
    float distance = normal.Normalize();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (distance > radii)
    {
        return false;
    }

    if (distance <= epsilon)
    {
        // The closest segment points coincide. Pick a stable normal from crossed axes,
        // then fall back to the center delta projected off the capsule axis.
        Vec3 axisA = NormalizeSafe(a1 - a0);
        Vec3 axisB = NormalizeSafe(b1 - b0);
        normal = Cross(axisA, axisB);

        Vec3 deltaCenter = ((b0 + b1) - (a0 + a1)) * 0.5f;

        if (normal.Normalize() == 0.0f)
        {
            Vec3 axis = Length2(axisA) > epsilon ? axisA : (Length2(axisB) > epsilon ? axisB : y_axis);
            normal = GramSchmidt(deltaCenter, axis);
            if (normal.Normalize() == 0.0f)
            {
                CoordinateSystem(axis, &normal);
            }
        }

        if (Dot(normal, deltaCenter) < 0.0f)
        {
            normal = -normal;
        }
    }

    manifold->contactPoints[0].anchorA = pa + normal * ra;
    manifold->contactPoints[0].anchorB = pb - normal * rb;
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool BoxVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const BoxShape* box = (const BoxShape*)a;

    Vec3 c = Mul(transformB, b->GetCenter());
    Vec3 center = Mul(transformA, box->GetCenter());

    Vec3 v0 = Mul(transformA, box->GetVertex(0));
    Vec3 axes[3] = {
        Mul(transformA, box->GetVertex(1)) - v0,
        Mul(transformA, box->GetVertex(2)) - v0,
        Mul(transformA, box->GetVertex(4)) - v0,
    };

    float extents[3];
    for (int32 i = 0; i < 3; ++i)
    {
        extents[i] = axes[i].Normalize() * 0.5f;
    }

    Vec3 d = c - center;
    Vec3 closest = center;
    float projections[3];
    bool inside = true;
    for (int32 i = 0; i < 3; ++i)
    {
        projections[i] = Dot(d, axes[i]);
        float clamped = Clamp(projections[i], -extents[i], extents[i]);
        if (clamped != projections[i])
        {
            inside = false;
        }
        closest += axes[i] * clamped;
    }

    float radii = a->GetRadius() + b->GetRadius();
    Vec3 normal = c - closest;
    float separation = normal.Normalize();
    int32 contactID = 0;

    if (inside)
    {
        float minSeparation = max_float;
        int32 axisIndex = 0;
        float sign = 1.0f;

        for (int32 i = 0; i < 3; ++i)
        {
            float positive = extents[i] - projections[i];
            if (positive < minSeparation)
            {
                minSeparation = positive;
                axisIndex = i;
                sign = 1.0f;
            }

            float negative = extents[i] + projections[i];
            if (negative < minSeparation)
            {
                minSeparation = negative;
                axisIndex = i;
                sign = -1.0f;
            }
        }

        normal = axes[axisIndex] * sign;
        closest += normal * minSeparation;
        separation = -minSeparation;
        contactID = axisIndex * 2 + (sign > 0.0f ? 1 : 0);
    }
    else if (separation > radii)
    {
        return false;
    }
    else
    {
        for (int32 i = 0; i < 3; ++i)
        {
            if (projections[i] >= extents[i] - linear_slop)
            {
                contactID |= 1 << (i * 2);
            }
            else if (projections[i] <= -extents[i] + linear_slop)
            {
                contactID |= 1 << (i * 2 + 1);
            }
        }
    }

    if (separation <= epsilon && !inside)
    {
        normal = NormalizeSafe(c - center);
        if (Length2(normal) <= epsilon)
        {
            normal = Vec3{ 1.0f, 0.0f, 0.0f };
        }
    }

    manifold->contactPoints[0].anchorA = closest + normal * a->GetRadius();
    manifold->contactPoints[0].anchorB = c - normal * b->GetRadius();
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = contactID;
    manifold->contactCount = 1;

    return true;
}

bool BoxVsCapsule(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const BoxShape* boxA = (const BoxShape*)a;
    const CapsuleShape* capsuleB = (const CapsuleShape*)b;

    Transform tfBox = Mul(tfA, Transform{ boxA->GetCenter(), boxA->GetRotation() });

    Vec3 extentsA = boxA->GetHalfExtents();

    // Transform the capsule into the box's local space
    Vec3 p1 = MulT(tfBox, Mul(tfB, capsuleB->GetVertexA()));
    Vec3 p2 = MulT(tfBox, Mul(tfB, capsuleB->GetVertexB()));
    Vec3 d = p2 - p1;

    const Mat3 axesA = identity;

    float radii = boxA->GetRadius() + capsuleB->GetRadius();

    float minPenetration = max_float;
    Vec3 normal;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        // Project the box onto test axis
        float pA = extentsA.x * Abs(n.x) + extentsA.y * Abs(n.y) + extentsA.z * Abs(n.z);

        float proj1 = Dot(p1, n);
        float proj2 = Dot(p2, n);

        // Project capsule segment onto test axis
        float minB, maxB;
        if (proj1 < proj2)
        {
            minB = proj1;
            maxB = proj2;
        }
        else
        {
            minB = proj2;
            maxB = proj1;
        }

        // The box projection interval is [-pA, pA],
        // The capsule segment projection interval is [minB, maxB].
        //
        // Max(-maxB, minB) represents the signed separation between the two projected intervals.
        float separation = pA + radii - Max(-maxB, minB);

        // Found separation axis
        if (separation < 0.0f)
        {
            return false;
        }

        // Keep the axis with the minimum overlap
        if (separation < minPenetration)
        {
            minPenetration = separation;
            normal = n;

            // Ensure the normal point from the box toward the capsule
            if ((minB + maxB) * 0.5f < 0.0f)
            {
                normal = -normal;
            }
        }

        return true;
    };

    // Test the 3 face normals of the box
    for (int32 i = 0; i < 3; ++i)
    {
        if (!TestAxis(axesA[i]))
        {
            return false;
        }
    }

    // Test 3 edge-edge separating axis candidates
    for (int32 i = 0; i < 3; ++i)
    {
        if (!TestAxis(Cross(axesA[i], d)))
        {
            return false;
        }
    }

    // Test the closest-point axis from the first capsule endpoint to the box.
    // Clamp p1 to the box AABB to get the closest point on the box.
    Vec3 qa = Clamp(p1, -extentsA, extentsA);

    // Use the direction from Qa to p1 as a separating-axis candidate
    if (!TestAxis(p1 - qa))
    {
        return false;
    }

    // Test the closest-point axis from the second capsule endpoint
    Vec3 qb = Clamp(p2, -extentsA, extentsA);
    if (!TestAxis(p2 - qb))
    {
        return false;
    }

    // Found overlap

    normal = tfBox.q.Rotate(normal);
    FindContactPoints(normal, a, tfA, b, tfB, manifold);

    return manifold->contactCount > 0;
}

bool BoxVsBox(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const BoxShape* boxA = (const BoxShape*)a;
    const BoxShape* boxB = (const BoxShape*)b;

    Vec3 centerA = Mul(tfA, boxA->GetCenter());
    Vec3 centerB = Mul(tfB, boxB->GetCenter());

    Quat qA = tfA.q * boxA->GetRotation();
    Vec3 axesA[3] = { qA.Rotate(x_axis), qA.Rotate(y_axis), qA.Rotate(z_axis) };

    Quat qB = tfB.q * boxB->GetRotation();
    Vec3 axesB[3] = { qB.Rotate(x_axis), qB.Rotate(y_axis), qB.Rotate(z_axis) };

    Vec3 extentsA = boxA->GetHalfExtents();
    Vec3 extentsB = boxB->GetHalfExtents();

    Vec3 dir = centerB - centerA;

    float radii = boxA->GetRadius() + boxB->GetRadius();

    // Track the axis with the minimum penetration
    float minPenetration = max_float;
    Vec3 normal;

    auto TestAxis = [&](Vec3 n) -> bool {
        // Ignore degenerate axes
        // This can happen when two tested edges are nearly parallel
        if (n.Normalize() == 0)
        {
            return true;
        }

        // Project each box onto test axis
        float pa = extentsA.x * AbsDot(axesA[0], n) + extentsA.y * AbsDot(axesA[1], n) + extentsA.z * AbsDot(axesA[2], n);
        float pb = extentsB.x * AbsDot(axesB[0], n) + extentsB.y * AbsDot(axesB[1], n) + extentsB.z * AbsDot(axesB[2], n);

        // Distance between box centers projected onto axis n
        float d = AbsDot(dir, n);

        float separation = pa + pb + radii - d;

        // Found separation axis
        if (separation < 0)
        {
            return false;
        }

        // Keep the axis with the minimum overlap
        if (separation < minPenetration)
        {
            minPenetration = separation;
            normal = n;

            if (Dot(dir, n) < 0)
            {
                normal = -normal;
            }
        }

        return true;
    };

    // Test the 3 face normals of box A
    for (int32 i = 0; i < 3; ++i)
    {
        if (!TestAxis(axesA[i]))
        {
            return false;
        }
    }

    // Test the 3 face normals of box B
    for (int32 i = 0; i < 3; ++i)
    {
        if (!TestAxis(axesB[i]))
        {
            return false;
        }
    }

    // Test the 9 edge-edge axes formed by cross products
    // These axes are candidates for edge-edge separation
    for (int32 i = 0; i < 3; ++i)
    {
        for (int32 j = 0; j < 3; ++j)
        {
            if (!TestAxis(Cross(axesA[i], axesB[j])))
            {
                return false;
            }
        }
    }

    // Found overlap
    FindContactPoints(normal, a, tfA, b, tfB, manifold);

    return manifold->contactCount > 0;
}

bool ConvexVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const ConvexShape* convex = (const ConvexShape*)a;

    Vec3 centerB = Mul(transformB, b->GetCenter());
    Vec3 closest = a->GetClosestPoint(transformA, centerB);
    Vec3 normal = centerB - closest;
    float distance = normal.Normalize();

    float rb = b->GetRadius();
    if (distance > rb)
    {
        return false;
    }

    Vec3 localCenterB = MulT(transformA, centerB);
    int32 faceIndex = 0;
    float maxSeparation = Dot(convex->GetFaceNormals()[0], localCenterB - convex->GetVertex(convex->GetFaces()[0].indices[0]));

    for (int32 i = 1; i < int32(convex->GetFaces().size()); ++i)
    {
        float separation = Dot(convex->GetFaceNormals()[i], localCenterB - convex->GetVertex(convex->GetFaces()[i].indices[0]));
        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            faceIndex = i;
        }
    }

    if (distance <= epsilon)
    {
        normal = transformA.q.Rotate(convex->GetFaceNormals()[faceIndex]);
        distance = convex->GetRadius() - maxSeparation;
        closest = centerB - normal * distance;
    }

    manifold->contactPoints[0].anchorA = closest;
    manifold->contactPoints[0].anchorB = centerB - normal * rb;
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool ConvexVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    GJKResult gjkResult;
    bool collide = GJK(a, tfA, b, tfB, &gjkResult);

    Simplex& simplex = gjkResult.simplex;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    Vec3 normal;

    if (collide == false)
    {
        MuliAssert(simplex.count < max_simplex_vertex_count);

        if (gjkResult.distance >= radii)
        {
            return false;
        }

        switch (simplex.count)
        {
        case 1: // vertex vs. vertex collision
        {
            Vec3 normal = Normalize(-simplex.vertices[0].point);

            Point supportA = simplex.vertices[0].pointA;
            Point supportB = simplex.vertices[0].pointB;
            supportA.p += normal * ra;
            supportB.p -= normal * rb;

            manifold->contactPoints[0].anchorA = supportA.p;
            manifold->contactPoints[0].anchorB = supportB.p;
            manifold->contactPoints[0].p = (supportA.p + supportB.p) * 0.5f;
            manifold->contactPoints[0].normal = normal;
            manifold->contactPoints[0].id = 0;
            manifold->contactCount = 1;

            return true;
        }
        case 2: // vertex vs. edge collision
        {
            Vec3 edge = Normalize(simplex.vertices[1].point - simplex.vertices[0].point);
            normal = Normalize(GramSchmidt(-simplex.vertices[0].point, edge));

            break;
        }
        case 3: // vertex vs. face collision
        {
            Vec3 edgeA = simplex.vertices[1].point - simplex.vertices[0].point;
            Vec3 edgeB = simplex.vertices[2].point - simplex.vertices[0].point;
            normal = Normalize(Cross(edgeA, edgeB));

            Vec3 k = -simplex.vertices[0].point;
            if (Dot(normal, k) < 0)
            {
                normal = -normal;
            }

            break;
        }
        }
    }
    else
    {
        // Expand to a full simplex if the gjk termination simplex has fewer vertices.
        // Origin-on-line cases need perpendicular fallback directions to avoid a degenerate tetrahedron.
        switch (simplex.count)
        {
        case 1:
        {
            Vec3 d = Normalize(tfA.p - tfB.p);
            SupportPoint support = CSOSupport(a, tfA, b, tfB, d);
            if (support.point == simplex.vertices[0].point)
            {
                support = CSOSupport(a, tfA, b, tfB, -d);
            }

            simplex.AddVertex(support);
        }

            [[fallthrough]];

        case 2:
        {
            Vec3 edge = Normalize(simplex.vertices[1].point - simplex.vertices[0].point);
            Vec3 normal = GramSchmidt(-simplex.vertices[0].point, edge);
            SupportPoint support = CSOSupport(a, tfA, b, tfB, normal);
            if (support.point == simplex.vertices[0].point || support.point == simplex.vertices[1].point)
            {
                support = CSOSupport(a, tfA, b, tfB, -normal);
            }

            simplex.AddVertex(support);
        }

            [[fallthrough]];

        case 3:
        {
            Vec3 edge1 = simplex.vertices[1].point - simplex.vertices[0].point;
            Vec3 edge2 = simplex.vertices[2].point - simplex.vertices[0].point;

            Vec3 normal = Cross(edge1, edge2);
            normal.Normalize();

            SupportPoint support = CSOSupport(a, tfA, b, tfB, normal);
            if (support.point == simplex.vertices[0].point || support.point == simplex.vertices[1].point ||
                support.point == simplex.vertices[2].point)
            {
                support = CSOSupport(a, tfA, b, tfB, -normal);
            }

            simplex.AddVertex(support);
        }
        default:
            MuliAssert(simplex.count == max_simplex_vertex_count);
        }

        EPAResult epaResult;
        EPA(a, tfA, b, tfB, simplex, &epaResult);

        normal = epaResult.contactNormal;
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);

    return manifold->contactCount > 0;
}

bool TriangleVsSphere(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangle = (const TriangleShape*)a;

    Vec3 p = Mul(tfB, b->GetCenter());
    Vec3 localP = MulT(tfA, p);

    const Vec3* vertices = triangle->GetVertices();

    Vec3 triangleNormal = triangle->GetNormal();
    Vec3 closest = ClosestPointVsTriangle(localP, vertices[0], vertices[1], vertices[2]);

    Vec3 normal = localP - closest;
    float distance = normal.Normalize();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    if (distance > ra + rb)
    {
        return false;
    }

    float separation = Dot(localP - vertices[0], triangleNormal);
    if (distance <= epsilon)
    {
        normal = separation < 0.0f ? -triangleNormal : triangleNormal;
    }

    normal = tfA.q.Rotate(normal);
    manifold->contactPoints[0].anchorA = Mul(tfA, closest) + normal * ra;
    manifold->contactPoints[0].anchorB = p - normal * rb;
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool TriangleVsCapsule(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangle = (const TriangleShape*)a;
    const CapsuleShape* capsule = (const CapsuleShape*)b;

    Vec3 triangleNormal = tfA.q.Rotate(triangle->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangle->GetVertex(0)),
        Mul(tfA, triangle->GetVertex(1)),
        Mul(tfA, triangle->GetVertex(2)),
    };

    // Shift along the triangle normal to keep plane projections near zero.
    float planeOffset = Dot(verticesA[0], triangleNormal);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= triangleNormal * planeOffset;
    }

    Vec3 pointsB[2] = {
        Mul(tfB, capsule->GetVertexA()) - triangleNormal * planeOffset,
        Mul(tfB, capsule->GetVertexB()) - triangleNormal * planeOffset,
    };

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = triangleNormal;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 3; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(pointsB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 2; ++i)
        {
            float value = Dot(pointsB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            Vec3 axisNormal = positive ? n : -n;
            normal = axisNormal;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(triangleNormal))
    {
        return false;
    }

    Vec3 segment = pointsB[1] - pointsB[0];
    // Triangle edge vs capsule axis.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 edge = verticesA[(i + 1) % 3] - verticesA[i];
        if (!TestAxis(Cross(edge, segment)))
        {
            return false;
        }
    }

    // Capsule caps against triangle interior.
    for (int32 i = 0; i < 2; ++i)
    {
        Vec3 closest = ClosestPointVsTriangle(pointsB[i], verticesA[0], verticesA[1], verticesA[2]);
        if (!TestAxis(pointsB[i] - closest))
        {
            return false;
        }
    }

    // Triangle vertices against the capsule segment.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 closest = ClosestPointVsSegment(verticesA[i], pointsB[0], pointsB[1]);
        if (!TestAxis(closest - verticesA[i]))
        {
            return false;
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool TriangleVsBox(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangle = (const TriangleShape*)a;
    const BoxShape* box = (const BoxShape*)b;

    Vec3 triangleNormal = tfA.q.Rotate(triangle->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangle->GetVertex(0)),
        Mul(tfA, triangle->GetVertex(1)),
        Mul(tfA, triangle->GetVertex(2)),
    };

    // Shift along the triangle normal to keep plane projections near zero.
    float planeOffset = Dot(verticesA[0], triangleNormal);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= triangleNormal * planeOffset;
    }

    Vec3 verticesB[8];
    for (int32 i = 0; i < 8; ++i)
    {
        verticesB[i] = Mul(tfB, box->GetVertex(i)) - triangleNormal * planeOffset;
    }

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = triangleNormal;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 3; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(verticesB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 8; ++i)
        {
            float value = Dot(verticesB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            Vec3 axisNormal = positive ? n : -n;
            normal = axisNormal;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(triangleNormal))
    {
        return false;
    }

    Quat qB = tfB.q * box->GetRotation();
    Vec3 axesB[3] = { qB.Rotate(x_axis), qB.Rotate(y_axis), qB.Rotate(z_axis) };

    // Box face normals.
    for (int32 i = 0; i < 3; ++i)
    {
        if (!TestAxis(axesB[i]))
        {
            return false;
        }
    }

    // Triangle edge vs box edge axes.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 3] - verticesA[i];
        for (int32 j = 0; j < 3; ++j)
        {
            if (!TestAxis(Cross(edgeA, axesB[j])))
            {
                return false;
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool TriangleVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    // would it be better to use GJK/EPA?
    const TriangleShape* triangle = (const TriangleShape*)a;
    const ConvexShape* convex = (const ConvexShape*)b;

    Vec3 triangleNormal = tfA.q.Rotate(triangle->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangle->GetVertex(0)),
        Mul(tfA, triangle->GetVertex(1)),
        Mul(tfA, triangle->GetVertex(2)),
    };

    // Shift along the triangle normal to keep plane projections near zero.
    float planeOffset = Dot(verticesA[0], triangleNormal);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= triangleNormal * planeOffset;
    }

    int32 vertexCountB = convex->GetVertexCount();
    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = triangleNormal;

    auto ProjectAxis = [&](Vec3 n, float* min, float* max) {
        Vec3 p = Mul(tfB, convex->GetVertex(0)) - triangleNormal * planeOffset;
        *min = Dot(p, n);
        *max = *min;
        for (int32 i = 1; i < vertexCountB; ++i)
        {
            p = Mul(tfB, convex->GetVertex(i)) - triangleNormal * planeOffset;
            float value = Dot(p, n);
            *min = Min(*min, value);
            *max = Max(*max, value);
        }
    };

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 3; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB, maxB;
        ProjectAxis(n, &minB, &maxB);

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            Vec3 axisNormal = positive ? n : -n;
            normal = axisNormal;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(triangleNormal))
    {
        return false;
    }

    // Convex face normals.
    for (Vec3 n : convex->GetFaceNormals())
    {
        if (!TestAxis(tfB.q.Rotate(n)))
        {
            return false;
        }
    }

    // Triangle edge vs convex edge axes.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 3] - verticesA[i];
        for (const ConvexFace& face : convex->GetFaces())
        {
            for (int32 j = 0; j < face.count; ++j)
            {
                int32 i0 = face.indices[j];
                int32 i1 = face.indices[(j + 1) % face.count];
                Vec3 p0 = Mul(tfB, convex->GetVertex(i0)) - triangleNormal * planeOffset;
                Vec3 p1 = Mul(tfB, convex->GetVertex(i1)) - triangleNormal * planeOffset;
                if (!TestAxis(Cross(edgeA, p1 - p0)))
                {
                    return false;
                }
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool TriangleVsTriangle(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangleA = (const TriangleShape*)a;
    const TriangleShape* triangleB = (const TriangleShape*)b;

    Vec3 normalA = tfA.q.Rotate(triangleA->GetNormal());
    Vec3 normalB = tfB.q.Rotate(triangleB->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangleA->GetVertex(0)),
        Mul(tfA, triangleA->GetVertex(1)),
        Mul(tfA, triangleA->GetVertex(2)),
    };
    Vec3 verticesB[3] = {
        Mul(tfB, triangleB->GetVertex(0)),
        Mul(tfB, triangleB->GetVertex(1)),
        Mul(tfB, triangleB->GetVertex(2)),
    };

    // Shift along triangle A's normal to keep plane projections near zero.
    float planeOffset = Dot(verticesA[0], normalA);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= normalA * planeOffset;
        verticesB[i] -= normalA * planeOffset;
    }

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = normalA;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 3; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(verticesB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 3; ++i)
        {
            float value = Dot(verticesB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            Vec3 axisNormal = positive ? n : -n;
            normal = axisNormal;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(normalA))
    {
        return false;
    }

    // Triangle B face normal.
    if (!TestAxis(normalB))
    {
        return false;
    }

    // Triangle edge vs triangle edge axes.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 3] - verticesA[i];
        for (int32 j = 0; j < 3; ++j)
        {
            Vec3 edgeB = verticesB[(j + 1) % 3] - verticesB[j];
            if (!TestAxis(Cross(edgeA, edgeB)))
            {
                return false;
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool TriangleVsQuad(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangle = (const TriangleShape*)a;
    const QuadShape* quad = (const QuadShape*)b;

    Vec3 normalA = tfA.q.Rotate(triangle->GetNormal());
    Vec3 normalB = tfB.q.Rotate(quad->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangle->GetVertex(0)),
        Mul(tfA, triangle->GetVertex(1)),
        Mul(tfA, triangle->GetVertex(2)),
    };
    Vec3 verticesB[4] = {
        Mul(tfB, quad->GetVertex(0)),
        Mul(tfB, quad->GetVertex(1)),
        Mul(tfB, quad->GetVertex(2)),
        Mul(tfB, quad->GetVertex(3)),
    };

    float planeOffset = Dot(verticesA[0], normalA);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= normalA * planeOffset;
    }
    for (int32 i = 0; i < 4; ++i)
    {
        verticesB[i] -= normalA * planeOffset;
    }

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = normalA;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 3; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(verticesB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 4; ++i)
        {
            float value = Dot(verticesB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            normal = positive ? n : -n;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(normalA))
    {
        return false;
    }

    if (!TestAxis(normalB))
    {
        return false;
    }

    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 3] - verticesA[i];
        for (int32 j = 0; j < 4; ++j)
        {
            Vec3 edgeB = verticesB[(j + 1) % 4] - verticesB[j];
            if (!TestAxis(Cross(edgeA, edgeB)))
            {
                return false;
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool QuadVsSphere(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const QuadShape* quad = (const QuadShape*)a;

    Vec3 p = Mul(tfB, b->GetCenter());
    Vec3 localP = MulT(tfA, p);

    const Vec3* vertices = quad->GetVertices();

    Vec3 quadNormal = quad->GetNormal();
    Vec3 closest0 = ClosestPointVsTriangle(localP, vertices[0], vertices[1], vertices[2]);
    Vec3 closest1 = ClosestPointVsTriangle(localP, vertices[0], vertices[2], vertices[3]);
    Vec3 closest = Dist2(localP, closest0) <= Dist2(localP, closest1) ? closest0 : closest1;

    Vec3 normal = localP - closest;
    float distance = normal.Normalize();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    if (distance > ra + rb)
    {
        return false;
    }

    float separation = Dot(localP - vertices[0], quadNormal);
    if (distance <= epsilon)
    {
        normal = separation < 0.0f ? -quadNormal : quadNormal;
    }

    normal = tfA.q.Rotate(normal);
    manifold->contactPoints[0].anchorA = Mul(tfA, closest) + normal * ra;
    manifold->contactPoints[0].anchorB = p - normal * rb;
    manifold->contactPoints[0].p = (manifold->contactPoints[0].anchorA + manifold->contactPoints[0].anchorB) * 0.5f;
    manifold->contactPoints[0].normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool QuadVsCapsule(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const QuadShape* quad = (const QuadShape*)a;
    const CapsuleShape* capsule = (const CapsuleShape*)b;

    Vec3 quadNormal = tfA.q.Rotate(quad->GetNormal());
    Vec3 verticesA[4] = {
        Mul(tfA, quad->GetVertex(0)),
        Mul(tfA, quad->GetVertex(1)),
        Mul(tfA, quad->GetVertex(2)),
        Mul(tfA, quad->GetVertex(3)),
    };

    float planeOffset = Dot(verticesA[0], quadNormal);
    for (int32 i = 0; i < 4; ++i)
    {
        verticesA[i] -= quadNormal * planeOffset;
    }

    Vec3 pointsB[2] = {
        Mul(tfB, capsule->GetVertexA()) - quadNormal * planeOffset,
        Mul(tfB, capsule->GetVertexB()) - quadNormal * planeOffset,
    };

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = quadNormal;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 4; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(pointsB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 2; ++i)
        {
            float value = Dot(pointsB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            normal = positive ? n : -n;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(quadNormal))
    {
        return false;
    }

    Vec3 segment = pointsB[1] - pointsB[0];
    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 edge = verticesA[(i + 1) % 4] - verticesA[i];
        if (!TestAxis(Cross(edge, segment)))
        {
            return false;
        }
    }

    for (int32 i = 0; i < 2; ++i)
    {
        Vec3 closest0 = ClosestPointVsTriangle(pointsB[i], verticesA[0], verticesA[1], verticesA[2]);
        Vec3 closest1 = ClosestPointVsTriangle(pointsB[i], verticesA[0], verticesA[2], verticesA[3]);
        Vec3 closest = Dist2(pointsB[i], closest0) <= Dist2(pointsB[i], closest1) ? closest0 : closest1;
        if (!TestAxis(pointsB[i] - closest))
        {
            return false;
        }
    }

    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 closest = ClosestPointVsSegment(verticesA[i], pointsB[0], pointsB[1]);
        if (!TestAxis(closest - verticesA[i]))
        {
            return false;
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool QuadVsBox(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const QuadShape* quad = (const QuadShape*)a;
    const BoxShape* box = (const BoxShape*)b;

    Vec3 quadNormal = tfA.q.Rotate(quad->GetNormal());
    Vec3 verticesA[4] = {
        Mul(tfA, quad->GetVertex(0)),
        Mul(tfA, quad->GetVertex(1)),
        Mul(tfA, quad->GetVertex(2)),
        Mul(tfA, quad->GetVertex(3)),
    };

    float planeOffset = Dot(verticesA[0], quadNormal);
    for (int32 i = 0; i < 4; ++i)
    {
        verticesA[i] -= quadNormal * planeOffset;
    }

    Vec3 verticesB[8];
    for (int32 i = 0; i < 8; ++i)
    {
        verticesB[i] = Mul(tfB, box->GetVertex(i)) - quadNormal * planeOffset;
    }

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = quadNormal;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 4; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(verticesB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 8; ++i)
        {
            float value = Dot(verticesB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            normal = positive ? n : -n;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(quadNormal))
    {
        return false;
    }

    Quat qB = tfB.q * box->GetRotation();
    Vec3 axesB[3] = { qB.Rotate(x_axis), qB.Rotate(y_axis), qB.Rotate(z_axis) };

    for (int32 i = 0; i < 3; ++i)
    {
        if (!TestAxis(axesB[i]))
        {
            return false;
        }
    }

    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 4] - verticesA[i];
        for (int32 j = 0; j < 3; ++j)
        {
            if (!TestAxis(Cross(edgeA, axesB[j])))
            {
                return false;
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool QuadVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const QuadShape* quad = (const QuadShape*)a;
    const ConvexShape* convex = (const ConvexShape*)b;

    Vec3 quadNormal = tfA.q.Rotate(quad->GetNormal());
    Vec3 verticesA[4] = {
        Mul(tfA, quad->GetVertex(0)),
        Mul(tfA, quad->GetVertex(1)),
        Mul(tfA, quad->GetVertex(2)),
        Mul(tfA, quad->GetVertex(3)),
    };

    float planeOffset = Dot(verticesA[0], quadNormal);
    for (int32 i = 0; i < 4; ++i)
    {
        verticesA[i] -= quadNormal * planeOffset;
    }

    int32 vertexCountB = convex->GetVertexCount();
    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = quadNormal;

    auto ProjectAxis = [&](Vec3 n, float* min, float* max) {
        Vec3 p = Mul(tfB, convex->GetVertex(0)) - quadNormal * planeOffset;
        *min = Dot(p, n);
        *max = *min;
        for (int32 i = 1; i < vertexCountB; ++i)
        {
            p = Mul(tfB, convex->GetVertex(i)) - quadNormal * planeOffset;
            float value = Dot(p, n);
            *min = Min(*min, value);
            *max = Max(*max, value);
        }
    };

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 4; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB, maxB;
        ProjectAxis(n, &minB, &maxB);

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            normal = positive ? n : -n;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(quadNormal))
    {
        return false;
    }

    for (Vec3 n : convex->GetFaceNormals())
    {
        if (!TestAxis(tfB.q.Rotate(n)))
        {
            return false;
        }
    }

    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 4] - verticesA[i];
        for (const ConvexFace& face : convex->GetFaces())
        {
            for (int32 j = 0; j < face.count; ++j)
            {
                int32 i0 = face.indices[j];
                int32 i1 = face.indices[(j + 1) % face.count];
                Vec3 p0 = Mul(tfB, convex->GetVertex(i0)) - quadNormal * planeOffset;
                Vec3 p1 = Mul(tfB, convex->GetVertex(i1)) - quadNormal * planeOffset;
                if (!TestAxis(Cross(edgeA, p1 - p0)))
                {
                    return false;
                }
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool QuadVsQuad(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const QuadShape* quadA = (const QuadShape*)a;
    const QuadShape* quadB = (const QuadShape*)b;

    Vec3 normalA = tfA.q.Rotate(quadA->GetNormal());
    Vec3 normalB = tfB.q.Rotate(quadB->GetNormal());
    Vec3 verticesA[4] = {
        Mul(tfA, quadA->GetVertex(0)),
        Mul(tfA, quadA->GetVertex(1)),
        Mul(tfA, quadA->GetVertex(2)),
        Mul(tfA, quadA->GetVertex(3)),
    };
    Vec3 verticesB[4] = {
        Mul(tfB, quadB->GetVertex(0)),
        Mul(tfB, quadB->GetVertex(1)),
        Mul(tfB, quadB->GetVertex(2)),
        Mul(tfB, quadB->GetVertex(3)),
    };

    float planeOffset = Dot(verticesA[0], normalA);
    for (int32 i = 0; i < 4; ++i)
    {
        verticesA[i] -= normalA * planeOffset;
        verticesB[i] -= normalA * planeOffset;
    }

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = normalA;

    auto TestAxis = [&](Vec3 n) -> bool {
        if (n.Normalize() == 0.0f)
        {
            return true;
        }

        float minA = Dot(verticesA[0], n);
        float maxA = minA;
        for (int32 i = 1; i < 4; ++i)
        {
            float value = Dot(verticesA[i], n);
            minA = Min(minA, value);
            maxA = Max(maxA, value);
        }

        float minB = Dot(verticesB[0], n);
        float maxB = minB;
        for (int32 i = 1; i < 4; ++i)
        {
            float value = Dot(verticesB[i], n);
            minB = Min(minB, value);
            maxB = Max(maxB, value);
        }

        float positiveSeparation = minB - maxA;
        float negativeSeparation = minA - maxB;
        float separation = Max(positiveSeparation, negativeSeparation);
        if (separation > radii)
        {
            return false;
        }

        float penetration = radii - separation;
        if (penetration < minPenetration)
        {
            bool positive = positiveSeparation > negativeSeparation;
            if (Abs(positiveSeparation - negativeSeparation) <= epsilon)
            {
                positive = minB + maxB > minA + maxA;
            }

            normal = positive ? n : -n;
            minPenetration = penetration;
        }

        return true;
    };

    if (!TestAxis(normalA))
    {
        return false;
    }

    if (!TestAxis(normalB))
    {
        return false;
    }

    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 edgeA = verticesA[(i + 1) % 4] - verticesA[i];
        for (int32 j = 0; j < 4; ++j)
        {
            Vec3 edgeB = verticesB[(j + 1) % 4] - verticesB[j];
            if (!TestAxis(Cross(edgeA, edgeB)))
            {
                return false;
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

struct HeightFieldContacts
{
    GrowableArray<ContactPoint, 8> contacts;
    Vec3 mean = Vec3::zero;
    Vec3 meanNormal = Vec3::zero;
};

static void AddHeightFieldContact(
    HeightFieldContacts* candidates, const Vec3& normal, const Vec3& anchorA, const Vec3& anchorB, int32 id
)
{
    float separation = Dot(anchorB - anchorA, normal);
    if (separation > 0.0f)
    {
        return;
    }

    ContactPoint candidate;
    candidate.anchorA = anchorA;
    candidate.anchorB = anchorB;
    candidate.p = (anchorA + anchorB) * 0.5f;
    candidate.normal = normal;
    candidate.id = id;

    candidates->contacts.push_back(candidate);
    candidates->mean += candidate.p;
    candidates->meanNormal += candidate.normal;
}

static void BuildHeightFieldManifold(const HeightFieldContacts& candidates, ContactManifold* manifold)
{
    int32 candidateCount = int32(candidates.contacts.size());
    if (candidateCount == 0)
    {
        manifold->contactCount = 0;
        return;
    }

    const ContactPoint* contacts = candidates.contacts.data();

    if (candidateCount <= max_contact_point_count)
    {
        for (int32 i = 0; i < candidateCount; ++i)
        {
            manifold->contactPoints[i] = contacts[i];
        }

        manifold->contactCount = candidateCount;
        return;
    }

    int32 indices[max_contact_point_count];
    int32 contactCount = 0;

    std::vector<Vec3> projected(candidateCount);
    std::vector<float> depth2(candidateCount);

    constexpr float minDepth2 = Sqr(linear_slop);
    Vec3 center = candidates.mean / candidateCount;
    Vec3 normal = Normalize(candidates.meanNormal / candidateCount);

    for (int32 i = 0; i < candidateCount; ++i)
    {
        Vec3 r = contacts[i].anchorA - center;
        projected[i] = GramSchmidt(r, normal);
        depth2[i] = Max(minDepth2, Length2(contacts[i].anchorB - contacts[i].anchorA));
    }

    int32 point1 = 0;
    float value = Max(minDepth2, Length2(projected[0])) * depth2[0];
    for (int32 i = 1; i < candidateCount; ++i)
    {
        float v = Max(minDepth2, Length2(projected[i])) * depth2[i];
        if (v > value)
        {
            point1 = i;
            value = v;
        }
    }

    int32 point2 = -1;
    value = -max_float;
    for (int32 i = 0; i < candidateCount; ++i)
    {
        if (i == point1)
        {
            continue;
        }

        float v = Max(minDepth2, Dist2(projected[i], projected[point1])) * depth2[i];
        if (v > value)
        {
            point2 = i;
            value = v;
        }
    }

    int32 point3 = -1;
    int32 point4 = -1;
    float minSide = 0.0f;
    float maxSide = 0.0f;
    Vec3 perp = Cross(projected[point2] - projected[point1], normal);

    for (int32 i = 0; i < candidateCount; ++i)
    {
        if (i == point1 || i == point2)
        {
            continue;
        }

        float side = Dot(perp, projected[i] - projected[point1]);
        if (side < minSide)
        {
            point3 = i;
            minSide = side;
        }
        else if (side > maxSide)
        {
            point4 = i;
            maxSide = side;
        }
    }

    indices[contactCount++] = point1;
    if (point3 != -1)
    {
        indices[contactCount++] = point3;
    }
    indices[contactCount++] = point2;
    if (point4 != -1)
    {
        indices[contactCount++] = point4;
    }

    for (int32 i = 0; i < contactCount; ++i)
    {
        manifold->contactPoints[i] = contacts[indices[i]];
    }

    manifold->contactCount = contactCount;
}

static bool HeightFieldVsShape(
    const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold
)
{
    const HeightFieldShape* heightField = (const HeightFieldShape*)a;

    AABB worldAABB;
    b->ComputeAABB(tfB, &worldAABB);

    Vec3 corners[8] = {
        MulT(tfA, Vec3{ worldAABB.min.x, worldAABB.min.y, worldAABB.min.z }),
        MulT(tfA, Vec3{ worldAABB.max.x, worldAABB.min.y, worldAABB.min.z }),
        MulT(tfA, Vec3{ worldAABB.min.x, worldAABB.max.y, worldAABB.min.z }),
        MulT(tfA, Vec3{ worldAABB.max.x, worldAABB.max.y, worldAABB.min.z }),
        MulT(tfA, Vec3{ worldAABB.min.x, worldAABB.min.y, worldAABB.max.z }),
        MulT(tfA, Vec3{ worldAABB.max.x, worldAABB.min.y, worldAABB.max.z }),
        MulT(tfA, Vec3{ worldAABB.min.x, worldAABB.max.y, worldAABB.max.z }),
        MulT(tfA, Vec3{ worldAABB.max.x, worldAABB.max.y, worldAABB.max.z }),
    };

    Vec3 min = corners[0];
    Vec3 max = min;
    for (int32 i = 1; i < 8; ++i)
    {
        min = Min(min, corners[i]);
        max = Max(max, corners[i]);
    }

    HeightFieldContacts candidates;
    AABB localAABB = AABB{ min, max };
    heightField->Query(localAABB, [&](int32 x, int32 z, int32 triangle, const Vec3& v0, const Vec3& v1, const Vec3& v2) {
        TriangleShape triangleShape{ v0, v1, v2 };

        ContactManifold manifold;
        bool touching = collide_function_map[Shape::triangle][b->GetType()](&triangleShape, tfA, b, tfB, &manifold);
        if (touching == false)
        {
            return;
        }

        int32 triangleId = ((z * heightField->GetCellCountX() + x) << 1) | triangle;
        for (int32 i = 0; i < manifold.contactCount; ++i)
        {
            ContactPoint contact = manifold.contactPoints[i];
            contact.id = (triangleId << 8) | (contact.id & 0xff);

            Vec3 normal = ResolveGhostNormal(
                heightField->GetActiveEdgeBits(x, z, triangle), triangleShape, tfA, contact.anchorA, contact.normal, Vec3::zero
            );
            AddHeightFieldContact(&candidates, normal, contact.anchorA, contact.anchorB, contact.id);
        }
    });

    if (candidates.contacts.size() == 0)
    {
        return false;
    }

    BuildHeightFieldManifold(candidates, manifold);
    return manifold->contactCount > 0;
}

void InitializeDetectionFunctionMap()
{
    if (detection_function_initialized)
    {
        return;
    }

    collide_function_map[Shape::sphere][Shape::sphere] = SphereVsSphere;

    collide_function_map[Shape::capsule][Shape::sphere] = CapsuleVsSphere;
    collide_function_map[Shape::capsule][Shape::capsule] = CapsuleVsCapsule;

    collide_function_map[Shape::box][Shape::sphere] = BoxVsSphere;
    collide_function_map[Shape::box][Shape::capsule] = BoxVsCapsule;
    collide_function_map[Shape::box][Shape::box] = BoxVsBox;

    collide_function_map[Shape::convex][Shape::sphere] = ConvexVsSphere;
    collide_function_map[Shape::convex][Shape::capsule] = ConvexVsConvex;
    collide_function_map[Shape::convex][Shape::box] = ConvexVsConvex;
    collide_function_map[Shape::convex][Shape::convex] = ConvexVsConvex;

    collide_function_map[Shape::triangle][Shape::sphere] = TriangleVsSphere;
    collide_function_map[Shape::triangle][Shape::capsule] = TriangleVsCapsule;
    collide_function_map[Shape::triangle][Shape::box] = TriangleVsBox;
    collide_function_map[Shape::triangle][Shape::convex] = TriangleVsConvex;
    collide_function_map[Shape::triangle][Shape::triangle] = TriangleVsTriangle;
    collide_function_map[Shape::triangle][Shape::quad] = TriangleVsQuad;

    collide_function_map[Shape::quad][Shape::sphere] = QuadVsSphere;
    collide_function_map[Shape::quad][Shape::capsule] = QuadVsCapsule;
    collide_function_map[Shape::quad][Shape::box] = QuadVsBox;
    collide_function_map[Shape::quad][Shape::convex] = QuadVsConvex;
    collide_function_map[Shape::quad][Shape::quad] = QuadVsQuad;

    collide_function_map[Shape::height_field][Shape::sphere] = HeightFieldVsShape;
    collide_function_map[Shape::height_field][Shape::capsule] = HeightFieldVsShape;
    collide_function_map[Shape::height_field][Shape::box] = HeightFieldVsShape;
    collide_function_map[Shape::height_field][Shape::convex] = HeightFieldVsShape;
    collide_function_map[Shape::height_field][Shape::triangle] = HeightFieldVsShape;
    collide_function_map[Shape::height_field][Shape::quad] = HeightFieldVsShape;
    collide_function_map[Shape::height_field][Shape::height_field] = nullptr;

    detection_function_initialized = true;
}

bool Collide(
    const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold, bool* featureFlipped
)
{
    MuliAssert(a != nullptr);
    MuliAssert(b != nullptr);

    if (!detection_function_initialized)
    {
        InitializeDetectionFunctionMap();
    }

    static ContactManifold defaultManifold;
    if (manifold == nullptr)
    {
        manifold = &defaultManifold;
    }
    *manifold = ContactManifold{};

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        if (featureFlipped)
        {
            *featureFlipped = true;
        }
        return collide_function_map[shapeB][shapeA](b, tfB, a, tfA, manifold);
    }
    else
    {
        if (featureFlipped)
        {
            *featureFlipped = false;
        }
        return collide_function_map[shapeA][shapeB](a, tfA, b, tfB, manifold);
    }
}

} // namespace muli3
