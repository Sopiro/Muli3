#include "muli3/collision.h"
#include "muli3/distance.h"
#include "muli3/frame.h"
#include "muli3/ghost.h"
#include "muli3/growable_stack.h"
#include "muli3/settings.h"
#include "muli3/shapes.h"

namespace muli3
{

bool detection_function_initialized = false;
CollideFunction* collide_function_map[Shape::shape_count][Shape::shape_count];
CollideFunction2* collide_function_map2[Shape::shape_count - Shape::height_field];

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

using EPAFaces = GrowableStack<EPAFace, epa_max_face_count>;
using EPAEdges = GrowableStack<EPAEdge, epa_max_edge_count>;

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

static constexpr int32 default_clipped_vertex_count = 64;

using ClippedFace = GrowableStack<Vec3, default_clipped_vertex_count>;

static Vec3 IntersectPlaneEdge(const Vec3& a, const Vec3& b, float da, float db)
{
    // Solve da + t * (db - da) = 0 using the signed distances at the edge endpoints.
    const float denom = da - db;
    if (Abs(denom) < epsilon)
    {
        return a;
    }

    float t = Clamp(da / denom, 0, 1);
    return a + t * (b - a);
}

static void ClipFace(ClippedFace* out, const ClippedFace& in, const Vec3& p, const Vec3& n)
{
    // Clip the polygon against the half-space whose boundary passes through p and whose normal points along n.
    MuliAssert(in.size() != 0);
    out->clear();

    // Start with the closing edge from the last vertex to the first vertex.
    Vec3 p0 = in[in.size() - 1];
    float d0 = Dot(p0 - p, n);
    bool inside0 = d0 >= -epsilon;

    for (int32 i = 0; i < in.size(); ++i)
    {
        Vec3 p1 = in[i];
        float d1 = Dot(p1 - p, n);
        bool inside1 = d1 >= -epsilon;

        if (inside0 && inside1)
        {
            // Keep p1 when both endpoints are inside the clipping half-space.
            out->push_back(p1);
        }
        else if (inside0 && !inside1)
        {
            // Keep only the intersection when the edge leaves the clipping half-space.
            Vec3 intersection = IntersectPlaneEdge(p0, p1, d0, d1);

            out->emplace_back(intersection);
        }
        else if (!inside0 && inside1)
        {
            // Keep the intersection and p1 when the edge enters the clipping half-space.
            Vec3 intersection = IntersectPlaneEdge(p0, p1, d0, d1);

            out->emplace_back(intersection);
            out->push_back(p1);
        }

        // Nothing to do with outside -> outside case
        // Advance to the next edge.
        p0 = p1;
        d0 = d1;
        inside0 = inside1;
    }
}

static void AssignContactId(ContactManifold* manifold, const Vec3& origin, const Vec3& tangent1)
{
    if (manifold->contactCount == 1)
    {
        manifold->contactPoints[0].id = 0;
        return;
    }

    Vec3 tangent2 = Cross(manifold->normal, tangent1);

    int32 indices[max_contact_point_count];
    float angles[max_contact_point_count];

    for (int32 i = 0; i < manifold->contactCount; ++i)
    {
        Vec3 point = manifold->contactPoints[i].anchorA;
        Vec3 delta = point - origin;

        float angle = std::atan2(Dot(delta, tangent2), Dot(delta, tangent1));

        indices[i] = i;
        angles[i] = angle < 0.0f ? angle + two_pi : angle;
    }

    // Sort from the positive tangent axis in counter-clockwise order around the contact normal.
    for (int32 i = 1; i < manifold->contactCount; ++i)
    {
        int32 index = indices[i];
        int32 j = i - 1;

        while (j >= 0 && angles[indices[j]] > angles[index])
        {
            indices[j + 1] = indices[j];
            --j;
        }

        indices[j + 1] = index;
    }

    for (int32 i = 0; i < manifold->contactCount; ++i)
    {
        manifold->contactPoints[indices[i]].id = i;
    }
}

static void FindContactPoints(
    const Vec3& n, const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold
)
{
    manifold->normal = n;

    // Find the faces on A and B that oppose each other along the collision normal.
    Face faceA = a->GetFeaturedFace(tfA, n);
    Face faceB = b->GetFeaturedFace(tfB, -n);

    // Build the face polygons in world space for clipping.
    ClippedFace clippedA, clippedB;
    clippedA.resize(faceA.vertexCount);
    clippedB.resize(faceB.vertexCount);

    float ra = a->GetRadius();
    float rb = b->GetRadius();

    Vec3 origin(0);

    // Offset shape A vertices along the face normal by its collision radius.
    for (int32 i = 0; i < faceA.vertexCount; ++i)
    {
        int32 vertexIndex = a->GetVertexIndex(faceA.vertexStart + i);
        Vec3 point = Mul(tfA, a->GetVertex(vertexIndex));
        origin += point;
        clippedA[i] = point + faceA.normal * ra;
    }
    origin /= faceA.vertexCount;

    Vec3 mid = (clippedA[faceA.vertexCount - 1] + clippedA[0]) * 0.5f;
    Vec3 tangent1 = GramSchmidt(mid - origin, n);

    // Offset shape B vertices along the face normal by its collision radius.
    for (int32 i = 0; i < faceB.vertexCount; ++i)
    {
        int32 vertexIndex = b->GetVertexIndex(faceB.vertexStart + i);
        clippedB[i] = Mul(tfB, b->GetVertex(vertexIndex)) + faceB.normal * rb;
    }

    ClippedFace* ref; // Reference face
    ClippedFace* inc; // Incident face

    bool flipped;

    float aParallelness = AbsDot(faceA.normal, n);
    float bParallelness = AbsDot(faceB.normal, n);

    // Keep A as reference when B has fewer vertices and cannot bound A's face.
    if (std::min(faceA.vertexCount, faceB.vertexCount) < 3 && (faceB.vertexCount < faceA.vertexCount))
    {
        ref = &clippedA;
        inc = &clippedB;
        flipped = false;
    }
    else if (bParallelness > aParallelness)
    {
        // Otherwise use the face whose normal is more closely aligned with the collision normal.
        ref = &clippedB;
        inc = &clippedA;
        flipped = true;
    }
    else
    {
        // Keep shape A as reference on ties for deterministic selection.
        ref = &clippedA;
        inc = &clippedB;
        flipped = false;
    }

    // Define the reference plane using its outward normal and any point on the face.
    Vec3 planeNormal = flipped ? faceB.normal : faceA.normal;
    Vec3 planePoint = ref->at(0);

    // Ping-pong buffers and indices.
    ClippedFace out;
    ClippedFace* faces[2] = { inc, &out };

    int32 input = 0;
    int32 output = 1;

    for (int32 i0 = ref->size() - 1, i1 = 0; i1 < ref->size(); i0 = i1, ++i1)
    {
        // Build the inward-facing side plane of this reference edge.
        Vec3 edge = ref->at(i1) - ref->at(i0);
        Vec3 inward = Normalize(Cross(planeNormal, edge));

        // Clip the current polygon against this side plane.
        ClipFace(faces[output], *faces[input], ref->at(i0), inward);

        if (faces[output]->size() == 0)
        {
            manifold->contactCount = 0;
            return;
        }

        // Pass this result to the next reference side plane.
        std::swap(input, output);
    }

    // Keep only points behind the reference plane.
    faces[output]->clear();
    for (int32 i = 0; i < faces[input]->size(); ++i)
    {
        // Negative separation lies behind the outward-facing reference plane.
        float separation = Dot(faces[input]->at(i) - planePoint, planeNormal);
        if (separation < 0.0f)
        {
            faces[output]->push_back(faces[input]->at(i));
        }
    }

    if (faces[output]->size() == 0)
    {
        manifold->contactCount = 0;
        return;
    }

    GrowableStack<ContactPoint, default_clipped_vertex_count> candidates;
    candidates.resize(faces[output]->size());
    for (int32 i = 0; i < faces[output]->size(); ++i)
    {
        Vec3 point = faces[output]->at(i);
        float separation = Dot(point - planePoint, planeNormal);

        // The clipped point lies on the incident face, so project the other anchor onto the reference plane.
        if (flipped)
        {
            candidates[i].anchorA = point;
            candidates[i].anchorB = point - planeNormal * separation;
        }
        else
        {
            candidates[i].anchorA = point - planeNormal * separation;
            candidates[i].anchorB = point;
        }
    }

    // Keep the complete clipped polygon when it already fits the manifold capacity.
    if (faces[output]->size() <= max_contact_point_count)
    {
        for (int32 i = 0; i < faces[output]->size(); ++i)
        {
            manifold->contactPoints[i] = candidates[i];
        }

        manifold->contactCount = faces[output]->size();
        AssignContactId(manifold, origin, tangent1);
        return;
    }

    // Reduce contact points

    int32 indices[max_contact_point_count];
    int32 contactCount = 0;

    constexpr float minDepth2 = Sqr(linear_slop);

    Vec3 centerA = Mul(tfA, a->GetCenter());

    GrowableStack<Vec3, default_clipped_vertex_count> projected;
    GrowableStack<float, default_clipped_vertex_count> depth2;
    projected.resize(faces[output]->size());
    depth2.resize(faces[output]->size());

    // Work in the contact tangent plane around shape A.
    for (int32 i = 0; i < faces[output]->size(); ++i)
    {
        Vec3 r = candidates[i].anchorA - centerA;
        projected[i] = GramSchmidt(r, n);
        depth2[i] = Max(minDepth2, Length2(candidates[i].anchorB - candidates[i].anchorA));
    }

    // Start with the point that is farthest from the center and deepest
    int32 point1 = 0;
    float value = Max(minDepth2, Length2(projected[0])) * depth2[0];
    for (int32 i = 1; i < faces[output]->size(); ++i)
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
    for (int32 i = 0; i < faces[output]->size(); ++i)
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
    for (int32 i = 0; i < faces[output]->size(); ++i)
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
    }

    manifold->contactCount = contactCount;
    AssignContactId(manifold, origin, tangent1);
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
    manifold->normal = normal;
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
    manifold->normal = normal;
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

    Vec3 axisA = a1 - a0;
    float lengthA = axisA.Normalize();

    Vec3 axisB = b1 - b0;
    float lengthB = axisB.Normalize();

    if (distance <= epsilon)
    {
        // The closest segment points coincide. Pick a stable normal from crossed axes,
        // then fall back to the center delta projected off the capsule axis.
        normal = Cross(axisA, axisB);

        Vec3 deltaCenter = ((b0 + b1) - (a0 + a1)) * 0.5f;

        if (normal.Normalize() == 0.0f)
        {
            Vec3 axis = lengthA > epsilon ? axisA : (lengthB > epsilon ? axisB : y_axis);
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

    // Nearly parallel capsules can support a line contact.
    // Clip capsule B's axis to capsule A's axis range and emit the two surviving endpoints.
    Vec3 d = Cross(axisA, axisB);

    constexpr float sineThreshold = 0.05233f; // ~ sin 3
    if (Length2(d) < Sqr(sineThreshold))
    {
        float s0 = Dot(b0 - a0, axisA);
        float s1 = Dot(b1 - a0, axisA);
        float ds = s1 - s0;

        float t0 = 0.0f;
        float t1 = 1.0f;
        if (Abs(ds) <= epsilon)
        {
            if (s0 < 0.0f || s0 > lengthA)
            {
                t1 = -1.0f;
            }
        }
        else if (ds > 0.0f)
        {
            t0 = Max(t0, -s0 / ds);
            t1 = Min(t1, (lengthA - s0) / ds);
        }
        else
        {
            t0 = Max(t0, (lengthA - s0) / ds);
            t1 = Min(t1, -s0 / ds);
        }

        if ((t1 - t0) * lengthB > linear_slop)
        {
            Vec3 pointsB[2] = { b0 + (b1 - b0) * t0, b0 + (b1 - b0) * t1 };
            Vec3 pointsA[2] = { ClosestPointVsSegment(pointsB[0], a0, a1), ClosestPointVsSegment(pointsB[1], a0, a1) };

            Vec3 normals[2];
            float minDistance = 0.01f * linear_slop;
            bool valid = true;
            for (int32 i = 0; i < 2; ++i)
            {
                normals[i] = pointsB[i] - pointsA[i];
                float d = normals[i].Normalize();
                if (d < minDistance)
                {
                    valid = false;
                    break;
                }
                else if (d > radii)
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                Vec3 manifoldNormal = normals[0] + normals[1];
                if (manifoldNormal.Normalize() == 0.0f)
                {
                    manifoldNormal = normal;
                }

                for (int32 i = 0; i < 2; ++i)
                {
                    manifold->contactPoints[i].anchorA = pointsA[i] + manifoldNormal * ra;
                    manifold->contactPoints[i].anchorB = pointsB[i] - manifoldNormal * rb;
                    manifold->contactPoints[i].id = i;
                }

                manifold->normal = manifoldNormal;
                manifold->contactCount = 2;
                return true;
            }
        }
    }

    manifold->contactPoints[0].anchorA = pa + normal * ra;
    manifold->contactPoints[0].anchorB = pb - normal * rb;
    manifold->normal = normal;
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
    manifold->normal = normal;
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
        float d = Dot(dir, n);

        float separation = pa + pb + radii - Abs(d);

        // Found separation axis
        if (separation < 0.0f)
        {
            return false;
        }

        // Keep the axis with the minimum overlap
        if (separation < minPenetration)
        {
            minPenetration = separation;
            normal = d > 0.0f ? n : -n;
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
    Vec3 localCenterB = MulT(transformA, centerB);

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    std::span<const Face> faces = convex->GetFaces();
    std::span<const int32> indices = convex->GetIndices();

    int32 faceIndex = 0;
    float maxSeparation = -max_float;
    float minDistance2 = max_float;
    Vec3 localClosest;

    // The closest core feature belongs to at least one face whose plane is violated by the sphere center.
    for (int32 i = 0; i < int32(faces.size()); ++i)
    {
        const Face& face = faces[i];
        const Vec3& va = convex->GetVertex(indices[face.vertexStart]);
        float separation = Dot(face.normal, localCenterB - va);
        if (separation > radii)
        {
            return false;
        }

        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            faceIndex = i;
        }

        if (separation <= 0.0f)
        {
            continue;
        }

        // Triangulate faces and test aganist sub triangles
        for (int32 j = 1; j + 1 < face.vertexCount; ++j)
        {
            const Vec3& vb = convex->GetVertex(indices[face.vertexStart + j]);
            const Vec3& vc = convex->GetVertex(indices[face.vertexStart + j + 1]);
            Vec3 closest = ClosestPointVsTriangle(localCenterB, va, vb, vc);
            float distance2 = Dist2(localCenterB, closest);

            if (distance2 < minDistance2)
            {
                minDistance2 = distance2;
                localClosest = closest;
            }
        }
    }

    Vec3 normal;
    Vec3 closest;
    if (maxSeparation > 0.0f)
    {
        if (minDistance2 > Sqr(radii))
        {
            return false;
        }

        Vec3 localNormal = localCenterB - localClosest;
        float distance = localNormal.Normalize();
        if (distance > epsilon)
        {
            normal = transformA.q.Rotate(localNormal);
            closest = Mul(transformA, localClosest) + normal * ra;
        }
        else
        {
            normal = transformA.q.Rotate(faces[faceIndex].normal);
            closest = centerB + normal * (ra - maxSeparation);
        }
    }
    else
    {
        // The sphere center is in the core hull. Use the nearest face as the exit direction.
        normal = transformA.q.Rotate(faces[faceIndex].normal);
        closest = centerB + normal * (ra - maxSeparation);
    }

    manifold->contactPoints[0].anchorA = closest;
    manifold->contactPoints[0].anchorB = centerB - normal * rb;
    manifold->normal = normal;
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
            manifold->normal = normal;
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

static bool TestAxis(
    float* minPenetration, Vec3* normal, const Vec3& axis, float minA, float maxA, float minB, float maxB, float radii
)
{
    float separation1 = minB - maxA;
    float separation2 = minA - maxB;

    float separation = Max(separation1, separation2);
    if (separation > radii)
    {
        return false;
    }

    float penetration = radii - separation;
    if (penetration < *minPenetration)
    {
        *normal = separation1 > separation2 ? axis : -axis;
        *minPenetration = penetration;
    }

    return true;
}

template <int32 sizeA, int32 sizeB>
static bool TestAxis(
    float* minPenetration,
    Vec3* normal,
    const Vec3& axis,
    const Vec3 (&verticesA)[sizeA],
    const Vec3 (&verticesB)[sizeB],
    float radii
)
{
    float minA = Dot(axis, verticesA[0]);
    float maxA = minA;
    for (int32 i = 1; i < sizeA; ++i)
    {
        float projection = Dot(axis, verticesA[i]);
        minA = Min(minA, projection);
        maxA = Max(maxA, projection);
    }

    float minB = Dot(axis, verticesB[0]);
    float maxB = minB;
    for (int32 i = 1; i < sizeB; ++i)
    {
        float projection = Dot(axis, verticesB[i]);
        minB = Min(minB, projection);
        maxB = Max(maxB, projection);
    }

    return TestAxis(minPenetration, normal, axis, minA, maxA, minB, maxB, radii);
}

bool TriangleVsSphere(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangle = (const TriangleShape*)a;

    Vec3 p = Mul(tfB, b->GetCenter());
    Vec3 localP = MulT(tfA, p);

    const Vec3* vertices = triangle->GetVertices();

    Vec3 normalA = triangle->GetNormal();
    Vec3 closest = ClosestPointVsTriangle(localP, vertices[0], vertices[1], vertices[2]);

    Vec3 normal = localP - closest;
    float distance = normal.Normalize();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    if (distance > ra + rb)
    {
        return false;
    }

    float separation = Dot(localP - vertices[0], normalA);
    if (distance <= epsilon)
    {
        normal = separation < 0.0f ? -normalA : normalA;
    }

    normal = tfA.q.Rotate(normal);
    manifold->contactPoints[0].anchorA = Mul(tfA, closest) + normal * ra;
    manifold->contactPoints[0].anchorB = p - normal * rb;
    manifold->normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool TriangleVsCapsule(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const TriangleShape* triangle = (const TriangleShape*)a;
    const CapsuleShape* capsule = (const CapsuleShape*)b;

    Vec3 normalA = tfA.q.Rotate(triangle->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangle->GetVertex(0)),
        Mul(tfA, triangle->GetVertex(1)),
        Mul(tfA, triangle->GetVertex(2)),
    };

    // Shift along the triangle normal to keep plane projections near zero.
    float planeOffset = Dot(verticesA[0], normalA);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= normalA * planeOffset;
    }

    Vec3 verticesB[2] = {
        Mul(tfB, capsule->GetVertexA()) - normalA * planeOffset,
        Mul(tfB, capsule->GetVertexB()) - normalA * planeOffset,
    };

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = normalA;

    float projectionA = Dot(verticesA[0], normalA);
    float projectionB0 = Dot(verticesB[0], normalA);
    float projectionB1 = Dot(verticesB[1], normalA);
    if (!TestAxis(
            &minPenetration, &normal, normalA, projectionA, projectionA, Min(projectionB0, projectionB1),
            Max(projectionB0, projectionB1), radii
        ))
    {
        return false;
    }

    Vec3 segment = verticesB[1] - verticesB[0];

    // Triangle edge vs capsule axis.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 edge = verticesA[(i + 1) % 3] - verticesA[i];
        Vec3 n = Cross(edge, segment);
        if (n.Normalize() == 0.0f)
        {
            continue;
        }

        float edgeProjectionA = Dot(verticesA[i], n);
        float oppositeProjectionA = Dot(verticesA[(i + 2) % 3], n);
        float projectionB = Dot(verticesB[0], n);
        if (!TestAxis(
                &minPenetration, &normal, n, Min(edgeProjectionA, oppositeProjectionA), Max(edgeProjectionA, oppositeProjectionA),
                projectionB, projectionB, radii
            ))
        {
            return false;
        }
    }

    // Capsule caps against triangle interior.
    for (int32 i = 0; i < 2; ++i)
    {
        Vec3 closest = ClosestPointVsTriangle(verticesB[i], verticesA[0], verticesA[1], verticesA[2]);
        Vec3 axis = verticesB[i] - closest;
        if (axis.Normalize() == 0.0f)
        {
            continue;
        }

        if (!TestAxis(&minPenetration, &normal, axis, verticesA, verticesB, radii))
        {
            return false;
        }
    }

    // Triangle vertices against the capsule segment.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 closest = ClosestPointVsSegment(verticesA[i], verticesB[0], verticesB[1]);
        Vec3 axis = closest - verticesA[i];
        if (axis.Normalize() == 0.0f)
        {
            continue;
        }

        if (!TestAxis(&minPenetration, &normal, axis, verticesA, verticesB, radii))
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

    Vec3 normalA = tfA.q.Rotate(triangle->GetNormal());
    Vec3 verticesA[3] = {
        Mul(tfA, triangle->GetVertex(0)),
        Mul(tfA, triangle->GetVertex(1)),
        Mul(tfA, triangle->GetVertex(2)),
    };

    // Shift along the triangle normal to keep plane projections near zero.
    float planeOffset = Dot(verticesA[0], normalA);
    for (int32 i = 0; i < 3; ++i)
    {
        verticesA[i] -= normalA * planeOffset;
    }

    Vec3 centerB = Mul(tfB, box->GetCenter()) - normalA * planeOffset;

    Quat qB = tfB.q * box->GetRotation();
    Vec3 axesB[3] = { qB.Rotate(x_axis), qB.Rotate(y_axis), qB.Rotate(z_axis) };
    Vec3 extentsB = box->GetHalfExtents();

    float radii = a->GetRadius() + b->GetRadius();
    float minPenetration = max_float;
    Vec3 normal = normalA;

    float projectionA = Dot(verticesA[0], normalA);
    float projectionB =
        extentsB.x * AbsDot(axesB[0], normalA) + extentsB.y * AbsDot(axesB[1], normalA) + extentsB.z * AbsDot(axesB[2], normalA);

    float centerProjectionB = Dot(centerB, normalA);
    if (!TestAxis(
            &minPenetration, &normal, normalA, projectionA, projectionA, centerProjectionB - projectionB,
            centerProjectionB + projectionB, radii
        ))
    {
        return false;
    }

    // Box face normals.
    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 axis = axesB[i];

        float projectionA0 = Dot(verticesA[0], axis);
        float projectionA1 = Dot(verticesA[1], axis);
        float projectionA2 = Dot(verticesA[2], axis);

        float minA = Min(projectionA0, projectionA1, projectionA2);
        float maxA = Max(projectionA0, projectionA1, projectionA2);

        float centerProjectionB = Dot(centerB, axis);
        if (!TestAxis(
                &minPenetration, &normal, axis, minA, maxA, centerProjectionB - extentsB[i], centerProjectionB + extentsB[i],
                radii
            ))
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
            Vec3 n = Cross(edgeA, axesB[j]);
            if (n.Normalize() == 0.0f)
            {
                continue;
            }

            float edgeProjectionA = Dot(verticesA[i], n);
            float oppositeProjectionA = Dot(verticesA[(i + 2) % 3], n);

            float minA = Min(edgeProjectionA, oppositeProjectionA);
            float maxA = Max(edgeProjectionA, oppositeProjectionA);

            float projectionB =
                extentsB.x * AbsDot(axesB[0], n) + extentsB.y * AbsDot(axesB[1], n) + extentsB.z * AbsDot(axesB[2], n);
            float centerProjectionB = Dot(centerB, n);

            if (!TestAxis(
                    &minPenetration, &normal, n, minA, maxA, centerProjectionB - projectionB, centerProjectionB + projectionB,
                    radii
                ))
            {
                return false;
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

    // Test on triangle B face normal
    float projectionA = Dot(verticesA[0], normalA);
    float projectionB0 = Dot(verticesB[0], normalA);
    float projectionB1 = Dot(verticesB[1], normalA);
    float projectionB2 = Dot(verticesB[2], normalA);
    float minB = Min(projectionB0, projectionB1, projectionB2);
    float maxB = Max(projectionB0, projectionB1, projectionB2);
    if (!TestAxis(&minPenetration, &normal, normalA, projectionA, projectionA, minB, maxB, radii))
    {
        return false;
    }

    // Test on triangle B face normal
    float projectionB = Dot(verticesB[0], normalB);
    float projectionA0 = Dot(verticesA[0], normalB);
    float projectionA1 = Dot(verticesA[1], normalB);
    float projectionA2 = Dot(verticesA[2], normalB);
    float minA = Min(projectionA0, projectionA1, projectionA2);
    float maxA = Max(projectionA0, projectionA1, projectionA2);
    if (!TestAxis(&minPenetration, &normal, normalB, minA, maxA, projectionB, projectionB, radii))
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

            Vec3 axis = Cross(edgeA, edgeB);
            if (axis.Normalize() == 0.0f)
            {
                continue;
            }

            float edgeProjectionA = Dot(verticesA[i], axis);
            float oppositeProjectionA = Dot(verticesA[(i + 2) % 3], axis);
            float edgeProjectionB = Dot(verticesB[j], axis);
            float oppositeProjectionB = Dot(verticesB[(j + 2) % 3], axis);
            if (!TestAxis(
                    &minPenetration, &normal, axis, Min(edgeProjectionA, oppositeProjectionA),
                    Max(edgeProjectionA, oppositeProjectionA), Min(edgeProjectionB, oppositeProjectionB),
                    Max(edgeProjectionB, oppositeProjectionB), radii
                ))
            {
                return false;
            }
        }
    }

    FindContactPoints(normal, a, tfA, b, tfB, manifold);
    return manifold->contactCount > 0;
}

bool PolygonVsSphere(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    const PolygonShape* polygon = (const PolygonShape*)a;

    Vec3 p = Mul(tfB, b->GetCenter());
    Vec3 localP = MulT(tfA, p);

    std::span<const Vec3> vertices = polygon->GetVertices();

    Vec3 polygonNormal = polygon->GetNormal();
    Vec3 closest = ClosestPointVsPolygon(localP, vertices);

    Vec3 normal = localP - closest;
    float distance = normal.Normalize();

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    if (distance > ra + rb)
    {
        return false;
    }

    float separation = Dot(localP - vertices[0], polygonNormal);
    if (distance <= epsilon)
    {
        normal = separation < 0.0f ? -polygonNormal : polygonNormal;
    }

    normal = tfA.q.Rotate(normal);
    manifold->contactPoints[0].anchorA = Mul(tfA, closest) + normal * ra;
    manifold->contactPoints[0].anchorB = p - normal * rb;
    manifold->normal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactCount = 1;

    return true;
}

bool HeightFieldVsShape(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ManifoldSet* manifolds)
{
    const HeightFieldShape* heightField = (const HeightFieldShape*)a;

    AABB worldAABB;
    b->ComputeAABB(tfB, &worldAABB);
    AABB localAABB = MulT(tfA, worldAABB);

    heightField->Query(localAABB, [&](int32 x, int32 z, int32 triangle, const Vec3& v0, const Vec3& v1, const Vec3& v2) {
        TriangleShape triangleShape{ v0, v1, v2 };

        ContactManifold manifold{};
        bool touching = collide_function_map[Shape::triangle][b->GetType()](&triangleShape, tfA, b, tfB, &manifold);
        if (touching == false)
        {
            return;
        }

        Vec3 point = Vec3::zero;
        int32 triangleId = ((z * heightField->GetCellCountX() + x) << 1) | triangle;
        manifold.id = triangleId + 1;
        for (int32 i = 0; i < manifold.contactCount; ++i)
        {
            point += manifold.contactPoints[i].anchorA;
        }
        point /= manifold.contactCount;

        manifold.normal = ResolveGhostNormal(
            heightField->GetActiveEdgeBits(x, z, triangle), triangleShape, tfA, point, manifold.normal, Vec3::zero
        );

        manifolds->push_back(manifold);
    });

    return manifolds->size() > 0;
}

bool MeshVsShape(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ManifoldSet* manifolds)
{
    const MeshShape* mesh = (const MeshShape*)a;

    AABB worldAABB;
    b->ComputeAABB(tfB, &worldAABB);
    AABB localAABB = MulT(tfA, worldAABB);

    mesh->Query(localAABB, [&](int32 triangle, const Vec3& v0, const Vec3& v1, const Vec3& v2) {
        TriangleShape triangleShape{ v0, v1, v2 };
        ContactManifold manifold{};
        bool touching = collide_function_map[Shape::triangle][b->GetType()](&triangleShape, tfA, b, tfB, &manifold);
        if (touching == false)
        {
            return;
        }

        Vec3 point = Vec3::zero;
        manifold.id = triangle + 1;
        for (int32 i = 0; i < manifold.contactCount; ++i)
        {
            point += manifold.contactPoints[i].anchorA;
        }
        point /= manifold.contactCount;

        manifold.normal =
            ResolveGhostNormal(mesh->GetActiveEdgeBits(triangle), triangleShape, tfA, point, manifold.normal, Vec3::zero);

        manifolds->push_back(manifold);
    });

    return manifolds->size() > 0;
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
    collide_function_map[Shape::triangle][Shape::convex] = ConvexVsConvex;
    collide_function_map[Shape::triangle][Shape::triangle] = TriangleVsTriangle;
    collide_function_map[Shape::triangle][Shape::polygon] = ConvexVsConvex;

    collide_function_map[Shape::polygon][Shape::sphere] = PolygonVsSphere;
    collide_function_map[Shape::polygon][Shape::capsule] = ConvexVsConvex;
    collide_function_map[Shape::polygon][Shape::box] = ConvexVsConvex;
    collide_function_map[Shape::polygon][Shape::convex] = ConvexVsConvex;
    collide_function_map[Shape::polygon][Shape::polygon] = ConvexVsConvex;

    collide_function_map2[Shape::height_field - Shape::height_field] = HeightFieldVsShape;
    collide_function_map2[Shape::mesh - Shape::height_field] = MeshVsShape;

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

    ContactManifold defaultManifold;
    if (manifold == nullptr)
    {
        manifold = &defaultManifold;
    }
    *manifold = ContactManifold{};

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeA > Shape::triangle || shapeB > Shape::triangle)
    {
        MuliAssert(false);
        return false;
    }

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
