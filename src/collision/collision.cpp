#include "muli3/collision.h"
#include "muli3/box_shape.h"
#include "muli3/capsule_shape.h"
#include "muli3/convex_shape.h"
#include "muli3/frame.h"
#include "muli3/growable_array.h"
#include "muli3/settings.h"

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

    faces.EmplaceBack(a, b, c, normal, distance, false);
    return true;
}

static void AddEPAEdge(EPAEdges& edges, int32 a, int32 b)
{
    for (int32 i = 0; i < edges.Count(); ++i)
    {
        if (edges[i].a == b && edges[i].b == a)
        {
            std::swap(edges[i], edges.Back());
            edges.PopBack();
            return;
        }
    }

    edges.EmplaceBack(a, b);
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

    if (faces.Count() != max_simplex_vertex_count)
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
        for (int32 i = 0; i < faces.Count(); ++i)
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

        for (int32 i = 0; i < faces.Count(); ++i)
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

        for (int32 i = 0; i < edges.Count(); ++i)
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

static int32 ClipFace(const Face& f, const Vec3& p, const Vec3& dir, Vec3* result, int32* clipBegin, int32* clipEnd)
{
    const int32 count = f.count;

    float d[max_face_vertices];
    bool outside[max_face_vertices];
    int32 active[max_face_vertices];

    int32 activeCount = 0;
    int32 outsideCount = 0;

    for (int32 i = 0; i < count; ++i)
    {
        if (f.points[i].id < 0)
        {
            continue;
        }

        d[i] = Dot(f.points[i].p - p, dir);

        // Dot(x - p, dir) >= 0 : inside
        // Dot(x - p, dir) <  0 : outside / clipped
        outside[i] = d[i] < -epsilon;

        active[activeCount++] = i;

        if (outside[i])
        {
            ++outsideCount;
        }
    }

    MuliAssert(activeCount >= 2);

    // No clips.
    if (outsideCount == 0)
    {
        *clipBegin = -1;
        *clipEnd = -1;
        return 0;
    }

    // The entire active face is outside the clipping plane.
    // No intersection points can be generated in this case.
    if (outsideCount == activeCount)
    {
        *clipBegin = active[0];
        *clipEnd = active[activeCount - 1];
        return outsideCount;
    }

    int32 begin = -1;
    int32 end = -1;

    int32 beforeBegin = -1;
    int32 afterEnd = -1;

    for (int32 k0 = activeCount - 1, k1 = 0; k1 < activeCount; k0 = k1, ++k1)
    {
        const int32 i0 = active[k0];
        const int32 i1 = active[k1];

        const bool outside0 = outside[i0];
        const bool outside1 = outside[i1];

        // Active edge: i0 -> i1

        // inside -> outside
        // First vertex of the clipped range.
        if (!outside0 && outside1)
        {
            beforeBegin = i0;
            begin = i1;
        }

        // outside -> inside
        // Last vertex of the clipped range.
        if (outside0 && !outside1)
        {
            end = i0;
            afterEnd = i1;
        }
    }

    MuliAssert(begin >= 0 && end >= 0);
    MuliAssert(beforeBegin >= 0 && afterEnd >= 0);

    result[0] = IntersectPlaneEdge(f.points[beforeBegin].p, f.points[begin].p, d[beforeBegin], d[begin]);
    result[1] = IntersectPlaneEdge(f.points[end].p, f.points[afterEnd].p, d[end], d[afterEnd]);

    *clipBegin = begin;
    *clipEnd = end;

    return outsideCount;
}

static void FindContactPoints(
    const Vec3& n, const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold
)
{
    Face faceA = a->GetFeaturedFace(tfA, n);
    Face faceB = b->GetFeaturedFace(tfB, -n);

    TranslateFace(&faceA, n * a->GetRadius());
    TranslateFace(&faceB, -n * b->GetRadius());

    Face ref; // Reference face
    Face inc; // Incident face

    float aParallelness = AbsDot(faceA.normal, n);
    float bParallelness = AbsDot(faceB.normal, n);

    if (bParallelness > aParallelness)
    {
        ref = faceB;
        inc = faceA;
        manifold->featureFlipped = true;
        manifold->contactNormal = -n;
    }
    else
    {
        ref = faceA;
        inc = faceB;
        manifold->featureFlipped = false;
        manifold->contactNormal = n;
    }

    manifold->referencePoint = ref.points[0];

    Vec3 planeNormal = ref.normal;
    Vec3 planePoint = ref.points[0].p;

    for (int32 i0 = ref.count - 1, i1 = 0; i1 < ref.count; i0 = i1, ++i1)
    {
        Vec3 edge = ref.points[i1].p - ref.points[i0].p;
        Vec3 inward = Normalize(Cross(planeNormal, edge));

        Vec3 clipped[2];
        int32 clipBegin, clipEnd;
        if (!ClipFace(inc, ref.points[i0].p, inward, clipped, &clipBegin, &clipEnd))
        {
            continue;
        }

        if (clipBegin == clipEnd)
        {
            inc.points[clipBegin].p = clipped[0];
        }
        else
        {
            inc.points[clipBegin].p = clipped[0];
            inc.points[clipEnd].p = clipped[1];

            // Invalidate vertices between clipBegin and clipEnd
            for (int32 i = (clipBegin + 1 == inc.count) ? 0 : clipBegin + 1; i != clipEnd; i = (i + 1 == inc.count) ? 0 : i + 1)
            {
                inc.points[i].id = -1;
            }
        }
    }

    // Invalidate vertices that lie above the reference plane
    for (int32 i = 0; i < inc.count; ++i)
    {
        float penetration = Dot(inc.points[i].p - planePoint, planeNormal);
        if (penetration > 0)
        {
            inc.points[i].id = -1;
        }
    }

    Face* major = faceA.count > faceB.count ? &faceA : &faceB;

    int32 contactCount = 0;
    for (int32 i = 0; i < inc.count; ++i)
    {
        if (inc.points[i].id == -1)
        {
            continue;
        }

        // To ensure consistent warm starting, the contact point id is always set based on the face with more vertices
        manifold->contactPoints[contactCount].p = inc.points[i].p;
        manifold->contactPoints[contactCount].id = major->points[i].id;
        ++contactCount;
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

    if (!manifold)
    {
        return true;
    }

    float distance = SafeSqrt(distance2);
    Vec3 normal = distance > epsilon ? d / distance : Vec3{ 1.0f, 0.0f, 0.0f };
    if (distance <= epsilon)
    {
        distance = radii;
    }

    manifold->contactNormal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = 0;
    manifold->referencePoint.p = pa + normal * ra;
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - distance;
    manifold->featureFlipped = false;

    return true;
}

bool BoxVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const Box* box = (const Box*)a;

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

    if (!manifold)
    {
        return true;
    }

    if (separation <= epsilon && !inside)
    {
        normal = NormalizeSafe(c - center);
        if (Length2(normal) <= epsilon)
        {
            normal = Vec3{ 1.0f, 0.0f, 0.0f };
        }
    }

    manifold->contactNormal = normal;
    manifold->contactPoints[0].id = contactID;
    manifold->contactPoints[0].p = c - normal * b->GetRadius();
    manifold->referencePoint.id = contactID;
    manifold->referencePoint.p = closest + normal * a->GetRadius();
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - separation;
    manifold->featureFlipped = false;

    return true;
}

bool CapsuleVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const Capsule* capsule = (const Capsule*)a;

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

    if (!manifold)
    {
        return true;
    }

    normal = transformA.q.Rotate(normal);
    Point supportA;
    supportA.id = t <= linear_slop ? 0 : (t >= 1.0f - linear_slop ? 1 : 0);
    supportA.p = Mul(transformA, closest) + normal * ra;

    manifold->contactNormal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = centerB - normal * rb;
    manifold->referencePoint = supportA;
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - distance;
    manifold->featureFlipped = false;

    return true;
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

    float penetrationDepth;
    if (distance <= epsilon)
    {
        normal = transformA.q.Rotate(convex->GetFaceNormals()[faceIndex]);
        distance = convex->GetRadius() - maxSeparation;
        closest = centerB - normal * distance;
        penetrationDepth = rb + distance;
    }
    else
    {
        penetrationDepth = rb - distance;
    }

    if (!manifold)
    {
        return true;
    }

    manifold->contactNormal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = centerB - normal * rb;
    manifold->referencePoint.id = convex->GetFaces()[faceIndex].indices[0];
    manifold->referencePoint.p = closest;
    manifold->contactCount = 1;
    manifold->penetrationDepth = penetrationDepth;
    manifold->featureFlipped = false;

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

            manifold->contactNormal = normal;
            manifold->contactPoints[0] = supportB;
            manifold->contactCount = 1;
            manifold->referencePoint = supportA;
            manifold->penetrationDepth = radii - gjkResult.distance;
            manifold->featureFlipped = false;

            return true;
        }
        case 2: // vertex vs. edge collision
        {
            Vec3 edge = Normalize(simplex.vertices[1].point - simplex.vertices[0].point);
            Vec3 normal = GramSchmidt(-simplex.vertices[0].point, edge);
            normal.Normalize();

            manifold->contactNormal = normal;
            manifold->penetrationDepth = radii - gjkResult.distance;
            break;
        }
        case 3: // vertex vs. face collision
        {
            Vec3 edgeA = simplex.vertices[1].point - simplex.vertices[0].point;
            Vec3 edgeB = simplex.vertices[2].point - simplex.vertices[0].point;
            Vec3 normal = Cross(edgeA, edgeB);
            normal.Normalize();

            Vec3 k = -simplex.vertices[0].point;
            if (Dot(normal, k) < 0)
            {
                normal = -normal;
            }

            manifold->contactNormal = normal;
            manifold->penetrationDepth = radii - gjkResult.distance;
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

        manifold->contactNormal = epaResult.contactNormal;
        manifold->penetrationDepth = epaResult.penetrationDepth + radii;
    }

    FindContactPoints(manifold->contactNormal, a, tfA, b, tfB, manifold);

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
    collide_function_map[Shape::capsule][Shape::capsule] = ConvexVsConvex;

    collide_function_map[Shape::box][Shape::sphere] = BoxVsSphere;
    collide_function_map[Shape::box][Shape::capsule] = ConvexVsConvex;
    collide_function_map[Shape::box][Shape::box] = ConvexVsConvex;

    collide_function_map[Shape::convex][Shape::sphere] = ConvexVsSphere;
    collide_function_map[Shape::convex][Shape::capsule] = ConvexVsConvex;
    collide_function_map[Shape::convex][Shape::box] = ConvexVsConvex;
    collide_function_map[Shape::convex][Shape::convex] = ConvexVsConvex;

    detection_function_initialized = true;
}

bool Collide(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    MuliAssert(a != nullptr);
    MuliAssert(b != nullptr);

    if (!detection_function_initialized)
    {
        InitializeDetectionFunctionMap();
    }

    if (manifold)
    {
        *manifold = ContactManifold{};
    }

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        MuliAssert(collide_function_map[shapeB][shapeA] != nullptr);

        bool collide = collide_function_map[shapeB][shapeA](b, tfB, a, tfA, manifold);
        manifold->featureFlipped = !manifold->featureFlipped;

        return collide;
    }
    else
    {
        MuliAssert(collide_function_map[shapeA][shapeB] != nullptr);

        return collide_function_map[shapeA][shapeB](a, tfA, b, tfB, manifold);
    }
}

} // namespace muli3
