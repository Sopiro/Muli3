#include "muli3/collision.h"
#include "muli3/box.h"
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

static Vec3 GetPerpendicular(const Vec3& v)
{
    Vec3 axis = Abs(v.x) < Abs(v.y) ? x_axis : y_axis;
    if (Abs(v.z) < Abs(Dot(v, axis)))
    {
        axis = z_axis;
    }

    Vec3 p = Cross(v, axis);
    if (Length2(p) <= epsilon)
    {
        p = Cross(v, x_axis);
    }

    return NormalizeSafe(p);
}

static bool AddEPAFace(EPAFace* faces, int32* faceCount, const SupportPoint* vertices, int32 a, int32 b, int32 c)
{
    if (*faceCount >= epa_max_face_count)
    {
        return false;
    }

    Vec3 pa = vertices[a].point;
    Vec3 pb = vertices[b].point;
    Vec3 pc = vertices[c].point;

    Vec3 normal = Cross(pb - pa, pc - pa);
    float length = normal.Normalize();
    if (length <= epsilon)
    {
        return false;
    }

    float distance = Dot(normal, pa);
    if (distance < 0.0f)
    {
        std::swap(b, c);
        normal = -normal;
        distance = -distance;
    }

    faces[*faceCount] = EPAFace{ a, b, c, normal, distance, false };
    ++(*faceCount);
    return true;
}

static bool AddEPAEdge(EPAEdge* edges, int32* edgeCount, int32 a, int32 b)
{
    for (int32 i = 0; i < *edgeCount; ++i)
    {
        if (edges[i].a == b && edges[i].b == a)
        {
            edges[i] = edges[*edgeCount - 1];
            --(*edgeCount);
            return true;
        }
    }

    if (*edgeCount >= epa_max_edge_count)
    {
        return false;
    }

    edges[*edgeCount] = EPAEdge{ a, b };
    ++(*edgeCount);
    return true;
}

static bool ExpandSimplexToTetrahedron(
    const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, Simplex* simplex
)
{
    switch (simplex->count)
    {
    case 1:
    {
        SupportPoint support = CSOSupport(a, tfA, b, tfB, x_axis);
        if (support.point == simplex->vertices[0].point)
        {
            support = CSOSupport(a, tfA, b, tfB, -x_axis);
        }

        simplex->AddVertex(support);
    }

        [[fallthrough]];

    case 2:
    {
        Vec3 e = simplex->vertices[1].point - simplex->vertices[0].point;
        Vec3 normal = GetPerpendicular(e);
        SupportPoint support = CSOSupport(a, tfA, b, tfB, normal);
        if (support.point == simplex->vertices[0].point || support.point == simplex->vertices[1].point)
        {
            support = CSOSupport(a, tfA, b, tfB, -normal);
        }

        simplex->AddVertex(support);
    }

        [[fallthrough]];

    case 3:
    {
        Vec3 a0 = simplex->vertices[0].point;
        Vec3 b0 = simplex->vertices[1].point;
        Vec3 c0 = simplex->vertices[2].point;

        Vec3 normal = Cross(b0 - a0, c0 - a0);
        if (Length2(normal) <= epsilon)
        {
            normal = GetPerpendicular(b0 - a0);
        }
        else
        {
            normal.Normalize();
        }

        SupportPoint support = CSOSupport(a, tfA, b, tfB, normal);
        if (support.point == simplex->vertices[0].point || support.point == simplex->vertices[1].point ||
            support.point == simplex->vertices[2].point)
        {
            support = CSOSupport(a, tfA, b, tfB, -normal);
        }

        simplex->AddVertex(support);
    }

        [[fallthrough]];

    case 4:
        return true;

    default:
        MuliAssert(false);
        return false;
    }
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
    Simplex tetrahedron = simplex;
    if (!ExpandSimplexToTetrahedron(a, tfA, b, tfB, &tetrahedron))
    {
        result->contactNormal = NormalizeSafe(simplex.GetSearchDirection());
        result->penetrationDepth = 0.0f;
        return;
    }

    SupportPoint vertices[epa_max_vertex_count];
    int32 vertexCount = tetrahedron.count;
    for (int32 i = 0; i < vertexCount; ++i)
    {
        vertices[i] = tetrahedron.vertices[i];
    }

    EPAFace faces[epa_max_face_count];
    int32 faceCount = 0;
    AddEPAFace(faces, &faceCount, vertices, 0, 1, 2);
    AddEPAFace(faces, &faceCount, vertices, 0, 3, 1);
    AddEPAFace(faces, &faceCount, vertices, 0, 2, 3);
    AddEPAFace(faces, &faceCount, vertices, 1, 3, 2);

    if (faceCount == 0)
    {
        result->contactNormal = NormalizeSafe(simplex.GetSearchDirection());
        result->penetrationDepth = 0.0f;
        return;
    }

    EPAFace best = faces[0];

    for (int32 k = 0; k < epa_max_iteration; ++k)
    {
        int32 bestIndex = -1;
        float bestDistance = max_float;
        for (int32 i = 0; i < faceCount; ++i)
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

        EPAEdge edges[epa_max_edge_count];
        int32 edgeCount = 0;

        for (int32 i = 0; i < faceCount; ++i)
        {
            EPAFace& face = faces[i];
            if (face.removed)
            {
                continue;
            }

            if (Dot(face.normal, support.point - vertices[face.a].point) > 0.0f)
            {
                face.removed = true;
                if (!AddEPAEdge(edges, &edgeCount, face.a, face.b) || !AddEPAEdge(edges, &edgeCount, face.b, face.c) ||
                    !AddEPAEdge(edges, &edgeCount, face.c, face.a))
                {
                    result->contactNormal = best.normal;
                    result->penetrationDepth = best.distance;
                    return;
                }
            }
        }

        for (int32 i = 0; i < edgeCount; ++i)
        {
            AddEPAFace(faces, &faceCount, vertices, edges[i].a, edges[i].b, newIndex);
        }
    }

    result->contactNormal = best.normal;
    result->penetrationDepth = best.distance;
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

bool ConvexVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    return false;
}

void InitializeDetectionFunctionMap()
{
    if (detection_function_initialized)
    {
        return;
    }

    collide_function_map[Shape::sphere][Shape::sphere] = SphereVsSphere;
    collide_function_map[Shape::box][Shape::sphere] = BoxVsSphere;
    collide_function_map[Shape::box][Shape::box] = ConvexVsConvex;

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
