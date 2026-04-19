#include <muli3/math.h>

namespace muli3
{

Quat::Quat(const Mat3& m)
{
    if (m.ez.z < 0.0f)
    {
        if (m.ex.x > m.ey.y)
        {
            float t = 1.0f + m.ex.x - m.ey.y - m.ez.z;
            *this = Quat(t, m.ex.y + m.ey.x, m.ez.x + m.ex.z, m.ey.z - m.ez.y) * (0.5f / sqrtf(t));
        }
        else
        {
            float t = 1.0f - m.ex.x + m.ey.y - m.ez.z;
            *this = Quat(m.ex.y + m.ey.x, t, m.ey.z + m.ez.y, m.ez.x - m.ex.z) * (0.5f / sqrtf(t));
        }
    }
    else
    {
        if (m.ex.x < -m.ey.y)
        {
            float t = 1.0f - m.ex.x - m.ey.y + m.ez.z;
            *this = Quat(m.ez.x + m.ex.z, m.ey.z + m.ez.y, t, m.ex.y - m.ey.x) * (0.5f / sqrtf(t));
        }
        else
        {
            float t = 1.0f + m.ex.x + m.ey.y + m.ez.z;
            *this = Quat(m.ey.z - m.ez.y, m.ez.x - m.ex.z, m.ex.y - m.ey.x, t) * (0.5f / sqrtf(t));
        }
    }
}

Mat3::Mat3(const Quat& q)
{
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xz = q.x * q.z;
    float xy = q.x * q.y;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    ex.x = 1.0f - 2.0f * (yy + zz);
    ex.y = 2.0f * (xy + wz);
    ex.z = 2.0f * (xz - wy);

    ey.x = 2.0f * (xy - wz);
    ey.y = 1.0f - 2.0f * (xx + zz);
    ey.z = 2.0f * (yz + wx);

    ez.x = 2.0f * (xz + wy);
    ez.y = 2.0f * (yz - wx);
    ez.z = 1.0f - 2.0f * (xx + yy);
}

Mat3 Mat3::GetInverse() const
{
    Mat3 t;

    float det = ex.x * (ey.y * ez.z - ey.z * ez.y) - ey.x * (ex.y * ez.z - ez.y * ex.z) + ez.x * (ex.y * ey.z - ey.y * ex.z);
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    t.ex.x = (ey.y * ez.z - ey.z * ez.y) * det;
    t.ey.x = (ez.x * ey.z - ey.x * ez.z) * det;
    t.ez.x = (ey.x * ez.y - ez.x * ey.y) * det;
    t.ex.y = (ez.y * ex.z - ex.y * ez.z) * det;
    t.ey.y = (ex.x * ez.z - ez.x * ex.z) * det;
    t.ez.y = (ex.y * ez.x - ex.x * ez.y) * det;
    t.ex.z = (ex.y * ey.z - ex.z * ey.y) * det;
    t.ey.z = (ex.z * ey.x - ex.x * ey.z) * det;
    t.ez.z = (ex.x * ey.y - ex.y * ey.x) * det;

    return t;
}

Mat4::Mat4(const Transform& t)
{
    Mat3 rotation{ t.rotation };
    ex = Vec4{ rotation.ex * t.scale.x, 0.0f };
    ey = Vec4{ rotation.ey * t.scale.y, 0.0f };
    ez = Vec4{ rotation.ez * t.scale.z, 0.0f };
    ew = Vec4{ t.position, 1.0f };
}

Mat4 Mat4::Translate(const Vec3& v) const
{
    Mat4 t;
    t.ex = ex;
    t.ey = ey;
    t.ez = ez;
    t.ew.x = ex.x * v.x + ey.x * v.y + ez.x * v.z + ew.x;
    t.ew.y = ex.y * v.x + ey.y * v.y + ez.y * v.z + ew.y;
    t.ew.z = ex.z * v.x + ey.z * v.y + ez.z * v.z + ew.z;
    t.ew.w = ew.w;
    return t;
}

Mat4 Mat4::GetInverse() const
{
    float a2323 = ez.z * ew.w - ez.w * ew.z;
    float a1323 = ez.y * ew.w - ez.w * ew.y;
    float a1223 = ez.y * ew.z - ez.z * ew.y;
    float a0323 = ez.x * ew.w - ez.w * ew.x;
    float a0223 = ez.x * ew.z - ez.z * ew.x;
    float a0123 = ez.x * ew.y - ez.y * ew.x;
    float a2313 = ey.z * ew.w - ey.w * ew.z;
    float a1313 = ey.y * ew.w - ey.w * ew.y;
    float a1213 = ey.y * ew.z - ey.z * ew.y;
    float a2312 = ey.z * ez.w - ey.w * ez.z;
    float a1312 = ey.y * ez.w - ey.w * ez.y;
    float a1212 = ey.y * ez.z - ey.z * ez.y;
    float a0313 = ey.x * ew.w - ey.w * ew.x;
    float a0213 = ey.x * ew.z - ey.z * ew.x;
    float a0312 = ey.x * ez.w - ey.w * ez.x;
    float a0212 = ey.x * ez.z - ey.z * ez.x;
    float a0113 = ey.x * ew.y - ey.y * ew.x;
    float a0112 = ey.x * ez.y - ey.y * ez.x;

    float det = ex.x * (ey.y * a2323 - ey.z * a1323 + ey.w * a1223) - ex.y * (ey.x * a2323 - ey.z * a0323 + ey.w * a0223) +
                ex.z * (ey.x * a1323 - ey.y * a0323 + ey.w * a0123) - ex.w * (ey.x * a1223 - ey.y * a0223 + ey.z * a0123);

    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    Mat4 t;
    t.ex.x = det * (ey.y * a2323 - ey.z * a1323 + ey.w * a1223);
    t.ex.y = det * -(ex.y * a2323 - ex.z * a1323 + ex.w * a1223);
    t.ex.z = det * (ex.y * a2313 - ex.z * a1313 + ex.w * a1213);
    t.ex.w = det * -(ex.y * a2312 - ex.z * a1312 + ex.w * a1212);
    t.ey.x = det * -(ey.x * a2323 - ey.z * a0323 + ey.w * a0223);
    t.ey.y = det * (ex.x * a2323 - ex.z * a0323 + ex.w * a0223);
    t.ey.z = det * -(ex.x * a2313 - ex.z * a0313 + ex.w * a0213);
    t.ey.w = det * (ex.x * a2312 - ex.z * a0312 + ex.w * a0212);
    t.ez.x = det * (ey.x * a1323 - ey.y * a0323 + ey.w * a0123);
    t.ez.y = det * -(ex.x * a1323 - ex.y * a0323 + ex.w * a0123);
    t.ez.z = det * (ex.x * a1313 - ex.y * a0313 + ex.w * a0113);
    t.ez.w = det * -(ex.x * a1312 - ex.y * a0312 + ex.w * a0112);
    t.ew.x = det * -(ey.x * a1223 - ey.y * a0223 + ey.z * a0123);
    t.ew.y = det * (ex.x * a1223 - ex.y * a0223 + ex.z * a0123);
    t.ew.z = det * -(ex.x * a1213 - ex.y * a0213 + ex.z * a0113);
    t.ew.w = det * (ex.x * a1212 - ex.y * a0212 + ex.z * a0112);
    return t;
}

Mat4 Mat4::Orth(float left, float right, float bottom, float top, float zNear, float zFar)
{
    Mat4 t{ identity };
    t.ex.x = 2.0f / (right - left);
    t.ey.y = 2.0f / (top - bottom);
    t.ez.z = -2.0f / (zFar - zNear);
    t.ew.x = -(right + left) / (right - left);
    t.ew.y = -(top + bottom) / (top - bottom);
    t.ew.z = -(zFar + zNear) / (zFar - zNear);
    return t;
}

Mat4 Mat4::Perspective(float verticalFov, float aspectRatio, float zNear, float zFar)
{
    Mat4 t{ identity };

    float tanHalfFov = tanf(verticalFov * 0.5f);
    t.ex.x = 1.0f / (aspectRatio * tanHalfFov);
    t.ey.y = 1.0f / tanHalfFov;
    t.ez.z = -(zFar + zNear) / (zFar - zNear);
    t.ez.w = -1.0f;
    t.ew.z = -(2.0f * zFar * zNear) / (zFar - zNear);
    t.ew.w = 0.0f;
    return t;
}

Mat4 Mat4::LookAt(const Vec3& eye, const Vec3& target, const Vec3& up)
{
    Vec3 backward = NormalizeSafe(eye - target);
    Vec3 right = NormalizeSafe(Cross(up, backward));
    Vec3 correctedUp = Cross(backward, right);

    Mat4 t{ identity };
    // Column-major layout: each column stores one component from every basis
    // vector.
    t.ex = Vec4{ right.x, correctedUp.x, backward.x, 0.0f };
    t.ey = Vec4{ right.y, correctedUp.y, backward.y, 0.0f };
    t.ez = Vec4{ right.z, correctedUp.z, backward.z, 0.0f };
    t.ew = Vec4{ -Dot(right, eye), -Dot(correctedUp, eye), -Dot(backward, eye), 1.0f };
    return t;
}

} // namespace muli3
