#pragma once

#include "matrix.h"
#include "vectors.h"

namespace muli3
{

struct Quat
{
    Float x, y, z, w;

    constexpr Quat() = default;

    constexpr Quat(Identity)
        : Quat(1)
    {
    }

    constexpr Quat(Float x, Float y, Float z, Float w)
        : x{ x }
        , y{ y }
        , z{ z }
        , w{ w }
    {
    }

    explicit constexpr Quat(Float w)
        : x{ 0 }
        , y{ 0 }
        , z{ 0 }
        , w{ w }
    {
    }

    Quat(const Mat3& m)
    {
        // https://math.stackexchange.com/questions/893984/conversion-of-rotation-matrix-to-quaternion
        if (m.ez.z < 0)
        {
            if (m.ex.x > m.ey.y)
            {
                Float t = 1 + m.ex.x - m.ey.y - m.ez.z;
                *this = Quat(t, m.ex.y + m.ey.x, m.ez.x + m.ex.z, m.ey.z - m.ez.y) * (0.5f / std::sqrt(t));
            }
            else
            {
                Float t = 1 - m.ex.x + m.ey.y - m.ez.z;
                *this = Quat(m.ex.y + m.ey.x, t, m.ey.z + m.ez.y, m.ez.x - m.ex.z) * (0.5f / std::sqrt(t));
            }
        }
        else
        {
            if (m.ex.x < -m.ey.y)
            {
                Float t = 1 - m.ex.x - m.ey.y + m.ez.z;
                *this = Quat(m.ez.x + m.ex.z, m.ey.z + m.ez.y, t, m.ex.y - m.ey.x) * (0.5f / std::sqrt(t));
            }
            else
            {
                Float t = 1 + m.ex.x + m.ey.y + m.ez.z;
                *this = Quat(m.ey.z - m.ez.y, m.ez.x - m.ex.z, m.ex.y - m.ey.x, t) * (0.5f / std::sqrt(t));
            }
        }
    }

    Quat(const Vec3& front, const Vec3& up)
    {
        Mat3 rotation;

        rotation.ez = front;
        rotation.ex = Cross(up, rotation.ez);
        rotation.ex.Normalize();
        rotation.ey = Cross(rotation.ez, rotation.ex);

        *this = Quat(rotation);
    }

    // Axis must be normalized
    Quat(Float angle, const Vec3& unitAxis)
    {
        Float halfAngle = angle * 0.5f;

        Float s = std::sin(halfAngle);
        x = unitAxis.x * s;
        y = unitAxis.y * s;
        z = unitAxis.z * s;
        w = std::cos(halfAngle);
    }

    constexpr Quat operator-() const
    {
        return Quat(-x, -y, -z, -w);
    }

    constexpr Quat operator*(Float s) const
    {
        return Quat(x * s, y * s, z * s, w * s);
    }

    constexpr bool IsIdentity() const
    {
        return x == 0 && y == 0 && z == 0 && w == 1;
    }

    constexpr Float Length2() const
    {
        return x * x + y * y + z * z + w * w;
    }

    Float Length() const
    {
        return std::sqrt(Length2());
    }

    Float Normalize()
    {
        Float length = Length();
        if (length < epsilon)
        {
            return 0;
        }

        Float invLength = 1 / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
        w *= invLength;

        return length;
    }

    constexpr Quat GetConjugate() const
    {
        return Quat(-x, -y, -z, w);
    }

    constexpr Vec3 GetImaginaryPart() const
    {
        return Vec3(x, y, z);
    }

    // Optimized qvq'
    constexpr Vec3 Rotate(const Vec3& v) const
    {
        Float vx = 2 * v.x;
        Float vy = 2 * v.y;
        Float vz = 2 * v.z;
        Float w2 = w * w - 0.5f;

        Float dot2 = (x * vx + y * vy + z * vz);

        return Vec3(
            (vx * w2 + (y * vz - z * vy) * w + x * dot2), (vy * w2 + (z * vx - x * vz) * w + y * dot2),
            (vz * w2 + (x * vy - y * vx) * w + z * dot2)
        );
    }

    constexpr Vec3 RotateInv(const Vec3& v) const
    {
        Float vx = 2 * v.x;
        Float vy = 2 * v.y;
        Float vz = 2 * v.z;
        Float w2 = w * w - 0.5f;

        Float dot2 = (x * vx + y * vy + z * vz);

        return Vec3(
            (vx * w2 - (y * vz - z * vy) * w + x * dot2), (vy * w2 - (z * vx - x * vz) * w + y * dot2),
            (vz * w2 - (x * vy - y * vx) * w + z * dot2)
        );
    }

    constexpr void SetIdentity()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 1;
    }

    // Computes rotation of x-axis
    constexpr Vec3 GetBasisX() const
    {
        Float x2 = x * 2;
        Float w2 = w * 2;

        return Vec3((w * w2) - 1 + x * x2, (z * w2) + y * x2, (-y * w2) + z * x2);
    }

    // Computes rotation of y-axis
    constexpr Vec3 GetBasisY() const
    {
        Float y2 = y * 2;
        Float w2 = w * 2;

        return Vec3((-z * w2) + x * y2, (w * w2) - 1 + y * y2, (x * w2) + z * y2);
    }

    // Computes rotation of z-axis
    constexpr Vec3 GetBasisZ() const
    {
        Float z2 = z * 2;
        Float w2 = w * 2;

        return Vec3((y * w2) + x * z2, (-x * w2) + y * z2, (w * w2) - 1 + z * z2);
    }

    Vec3 ToEuler() const
    {
        // Roll (x-axis)
        Float sinrCosp = 2 * (w * x + y * z);
        Float cosrCosp = 1 - 2 * (x * x + y * y);
        Float roll = std::atan2(sinrCosp, cosrCosp);

        // Pitch (y-axis)
        Float sinp = 2 * (w * y - z * x);
        Float pitch;
        if (std::abs(sinp) >= 1)
        {
            pitch = std::copysign(pi / 2, sinp); // use 90 degrees if out of range
        }
        else
        {
            pitch = std::asin(sinp);
        }

        // Yaw (z-axis)
        Float sinyCosp = 2 * (w * z + x * y);
        Float cosyCosp = 1 - 2 * (y * y + z * z);
        Float yaw = std::atan2(sinyCosp, cosyCosp);

        return Vec3{ roll, pitch, yaw };
    }

    static Quat FromEuler(const Vec3& eulerAngles)
    {
        Float cr = std::cos(eulerAngles.x * 0.5f);
        Float sr = std::sin(eulerAngles.x * 0.5f);
        Float cp = std::cos(eulerAngles.y * 0.5f);
        Float sp = std::sin(eulerAngles.y * 0.5f);
        Float cy = std::cos(eulerAngles.z * 0.5f);
        Float sy = std::sin(eulerAngles.z * 0.5f);

        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    std::string ToString() const
    {
        return ToEuler().ToString();
    }

    static const Quat zero;
};

const inline Quat Quat::zero{ 0.0f };

// Quat inline functions begin

constexpr inline bool operator==(const Quat& a, const Quat& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

constexpr inline Float Dot(const Quat& a, const Quat& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// Quaternion multiplication
constexpr inline Quat operator*(const Quat& a, const Quat& b)
{
    // clang-format off
    return Quat(a.w * b.x + b.w * a.x + a.y * b.z - b.y * a.z,
                a.w * b.y + b.w * a.y + a.z * b.x - b.z * a.x,
                a.w * b.z + b.w * a.z + a.x * b.y - b.x * a.y,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
    // clang-format on
}

constexpr inline Quat operator+(const Quat& a, const Quat& b)
{
    return Quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

constexpr inline Quat operator-(const Quat& a, const Quat& b)
{
    return Quat(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

// Compute angle between two quaternions
inline Float Angle(const Quat& a, const Quat& b)
{
    return std::acos(Dot(a, b)) * 2;
}

// Quat inline functions end

} // namespace muli3
