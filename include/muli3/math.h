#pragma once

#include "common.h"
#include "format.h"

namespace muli3
{

constexpr float pi = 3.14159265359f;
constexpr float epsilon = FLT_EPSILON;
constexpr float max_value = FLT_MAX;

struct Vec2;
struct Vec3;
struct Vec4;
struct Mat2;
struct Mat3;
struct Mat4;
struct Quat;
struct Transform;

enum Identity
{
    identity
};

struct Vec2
{
    float x, y;

    Vec2() = default;

    constexpr explicit Vec2(float s)
        : x{ s }
        , y{ s }
    {
    }

    constexpr Vec2(float x, float y)
        : x{ x }
        , y{ y }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
    }

    void Set(float s)
    {
        x = s;
        y = s;
    }

    void Set(float nx, float ny)
    {
        x = nx;
        y = ny;
    }

    float operator[](int32 i) const
    {
        return (&x)[i];
    }

    float& operator[](int32 i)
    {
        return (&x)[i];
    }

    Vec2 operator-() const
    {
        return Vec2{ -x, -y };
    }

    void operator+=(const Vec2& v)
    {
        x += v.x;
        y += v.y;
    }

    void operator-=(const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
    }

    void operator+=(float s)
    {
        x += s;
        y += s;
    }

    void operator-=(float s)
    {
        x -= s;
        y -= s;
    }

    void operator*=(float s)
    {
        x *= s;
        y *= s;
    }

    void operator/=(float s)
    {
        operator*=(1.0f / s);
    }

    float Length() const
    {
        return sqrtf(x * x + y * y);
    }

    float Length2() const
    {
        return x * x + y * y;
    }

    float Normalize()
    {
        float length = Length();
        assert(length > 0.0f);

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        return length;
    }

    float NormalizeSafe()
    {
        float length = Length();
        if (length < epsilon)
        {
            return 0.0f;
        }

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        return length;
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f", x, y);
    }

    static const Vec2 zero;
};

constexpr inline Vec2 Vec2::zero{ 0.0f };

struct Vec3
{
    float x, y, z;

    Vec3() = default;

    constexpr explicit Vec3(float s)
        : x{ s }
        , y{ s }
        , z{ s }
    {
    }

    constexpr Vec3(float x, float y, float z)
        : x{ x }
        , y{ y }
        , z{ z }
    {
    }

    Vec3(const Vec2& v)
        : x{ v.x }
        , y{ v.y }
        , z{ 0.0f }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    void Set(float s)
    {
        x = s;
        y = s;
        z = s;
    }

    void Set(float nx, float ny, float nz)
    {
        x = nx;
        y = ny;
        z = nz;
    }

    float operator[](int32 i) const
    {
        return (&x)[i];
    }

    float& operator[](int32 i)
    {
        return (&x)[i];
    }

    Vec3 operator-() const
    {
        return Vec3{ -x, -y, -z };
    }

    void operator+=(const Vec3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void operator-=(const Vec3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    void operator+=(float s)
    {
        x += s;
        y += s;
        z += s;
    }

    void operator-=(float s)
    {
        x -= s;
        y -= s;
        z -= s;
    }

    void operator*=(float s)
    {
        x *= s;
        y *= s;
        z *= s;
    }

    void operator/=(float s)
    {
        operator*=(1.0f / s);
    }

    float Length() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    float Length2() const
    {
        return x * x + y * y + z * z;
    }

    float LengthSquared() const
    {
        return Length2();
    }

    float Normalize()
    {
        float length = Length();
        assert(length > 0.0f);

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
        return length;
    }

    float NormalizeSafe()
    {
        float length = Length();
        if (length < epsilon)
        {
            return 0.0f;
        }

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
        return length;
    }

    Vec3 Normalized() const
    {
        Vec3 copy = *this;
        copy.NormalizeSafe();
        return copy;
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f\t%.4f", x, y, z);
    }

    static const Vec3 zero;
};

constexpr inline Vec3 Vec3::zero{ 0.0f };

struct Vec4
{
    float x, y, z, w;

    Vec4() = default;

    constexpr Vec4(float v, float w)
        : x{ v }
        , y{ v }
        , z{ v }
        , w{ w }
    {
    }

    constexpr Vec4(float x, float y, float z, float w)
        : x{ x }
        , y{ y }
        , z{ z }
        , w{ w }
    {
    }

    constexpr Vec4(const Vec3& v, float w)
        : x{ v.x }
        , y{ v.y }
        , z{ v.z }
        , w{ w }
    {
    }

    void SetZero()
    {
        x = y = z = w = 0.0f;
    }

    float operator[](int32 i) const
    {
        return (&x)[i];
    }

    float& operator[](int32 i)
    {
        return (&x)[i];
    }

    Vec4 operator-() const
    {
        return Vec4{ -x, -y, -z, -w };
    }

    void operator+=(const Vec4& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
    }

    void operator-=(const Vec4& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        w -= v.w;
    }

    void operator*=(float s)
    {
        x *= s;
        y *= s;
        z *= s;
        w *= s;
    }

    std::string ToString() const
    {
        return FormatString("%.4f\t%.4f\t%.4f\t%.4f", x, y, z, w);
    }

    static const Vec4 zero;
};

constexpr inline Vec4 Vec4::zero{ 0.0f, 0.0f };
constexpr inline Vec3 x_axis{ 1.0f, 0.0f, 0.0f };
constexpr inline Vec3 y_axis{ 0.0f, 1.0f, 0.0f };
constexpr inline Vec3 z_axis{ 0.0f, 0.0f, 1.0f };

struct Quat
{
    Quat() = default;

    Quat(Identity)
        : Quat(1.0f)
    {
    }

    Quat(float x, float y, float z, float w)
        : x{ x }
        , y{ y }
        , z{ z }
        , w{ w }
    {
    }

    constexpr explicit Quat(float w)
        : x{ 0.0f }
        , y{ 0.0f }
        , z{ 0.0f }
        , w{ w }
    {
    }

    Quat(const Mat3& m);

    Quat(float angle, const Vec3& unitAxis)
    {
        float halfAngle = angle * 0.5f;
        float s = sinf(halfAngle);
        x = unitAxis.x * s;
        y = unitAxis.y * s;
        z = unitAxis.z * s;
        w = cosf(halfAngle);
    }

    Quat operator-() const
    {
        return Quat{ -x, -y, -z, -w };
    }

    Quat operator*(float s) const
    {
        return Quat{ x * s, y * s, z * s, w * s };
    }

    float Length() const
    {
        return sqrtf(x * x + y * y + z * z + w * w);
    }

    float Length2() const
    {
        return x * x + y * y + z * z + w * w;
    }

    float Normalize()
    {
        float length = Length();
        assert(length > 0.0f);

        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
        w *= invLength;
        return length;
    }

    Quat Normalized() const
    {
        Quat copy = *this;
        copy.Normalize();
        return copy;
    }

    Quat GetConjugate() const
    {
        return Quat{ -x, -y, -z, w };
    }

    Quat Inversed() const
    {
        return GetConjugate();
    }

    Vec3 Rotate(const Vec3& v) const
    {
        float vx = 2.0f * v.x;
        float vy = 2.0f * v.y;
        float vz = 2.0f * v.z;
        float w2 = w * w - 0.5f;
        float dot2 = x * vx + y * vy + z * vz;

        return Vec3(
            vx * w2 + (y * vz - z * vy) * w + x * dot2, vy * w2 + (z * vx - x * vz) * w + y * dot2,
            vz * w2 + (x * vy - y * vx) * w + z * dot2
        );
    }

    void SetIdentity()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 1.0f;
    }

    static Quat FromEuler(const Vec3& eulerAngles)
    {
        float cr = cosf(eulerAngles.x * 0.5f);
        float sr = sinf(eulerAngles.x * 0.5f);
        float cp = cosf(eulerAngles.y * 0.5f);
        float sp = sinf(eulerAngles.y * 0.5f);
        float cy = cosf(eulerAngles.z * 0.5f);
        float sy = sinf(eulerAngles.z * 0.5f);

        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

struct Mat2
{
    Vec2 ex, ey;

    Mat2() = default;

    constexpr Mat2(Identity)
        : Mat2(1.0f)
    {
    }

    constexpr explicit Mat2(float v)
        : ex{ v, 0.0f }
        , ey{ 0.0f, v }
    {
    }
};

struct Mat3
{
    Vec3 ex, ey, ez;

    Mat3() = default;

    constexpr Mat3(Identity)
        : Mat3(1.0f)
    {
    }

    constexpr explicit Mat3(float v)
        : ex{ v, 0.0f, 0.0f }
        , ey{ 0.0f, v, 0.0f }
        , ez{ 0.0f, 0.0f, v }
    {
    }

    constexpr explicit Mat3(const Vec3& v)
        : ex{ v.x, 0.0f, 0.0f }
        , ey{ 0.0f, v.y, 0.0f }
        , ez{ 0.0f, 0.0f, v.z }
    {
    }

    constexpr Mat3(const Vec3& c1, const Vec3& c2, const Vec3& c3)
        : ex{ c1 }
        , ey{ c2 }
        , ez{ c3 }
    {
    }

    Mat3(const Quat& q);

    Vec3& operator[](int32 i)
    {
        return (&ex)[i];
    }

    const Vec3& operator[](int32 i) const
    {
        return (&ex)[i];
    }

    void SetIdentity()
    {
        ex = Vec3{ 1.0f, 0.0f, 0.0f };
        ey = Vec3{ 0.0f, 1.0f, 0.0f };
        ez = Vec3{ 0.0f, 0.0f, 1.0f };
    }

    void SetZero()
    {
        ex = Vec3::zero;
        ey = Vec3::zero;
        ez = Vec3::zero;
    }

    Mat3 GetTranspose() const
    {
        return Mat3{
            Vec3{ ex.x, ey.x, ez.x },
            Vec3{ ex.y, ey.y, ez.y },
            Vec3{ ex.z, ey.z, ez.z },
        };
    }

    Mat3 Transposed() const
    {
        return GetTranspose();
    }

    Mat3 GetInverse() const;

    Mat3 Inversed() const
    {
        return GetInverse();
    }

    static Mat3 Diagonal(float x, float y, float z)
    {
        return Mat3{ Vec3{ x, 0.0f, 0.0f }, Vec3{ 0.0f, y, 0.0f }, Vec3{ 0.0f, 0.0f, z } };
    }
};

struct Mat4
{
    Vec4 ex, ey, ez, ew;

    Mat4() = default;

    constexpr Mat4(Identity)
        : Mat4(1.0f)
    {
    }

    constexpr explicit Mat4(float v)
        : ex{ v, 0.0f, 0.0f, 0.0f }
        , ey{ 0.0f, v, 0.0f, 0.0f }
        , ez{ 0.0f, 0.0f, v, 0.0f }
        , ew{ 0.0f, 0.0f, 0.0f, v }
    {
    }

    constexpr Mat4(const Vec4& c1, const Vec4& c2, const Vec4& c3, const Vec4& c4)
        : ex{ c1 }
        , ey{ c2 }
        , ez{ c3 }
        , ew{ c4 }
    {
    }

    constexpr Mat4(const Mat3& r, const Vec3& p)
        : ex{ r.ex, 0.0f }
        , ey{ r.ey, 0.0f }
        , ez{ r.ez, 0.0f }
        , ew{ p, 1.0f }
    {
    }

    Mat4(const Transform& t);

    Vec4& operator[](int32 i)
    {
        return (&ex)[i];
    }

    const Vec4& operator[](int32 i) const
    {
        return (&ex)[i];
    }

    void SetIdentity()
    {
        ex = Vec4{ 1.0f, 0.0f, 0.0f, 0.0f };
        ey = Vec4{ 0.0f, 1.0f, 0.0f, 0.0f };
        ez = Vec4{ 0.0f, 0.0f, 1.0f, 0.0f };
        ew = Vec4{ 0.0f, 0.0f, 0.0f, 1.0f };
    }

    void SetZero()
    {
        ex = Vec4::zero;
        ey = Vec4::zero;
        ez = Vec4::zero;
        ew = Vec4::zero;
    }

    Mat4 GetTranspose() const
    {
        return Mat4{
            Vec4{ ex.x, ey.x, ez.x, ew.x },
            Vec4{ ex.y, ey.y, ez.y, ew.y },
            Vec4{ ex.z, ey.z, ez.z, ew.z },
            Vec4{ ex.w, ey.w, ez.w, ew.w },
        };
    }

    Mat4 GetInverse() const;

    const float* Data() const
    {
        return &ex.x;
    }

    Mat4 Scale(const Vec3& s) const;
    Mat4 Translate(const Vec3& v) const;
    static Mat4 Translation(const Vec3& v);
    static Mat4 Rotation(const Quat& q);
    static Mat4 ScaleMatrix(const Vec3& s);
    static Mat4 Orth(float left, float right, float bottom, float top, float zNear, float zFar);
    static Mat4 Perspective(float verticalFov, float aspectRatio, float zNear, float zFar);
    static Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up);
};

struct Transform
{
    Transform() = default;

    Transform(Identity)
        : position{ 0.0f }
        , rotation{ identity }
        , scale{ 1.0f }
    {
    }

    Transform(const Vec3& position)
        : position{ position }
        , rotation{ identity }
        , scale{ 1.0f }
    {
    }

    Transform(const Vec3& position, const Quat& rotation)
        : position{ position }
        , rotation{ rotation }
        , scale{ 1.0f }
    {
    }

    Transform(const Vec3& position, const Quat& rotation, const Vec3& scale)
        : position{ position }
        , rotation{ rotation }
        , scale{ scale }
    {
    }

    Vec3 position;
    Quat rotation;
    Vec3 scale{ 1.0f, 1.0f, 1.0f };
};

inline float Dot(const Vec2& a, const Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline float Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float Dot(const Vec4& a, const Vec4& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline Vec2 operator+(const Vec2& a, const Vec2& b)
{
    return Vec2{ a.x + b.x, a.y + b.y };
}

inline Vec2 operator-(const Vec2& a, const Vec2& b)
{
    return Vec2{ a.x - b.x, a.y - b.y };
}

inline Vec2 operator*(const Vec2& v, float s)
{
    return Vec2{ v.x * s, v.y * s };
}

inline Vec2 operator*(float s, const Vec2& v)
{
    return Vec2{ v.x * s, v.y * s };
}

inline Vec2 operator/(const Vec2& v, float s)
{
    return v * (1.0f / s);
}

inline Vec3 operator+(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x + b.x, a.y + b.y, a.z + b.z };
}

inline Vec3 operator-(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z };
}

inline Vec3 operator*(const Vec3& v, float s)
{
    return Vec3{ v.x * s, v.y * s, v.z * s };
}

inline Vec3 operator*(float s, const Vec3& v)
{
    return Vec3{ v.x * s, v.y * s, v.z * s };
}

inline Vec3 operator*(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x * b.x, a.y * b.y, a.z * b.z };
}

inline Vec3 operator/(const Vec3& v, float s)
{
    return v * (1.0f / s);
}

inline Vec3 operator/(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x / b.x, a.y / b.y, a.z / b.z };
}

inline Vec4 operator+(const Vec4& a, const Vec4& b)
{
    return Vec4{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

inline Vec4 operator-(const Vec4& a, const Vec4& b)
{
    return Vec4{ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

inline Vec4 operator*(const Vec4& v, float s)
{
    return Vec4{ v.x * s, v.y * s, v.z * s, v.w * s };
}

inline Vec4 operator*(float s, const Vec4& v)
{
    return Vec4{ v.x * s, v.y * s, v.z * s, v.w * s };
}

inline Vec4 operator/(const Vec4& v, float s)
{
    return v * (1.0f / s);
}

inline Quat operator*(const Quat& a, const Quat& b)
{
    return Quat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

template <typename T>
inline T Normalize(const T& v)
{
    float invLength = 1.0f / v.Length();
    return v * invLength;
}

template <typename T>
inline T NormalizeSafe(const T& v)
{
    float length = v.Length();
    if (length < epsilon)
    {
        return T::zero;
    }

    float invLength = 1.0f / length;
    return v * invLength;
}

inline Vec3 Mul(const Mat3& m, const Vec3& v)
{
    return Vec3{
        m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z,
        m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z,
        m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z,
    };
}

inline Vec3 operator*(const Mat3& m, const Vec3& v)
{
    return Mul(m, v);
}

inline Mat3 Mul(const Mat3& a, const Mat3& b)
{
    return Mat3{ a * b.ex, a * b.ey, a * b.ez };
}

inline Mat3 operator*(const Mat3& a, const Mat3& b)
{
    return Mul(a, b);
}

inline Mat3 operator*(const Mat3& m, float s)
{
    return Mat3{ m.ex * s, m.ey * s, m.ez * s };
}

inline Vec4 Mul(const Mat4& m, const Vec4& v)
{
    return Vec4{
        m.ex.x * v.x + m.ey.x * v.y + m.ez.x * v.z + m.ew.x * v.w,
        m.ex.y * v.x + m.ey.y * v.y + m.ez.y * v.z + m.ew.y * v.w,
        m.ex.z * v.x + m.ey.z * v.y + m.ez.z * v.z + m.ew.z * v.w,
        m.ex.w * v.x + m.ey.w * v.y + m.ez.w * v.z + m.ew.w * v.w,
    };
}

inline Vec4 operator*(const Mat4& m, const Vec4& v)
{
    return Mul(m, v);
}

inline Mat4 Mul(const Mat4& a, const Mat4& b)
{
    return Mat4{ a * b.ex, a * b.ey, a * b.ez, a * b.ew };
}

inline Mat4 operator*(const Mat4& a, const Mat4& b)
{
    return Mul(a, b);
}

inline Mat4 Mat4::Scale(const Vec3& s) const
{
    Mat4 t;
    t.ex = ex * s.x;
    t.ey = ey * s.y;
    t.ez = ez * s.z;
    t.ew = ew;
    return t;
}

inline Mat4 Mat4::Translation(const Vec3& v)
{
    return Mat4(identity).Translate(v);
}

inline Mat4 Mat4::Rotation(const Quat& q)
{
    return Mat4(Mat3{ q }, Vec3::zero);
}

inline Mat4 Mat4::ScaleMatrix(const Vec3& s)
{
    return Mat4(identity).Scale(s);
}

template <typename T>
inline T Min(T a, T b)
{
    return a < b ? a : b;
}

template <typename T>
inline T Max(T a, T b)
{
    return a > b ? a : b;
}

template <typename T>
inline T Clamp(T v, T minValue, T maxValue)
{
    return Max(minValue, Min(v, maxValue));
}

template <typename T>
inline T Sqr(T v)
{
    return v * v;
}

template <typename T>
inline T Abs(T v)
{
    return v >= T(0) ? v : -v;
}

inline Vec3 Min(const Vec3& a, const Vec3& b)
{
    return Vec3{ Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z) };
}

inline Vec3 Max(const Vec3& a, const Vec3& b)
{
    return Vec3{ Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z) };
}

inline float Cos(float s)
{
    return cosf(s);
}

inline float SafeSqrt(float x)
{
    return sqrtf(Max(0.0f, x));
}

inline float Sin(float s)
{
    return sinf(s);
}

inline float Tan(float s)
{
    return tanf(s);
}

inline float Acos(float s)
{
    return acosf(s);
}

inline float Asin(float s)
{
    return asinf(s);
}

inline float Atan2(float y, float x)
{
    return atan2f(y, x);
}

inline float DegToRad(float deg)
{
    return deg * pi / 180.0f;
}

inline float RadToDeg(float rad)
{
    return rad * 180.0f / pi;
}

template <typename T, typename U>
inline T Lerp(const T& start, const T& end, U t)
{
    return start * (U(1) - t) + end * t;
}

template <typename T>
inline T Project(const T& v, const T& n)
{
    return v - n * Dot(v, n);
}

inline Vec3 Mul(const Transform& t, const Vec3& v)
{
    return t.rotation.Rotate(t.scale * v) + t.position;
}

inline Vec3 MulT(const Transform& t, const Vec3& v)
{
    return t.rotation.Inversed().Rotate(v - t.position) / t.scale;
}

inline Transform Mul(const Transform& a, const Transform& b)
{
    return Transform{
        a.rotation.Rotate(a.scale * b.position) + a.position,
        a.rotation * b.rotation,
        a.scale * b.scale,
    };
}

inline Transform operator*(const Transform& a, const Transform& b)
{
    return Mul(a, b);
}

inline Vec3 Reflect(const Vec3& v, const Vec3& n)
{
    return v - 2.0f * Dot(v, n) * n;
}

inline Mat4 MakeTransformMatrix(const Transform& transform)
{
    return Mat4(transform);
}

} // namespace muli3
