#pragma once

#include "quaternion.h"
#include "vectors.h"

namespace muli3
{

struct Transform
{
    Vec3 p; // position
    Quat q; // orientation

    constexpr Transform() = default;

    constexpr Transform(Identity)
        : p{ 0 }
        , q{ identity }
    {
    }

    constexpr Transform(const Vec3& position)
        : p{ position }
        , q{ identity }
    {
    }

    constexpr Transform(const Quat& orientation)
        : p{ 0 }
        , q{ orientation }
    {
    }

    constexpr Transform(const Vec3& position, const Quat& orientation)
        : p{ position }
        , q{ orientation }
    {
    }

    constexpr Transform(Float x, Float y, Float z, const Quat& orientation = Quat(1))
        : p{ x, y, z }
        , q{ orientation }
    {
    }

    Transform(const Mat4& m)
    {
        q = Mat3(Vec3(m[0][0], m[0][1], m[0][2]), Vec3(m[1][0], m[1][1], m[1][2]), Vec3(m[2][0], m[2][1], m[2][2]));

        p.x = m[3][0];
        p.y = m[3][1];
        p.z = m[3][2];
    }

    constexpr void Set(const Vec3& position, const Quat& orientation)
    {
        p = position;
        q = orientation;
    }

    constexpr void SetIdentity()
    {
        p.SetZero();
        q.SetIdentity();
    }

    std::string ToString() const
    {
        return std::format("p:{}\nq:{}", p.ToString(), q.ToString());
    }

    constexpr Transform& operator*=(const Transform& other);

    constexpr Transform GetInverse() const
    {
        Quat invQ = q.GetConjugate();
        return Transform{ invQ.Rotate(-p), invQ };
    }

    static Transform Translate(const Vec3& position)
    {
        return Transform(position);
    }

    static Transform Rotate(const Vec3& rotation)
    {
        return Transform(Quat::FromEuler(rotation));
    }

    static Transform LookAt(const Vec3& position, const Vec3& target, const Vec3& up)
    {
        Vec3 w = target - position;
        w.Normalize();
        return Transform(position, Quat(w, up));
    }
};

constexpr inline bool operator==(const Transform& a, const Transform& b)
{
    return a.p == b.p && a.q == b.q;
}

constexpr inline Vec3 operator*(const Transform& t, const Vec3& v)
{
    return t.q.Rotate(v) + t.p;
}

// A * V
constexpr inline Vec3 Mul(const Transform& t, const Vec3& v)
{
    return t.q.Rotate(v) + t.p;
}

// A^{-1} * V
constexpr inline Vec3 MulT(const Transform& t, const Vec3& v)
{
    return t.q.RotateInv(v - t.p);
}

constexpr inline Transform operator*(const Transform& a, const Transform& b)
{
    return Transform{ a.q.Rotate(b.p) + a.p, a.q * b.q };
}

// A * B
constexpr inline Transform Mul(const Transform& a, const Transform& b)
{
    return Transform{ a.q.Rotate(b.p) + a.p, a.q * b.q };
}

// A^{-1} * B
constexpr inline Transform MulT(const Transform& a, const Transform& b)
{
    Quat invQ = a.q.GetConjugate();
    return Transform{ invQ.Rotate(b.p - a.p), invQ * b.q };
}

constexpr inline Transform& Transform::operator*=(const Transform& other)
{
    *this = Mul(*this, other);
    return *this;
}

struct Motion
{
    Motion() = default;

    constexpr Motion(Identity)
        : localCenter{ 0.0f, 0.0f, 0.0f }
        , c0{ 0.0f, 0.0f, 0.0f }
        , c{ 0.0f, 0.0f, 0.0f }
        , q0{ identity }
        , q{ identity }
        , alpha0{ 0.0f }
    {
    }

    constexpr Motion(const Transform& tf)
        : localCenter{ 0.0f, 0.0f, 0.0f }
        , c0{ tf.p }
        , c{ tf.p }
        , q0{ tf.q }
        , q{ tf.q }
        , alpha0{ 0.0f }
    {
    }

    void GetTransform(float beta, Transform* transform) const;
    void Advance(float alpha);
    void Normalize();

    Vec3 localCenter;
    Vec3 c0, c;
    Quat q0, q;
    float alpha0;
};

inline void Motion::GetTransform(float beta, Transform* transform) const
{
    Quat q1 = q;
    if (Dot(q0, q1) < 0.0f)
    {
        q1 = -q1;
    }

    transform->q = q0 * (1.0f - beta) + q1 * beta;
    transform->q.Normalize();
    transform->p = c0 * (1.0f - beta) + c * beta - transform->q.Rotate(localCenter);
}

inline void Motion::Advance(float alpha)
{
    if (alpha0 >= 1.0f)
    {
        return;
    }

    float beta = (alpha - alpha0) / (1.0f - alpha0);
    c0 += (c - c0) * beta;

    Quat q1 = q;
    if (Dot(q0, q1) < 0.0f)
    {
        q1 = -q1;
    }

    q0 = q0 * (1.0f - beta) + q1 * beta;
    q0.Normalize();
    alpha0 = alpha;
}

inline void Motion::Normalize()
{
    q0.Normalize();
    q.Normalize();
}

} // namespace muli3
