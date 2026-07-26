#pragma once

#include "floats.h"     // IWYU pragma: export
#include "matrix.h"     // IWYU pragma: export
#include "quaternion.h" // IWYU pragma: export
#include "transform.h"  // IWYU pragma: export
#include "vectors.h"    // IWYU pragma: export

namespace muli3
{

constexpr inline Vec3 x_axis{ 1, 0, 0 };
constexpr inline Vec3 y_axis{ 0, 1, 0 };
constexpr inline Vec3 z_axis{ 0, 0, 1 };

template <typename T>
constexpr inline T Sqr(T v)
{
    return v * v;
}

inline Float SafeSqrt(Float x)
{
    return std::sqrt(std::max<Float>(0, x));
}

template <typename T>
constexpr inline T Abs(T a)
{
    return a > T(0) ? a : -a;
}

template <typename T>
constexpr inline T Min(T a)
{
    return a;
}

template <typename T, typename U, typename... Args>
constexpr inline auto Min(T a, U b, Args... args)
{
    return Min(a < b ? a : b, args...);
}

template <typename T>
constexpr inline T Max(T a)
{
    return a;
}

template <typename T, typename U, typename... Args>
constexpr inline auto Max(T a, U b, Args... args)
{
    return Max(a > b ? a : b, args...);
}

template <typename T>
constexpr inline Float AbsDot(T a, T b)
{
    return std::abs(Dot(a, b));
}

constexpr inline Float DegToRad(Float deg)
{
    return deg * pi / 180;
}

constexpr inline Float RadToDeg(Float rad)
{
    return rad * inv_pi * 180;
}

template <typename T, typename U, typename V>
constexpr inline T Clamp(T v, U l, V r)
{
    return v < l ? T(l) : (v > r ? T(r) : v);
}

template <template <typename> class V, typename T>
constexpr inline V<T> Normalize(const V<T>& v)
{
    T inv_length = T(1) / Length(v);
    return v * inv_length;
}

template <template <typename> class V, typename T>
constexpr inline V<T> NormalizeSafe(const V<T>& v)
{
    T length = Length(v);
    if (length < std::numeric_limits<T>::epsilon())
    {
        return V<T>::zero;
    }

    T inv_length = T(1) / length;
    return v * inv_length;
}

constexpr inline Float SmoothStep01(Float t)
{
    t = Clamp(t, 0, 1);
    return t * t * (3 - 2 * t);
}

constexpr inline Float SmoothStep(Float a, Float b, Float x)
{
    if (a == b)
    {
        return (x < a) ? Float(0) : Float(1);
    }

    return SmoothStep01((x - a) / (b - a));
}

constexpr inline Float SmootherStep01(Float t)
{
    t = Clamp(t, 0, 1);
    return t * t * t * (t * (t * 6 - 15) + 10);
}

constexpr inline Float SmootherStep(Float a, Float b, Float x)
{
    if (a == b)
    {
        return (x < a) ? Float(0) : Float(1);
    }

    return SmootherStep01((x - a) / (b - a));
}

template <typename V, typename T>
constexpr inline V Lerp(const V& start, const V& end, T t)
{
    return start * (T(1) - t) + end * t;
}

template <typename T>
constexpr inline T Slerp(const T& start, const T& end, Float t)
{
    Float dot = Clamp(Dot(start, end), -1.0f, 1.0f);
    Float angle = std::acos(dot) * t;

    T rv = end - start * dot;
    rv.Normalize();

    return start * std::cos(angle) + rv * std::sin(angle);
}

template <typename T>
constexpr inline T Project(const T& v, const T& n)
{
    return v - n * Dot(v, n);
}

template <typename T>
constexpr inline T Reflect(const T& v, const T& n)
{
    return -v + 2 * Dot(v, n) * n;
}

template <typename Predicate>
constexpr inline int32 FindInterval(int32 size, const Predicate& pred)
{
    int32 first = 0, len = size;
    while (len > 0)
    {
        int32 half = len >> 1, middle = first + half;
        if (pred(middle))
        {
            first = middle + 1;
            len -= half + 1;
        }
        else
        {
            len = half;
        }
    }
    return Clamp(first - 1, 0, size - 2);
}

constexpr inline float NormalizeAngle(float angle)
{
    if (angle >= -pi && angle <= pi)
    {
        return angle;
    }

    angle = std::fmod(angle, two_pi);

    if (angle > pi)
    {
        angle -= two_pi;
    }
    else if (angle < -pi)
    {
        angle += two_pi;
    }

    return angle;
}

} // namespace muli3
