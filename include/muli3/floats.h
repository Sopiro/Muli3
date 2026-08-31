#pragma once

#include "common.h"

typedef float Float;

namespace muli3
{

inline constexpr Float pi = Float(3.14159265358979323846);
inline constexpr Float two_pi = Float(2 * pi);
inline constexpr Float four_pi = Float(4 * pi);
inline constexpr Float inv_pi = Float(1 / pi);
inline constexpr Float inv_two_pi = Float(1 / (2 * pi));
inline constexpr Float inv_four_pi = Float(1 / (4 * pi));
inline constexpr Float epsilon = std::numeric_limits<Float>::epsilon();
inline constexpr Float infinity = std::numeric_limits<Float>::infinity();
inline constexpr Float max_float = std::numeric_limits<Float>::max();

inline bool IsNullish(int32 v)
{
    MuliNotUsed(v);
    return false;
}

inline bool IsNullish(Float v)
{
    return std::isnan(v) || std::isinf(v);
}

template <typename T>
inline bool IsNullish(const T& v)
{
    return v.IsNullish();
}

#define CheckNull(v)                                                                                                             \
    if (IsNullish(v))                                                                                                            \
    {                                                                                                                            \
        std::cout << #v;                                                                                                         \
        std::cout << " null" << std::endl;                                                                                       \
        MuliAssert(false);                                                                                                       \
    }

} // namespace muli3
