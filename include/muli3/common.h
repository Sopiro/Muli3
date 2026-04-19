#pragma once

#include "types.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace muli3
{

class NonCopyable
{
public:
    NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};

} // namespace muli3

#define MuliAssert(A) assert(A)
#define MuliNotUsed(x) ((void)(x))
