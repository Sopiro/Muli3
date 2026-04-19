#pragma once

#include "common.h"

namespace muli3
{

inline std::string FormatString(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    va_list copy;
    va_copy(copy, args);
    const int size = std::vsnprintf(nullptr, 0, fmt, copy);
    va_end(copy);

    std::string result((size_t)size, '\0');
    std::vsnprintf(result.data(), result.size() + 1, fmt, args);
    va_end(args);

    return result;
}

} // namespace muli3
