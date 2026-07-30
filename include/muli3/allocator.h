#pragma once

#include "common.h"

namespace muli3
{

inline size_t AlignUp(size_t size, size_t alignment)
{
    MuliAssert(alignment > 0);
    MuliAssert((alignment & (alignment - 1)) == 0);
    MuliAssert(size <= std::numeric_limits<size_t>::max() - (alignment - 1));
    return (size + alignment - 1) & ~(alignment - 1);
}

inline void* Alloc(size_t size)
{
    return std::malloc(size);
}

inline void Free(void* mem)
{
    std::free(mem);
}

struct Block
{
    Block* next;
};

struct Chunk
{
    int32 capacity;
    int32 blockSize;
    Block* blocks;
    Chunk* next;
};

class Allocator
{
public:
    Allocator() = default;
    virtual ~Allocator() = default;

    virtual void* Allocate(int32 size) = 0;
    virtual void Free(void* p, int32 size) = 0;
    virtual void Clear() = 0;
};

} // namespace muli3
