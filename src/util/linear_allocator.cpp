#include "muli3/linear_allocator.h"
#include "muli3/math.h"

namespace muli3
{

LinearAllocator::LinearAllocator(int32 initialCapacity)
    : entryCount{ 0 }
    , entryCapacity{ 32 }
    , capacity{ initialCapacity }
    , index{ 0 }
    , allocation{ 0 }
    , maxAllocation{ 0 }
{
    entries = (MemoryEntry*)muli3::Alloc(entryCapacity * sizeof(MemoryEntry));
    mem = (int8*)muli3::Alloc(capacity);
}

LinearAllocator::~LinearAllocator()
{
    muli3::Free(entries);
    muli3::Free(mem);
}

void* LinearAllocator::Allocate(int32 size)
{
    if (entryCount == entryCapacity)
    {
        MemoryEntry* old = entries;
        entryCapacity *= 2;
        entries = (MemoryEntry*)muli3::Alloc(entryCapacity * sizeof(MemoryEntry));
        memcpy(entries, old, entryCount * sizeof(MemoryEntry));
        muli3::Free(old);
    }

    MemoryEntry* entry = entries + entryCount;
    entry->size = size;

    int32 alignment = int32(alignof(std::max_align_t));
    int32 mask = alignment - 1;
    int32 alignedIndex = (index + mask) & ~mask;

    if (alignedIndex + size > capacity)
    {
        entry->data = (int8*)muli3::Alloc(size);
        entry->index = index;
        entry->allocationSize = size;
        entry->mallocUsed = true;
    }
    else
    {
        entry->data = mem + alignedIndex;
        entry->index = index;
        entry->allocationSize = alignedIndex + size - index;
        entry->mallocUsed = false;
        index = alignedIndex + size;
    }

    ++entryCount;
    allocation += entry->allocationSize;
    maxAllocation = Max(maxAllocation, allocation);

    return entry->data;
}

void LinearAllocator::Free(void* p, int32 size)
{
    MuliAssert(entryCount > 0);

    MemoryEntry* entry = entries + entryCount - 1;
    MuliAssert(entry->data == p);
    MuliAssert(entry->size == size);

    if (entry->mallocUsed)
    {
        muli3::Free(p);
    }
    else
    {
        index = entry->index;
    }

    allocation -= entry->allocationSize;
    --entryCount;
}

void LinearAllocator::Clear()
{
    while (entryCount > 0)
    {
        MemoryEntry* entry = entries + entryCount - 1;
        if (entry->mallocUsed)
        {
            muli3::Free(entry->data);
        }
        --entryCount;
    }

    index = 0;
    allocation = 0;
}

bool LinearAllocator::GrowMemory()
{
    if (maxAllocation <= capacity)
    {
        maxAllocation = 0;
        return false;
    }

    muli3::Free(mem);
    capacity = maxAllocation;
    mem = (int8*)muli3::Alloc(capacity);
    maxAllocation = 0;

    return true;
}

} // namespace muli3
