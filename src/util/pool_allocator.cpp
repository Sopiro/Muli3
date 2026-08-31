#include "muli3/allocators.h"
#include "muli3/math.h"

namespace muli3
{

PoolAllocator::PoolAllocator(int32 defaultChunkByteSize)
    : defaultChunkByteSize{ defaultChunkByteSize }
{
    MuliAssert(defaultChunkByteSize > 0);
}

PoolAllocator::~PoolAllocator()
{
    Clear();
}

void PoolAllocator::Clear()
{
    for (const Pool& pool : pools)
    {
        MuliNotUsed(pool);
        MuliAssert(pool.allocationCount == 0);
    }

    for (Pool& pool : pools)
    {
        for (PoolChunk& chunk : pool.chunks)
        {
            muli3::Free(chunk.blocks);
        }

        pool.chunks.clear();
        pool.freeList = nullptr;
        pool.allocationCount = 0;
    }
}

PoolAllocator::PoolId PoolAllocator::CreatePool(int32 elementSize, int32 chunkCapacity, int32 alignment)
{
    MuliAssert(elementSize > 0);
    MuliAssert(alignment > 0);
    MuliAssert((alignment & (alignment - 1)) == 0);
    MuliAssert(alignment <= int32(alignof(std::max_align_t)));

    int32 stride = Max(elementSize, int32(sizeof(Block)));
    int32 blockAlignment = Max(alignment, int32(alignof(Block)));
    int32 mask = blockAlignment - 1;
    stride = (stride + mask) & ~mask;

    if (chunkCapacity <= 0)
    {
        chunkCapacity = Max(defaultChunkByteSize / stride, 1);
    }

    Pool pool{};
    pool.stride = stride;
    pool.chunkCapacity = chunkCapacity;
    pool.freeList = nullptr;

    PoolId poolId = int32(pools.size());
    pools.push_back(pool);

    return poolId;
}

void* PoolAllocator::AllocateFromPool(PoolId poolId)
{
    MuliAssert(0 <= poolId && poolId < int32(pools.size()));

    Pool& pool = pools[poolId];
    if (pool.freeList == nullptr)
    {
        GrowPool(poolId);
    }

    MuliAssert(pool.freeList != nullptr);

    Block* block = pool.freeList;
    pool.freeList = block->next;
    ++pool.allocationCount;

    return block;
}

void PoolAllocator::FreeFromPool(PoolId poolId, void* p)
{
    if (p == nullptr)
    {
        return;
    }

    MuliAssert(0 <= poolId && poolId < int32(pools.size()));

    Pool& pool = pools[poolId];
    MuliAssert(pool.allocationCount > 0);
    MuliAssert(GetIdFromPool(poolId, p) >= 0);

    Block* block = (Block*)p;
    block->next = pool.freeList;
    pool.freeList = block;
    --pool.allocationCount;
}

void* PoolAllocator::GetFromPool(PoolId poolId, SlotId id)
{
    MuliAssert(0 <= poolId && poolId < int32(pools.size()));
    MuliAssert(id >= 0);

    Pool& pool = pools[poolId];
    int32 chunkIndex = id / pool.chunkCapacity;
    int32 localIndex = id - chunkIndex * pool.chunkCapacity;

    MuliAssert(0 <= chunkIndex && chunkIndex < int32(pool.chunks.size()));

    PoolChunk& chunk = pool.chunks[chunkIndex];
    return (int8*)chunk.blocks + localIndex * pool.stride;
}

PoolAllocator::SlotId PoolAllocator::GetIdFromPool(PoolId poolId, const void* p)
{
    MuliAssert(0 <= poolId && poolId < int32(pools.size()));
    MuliAssert(p != nullptr);

    Pool& pool = pools[poolId];
    int8* ptr = (int8*)p;
    for (int32 i = 0; i < int32(pool.chunks.size()); ++i)
    {
        PoolChunk& chunk = pool.chunks[i];
        int8* begin = (int8*)chunk.blocks;
        int8* end = begin + pool.chunkCapacity * pool.stride;

        if (begin <= ptr && ptr < end)
        {
            ptrdiff_t offset = ptr - begin;
            MuliAssert(offset % pool.stride == 0);
            return i * pool.chunkCapacity + int32(offset / pool.stride);
        }
    }

    MuliAssert(false);
    return -1;
}

int32 PoolAllocator::GetSlotCount(PoolId poolId)
{
    MuliAssert(0 <= poolId && poolId < int32(pools.size()));

    Pool& pool = pools[poolId];
    return int32(pool.chunks.size()) * pool.chunkCapacity;
}

PoolAllocator::PoolId PoolAllocator::GetPool(int32 elementSize, int32 alignment, const void* key, int32 chunkCapacity)
{
    for (TypePoolEntry entry : typePools)
    {
        if (entry.key == key)
        {
            if (chunkCapacity > 0)
            {
                MuliAssert(pools[entry.poolId].chunkCapacity == chunkCapacity);
            }
            return entry.poolId;
        }
    }

    PoolId poolId = CreatePool(elementSize, chunkCapacity, alignment);
    typePools.push_back(TypePoolEntry{ key, poolId });
    return poolId;
}

void PoolAllocator::GrowPool(PoolId poolId)
{
    Pool& pool = pools[poolId];
    int32 chunkCapacity = pool.chunkCapacity;

    MuliAssert(chunkCapacity > 0);
    MuliAssert(pool.stride > 0);

    Block* blocks = (Block*)muli3::Alloc(chunkCapacity * pool.stride);

    for (int32 i = 0; i < chunkCapacity - 1; ++i)
    {
        Block* block = (Block*)((int8*)blocks + pool.stride * i);
        block->next = (Block*)((int8*)blocks + pool.stride * (i + 1));
    }

    Block* last = (Block*)((int8*)blocks + pool.stride * (chunkCapacity - 1));
    last->next = pool.freeList;

    pool.chunks.emplace_back(blocks);

    pool.freeList = blocks;
}

} // namespace muli3
