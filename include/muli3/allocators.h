#pragma once

#include "allocator.h"

namespace muli3
{

class LinearAllocator : public Allocator
{
public:
    LinearAllocator(int32 initialCapacity = 16 * 1024);
    ~LinearAllocator();

    LinearAllocator(const LinearAllocator&) = delete;
    LinearAllocator& operator=(const LinearAllocator&) = delete;

    void* Allocate(int32 size) override;
    void Free(void* p, int32 size) override;
    void Clear() override;

    bool GrowMemory();

    int32 GetCapacity() const;
    int32 GetAllocation() const;
    int32 GetMaxAllocation() const;

private:
    struct MemoryEntry
    {
        int8* data;
        int32 size;
        int32 index;
        int32 allocationSize;
        bool mallocUsed;
    };

    MemoryEntry* entries;
    int32 entryCount;
    int32 entryCapacity;

    int8* mem;
    int32 capacity;
    int32 index;

    int32 allocation;
    int32 maxAllocation;
};

inline int32 LinearAllocator::GetCapacity() const
{
    return capacity;
}

inline int32 LinearAllocator::GetAllocation() const
{
    return allocation;
}

inline int32 LinearAllocator::GetMaxAllocation() const
{
    return maxAllocation;
}

class PoolAllocator
{
public:
    using PoolId = int32;
    using SlotId = int32;

    PoolAllocator(int32 defaultChunkByteSize = 16 * 1024);
    ~PoolAllocator();

    PoolAllocator(const PoolAllocator&) = delete;
    PoolAllocator& operator=(const PoolAllocator&) = delete;

    void Clear();

    template <typename T>
    PoolId Register(int32 chunkCapacity = 0);

    template <typename T>
    T* Allocate();
    template <typename T>
    void Free(T* p);

    template <typename T, typename... Args>
    T* New(Args&&... args);
    template <typename T>
    void Delete(T* p);

    template <typename T>
    T* Get(SlotId id);
    template <typename T>
    SlotId GetId(T* p);

    template <typename T>
    int32 GetSlotCount();

    int32 GetPoolCount() const;

private:
    struct PoolChunk
    {
        Block* blocks;
    };

    struct Pool
    {
        int32 stride;
        int32 chunkCapacity;
        int32 allocationCount;
        std::vector<PoolChunk> chunks;
        Block* freeList;
    };

    struct TypePoolEntry
    {
        const void* key;
        PoolId poolId;
    };

    template <typename T>
    PoolId GetPool();

    PoolId CreatePool(int32 elementSize, int32 chunkCapacity = 0, int32 alignment = alignof(std::max_align_t));

    void* AllocateFromPool(PoolId poolId);
    void FreeFromPool(PoolId poolId, void* p);

    void* GetFromPool(PoolId poolId, SlotId id);
    SlotId GetIdFromPool(PoolId poolId, const void* p);

    int32 GetSlotCount(PoolId poolId);

    PoolId GetPool(int32 elementSize, int32 alignment, const void* key, int32 chunkCapacity);
    void GrowPool(PoolId poolId);

    int32 defaultChunkByteSize;

    std::vector<Pool> pools;
    std::vector<TypePoolEntry> typePools;
};

inline int32 PoolAllocator::GetPoolCount() const
{
    return int32(pools.size());
}

template <typename T>
inline PoolAllocator::PoolId PoolAllocator::Register(int32 chunkCapacity)
{
    MuliAssert(alignof(T) <= alignof(std::max_align_t));

    static int32 typeKey;
    return GetPool(sizeof(T), alignof(T), &typeKey, chunkCapacity);
}

template <typename T>
inline T* PoolAllocator::Allocate()
{
    return (T*)AllocateFromPool(GetPool<T>());
}

template <typename T>
inline void PoolAllocator::Free(T* p)
{
    FreeFromPool(GetPool<T>(), p);
}

template <typename T, typename... Args>
inline T* PoolAllocator::New(Args&&... args)
{
    return new (Allocate<T>()) T(std::forward<Args>(args)...);
}

template <typename T>
inline void PoolAllocator::Delete(T* p)
{
    if (p == nullptr)
    {
        return;
    }

    p->~T();
    Free(p);
}

template <typename T>
inline T* PoolAllocator::Get(SlotId id)
{
    return (T*)GetFromPool(GetPool<T>(), id);
}

template <typename T>
inline PoolAllocator::SlotId PoolAllocator::GetId(T* p)
{
    return GetIdFromPool(GetPool<T>(), p);
}

template <typename T>
inline int32 PoolAllocator::GetSlotCount()
{
    return GetSlotCount(GetPool<T>());
}

template <typename T>
inline PoolAllocator::PoolId PoolAllocator::GetPool()
{
    return Register<T>();
}

} // namespace muli3