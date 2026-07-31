#pragma once

#include "allocator.h"

namespace muli3
{

// A pointer-stable pool. Growing the pool allocates new chunks without moving existing objects.
template <typename T>
class Pool
{
public:
    using Id = int32;

    // A power-of-two chunk capacity enables shift-and-mask ID lookup.
    explicit Pool(int32 chunkCapacity = 1024);
    ~Pool();

    Pool(const Pool&) = delete;
    Pool& operator=(const Pool&) = delete;

    template <typename... Args>
    T* New(Args&&... args);
    void Delete(T* object);

    template <typename... Args>
    Id NewId(Args&&... args);
    void Delete(Id id);

    T* Get(Id id);
    const T* Get(Id id) const;

    int32 GetCapacity() const;
    int32 GetCount() const;

private:
    struct Chunk
    {
        Block* blocks;
    };

    void* Allocate();
    void Grow();

    void* GetBlock(Id id);
    const void* GetBlock(Id id) const;

    Id GetId(const T* object) const;
    bool Contains(const T* object) const;

    int32 chunkCapacity;
    int32 chunkMask;
    int32 chunkShift;
    int32 allocationCount;
    size_t stride;

    std::vector<Chunk> chunks;
    Block* freeList;
};

template <typename T>
inline Pool<T>::Pool(int32 chunkCapacity)
    : chunkCapacity{ chunkCapacity }
    , chunkMask{ chunkCapacity - 1 }
    , chunkShift{ 0 }
    , allocationCount{ 0 }
    , stride{ 0 }
    , freeList{ nullptr }
{
    MuliAssert(chunkCapacity > 0);
    MuliAssert((chunkCapacity & (chunkCapacity - 1)) == 0 && "Capacity must be a power of 2");
    MuliAssert(alignof(T) <= alignof(std::max_align_t));

    chunkShift = std::countr_zero(uint32(chunkCapacity));

    size_t alignment = std::max(size_t(alignof(T)), size_t(alignof(Block)));
    size_t blockSize = std::max(sizeof(T), sizeof(Block));
    stride = AlignUp(blockSize, alignment);
}

template <typename T>
inline Pool<T>::~Pool()
{
    MuliAssert(allocationCount == 0);

    for (const Chunk& chunk : chunks)
    {
        muli3::Free(chunk.blocks);
    }
}

template <typename T>
template <typename... Args>
inline T* Pool<T>::New(Args&&... args)
{
    void* block = Allocate();
    T* object = new (block) T(std::forward<Args>(args)...);
    ++allocationCount;

    return object;
}

template <typename T>
inline void Pool<T>::Delete(T* object)
{
    MuliAssert(object != nullptr);
    MuliAssert(Contains(object));
    MuliAssert(allocationCount > 0);

    object->~T();

    Block* block = (Block*)object;
    block->next = freeList;
    freeList = block;
    --allocationCount;
}

template <typename T>
template <typename... Args>
inline Pool<T>::Id Pool<T>::NewId(Args&&... args)
{
    T* object = New(std::forward<Args>(args)...);
    return GetId(object);
}

template <typename T>
inline void Pool<T>::Delete(Id id)
{
    MuliAssert(0 <= id && id < GetCapacity());
    MuliAssert(allocationCount > 0);

    void* storage = GetBlock(id);
    T* object = (T*)storage;
    object->~T();

    Block* block = (Block*)storage;
    block->next = freeList;
    freeList = block;
    --allocationCount;
}

template <typename T>
inline void* Pool<T>::Allocate()
{
    Block* block = freeList;
    if (block == nullptr)
    {
        Grow();
        block = freeList;
    }

    MuliAssert(block != nullptr);

    freeList = block->next;
    return block;
}

template <typename T>
inline T* Pool<T>::Get(Id id)
{
    return (T*)GetBlock(id);
}

template <typename T>
inline const T* Pool<T>::Get(Id id) const
{
    return (const T*)GetBlock(id);
}

template <typename T>
inline void* Pool<T>::GetBlock(Id id)
{
    MuliAssert(0 <= id && id < GetCapacity());

    int32 chunkIndex = id >> chunkShift;
    int32 localIndex = id & chunkMask;
    MuliAssert(chunkIndex < int32(chunks.size()));

    return (void*)((int8*)chunks[chunkIndex].blocks + size_t(localIndex) * stride);
}

template <typename T>
inline const void* Pool<T>::GetBlock(Id id) const
{
    MuliAssert(0 <= id && id < GetCapacity());

    int32 chunkIndex = id >> chunkShift;
    int32 localIndex = id & chunkMask;
    MuliAssert(chunkIndex < int32(chunks.size()));

    return (const void*)((const int8*)chunks[chunkIndex].blocks + size_t(localIndex) * stride);
}

template <typename T>
inline typename Pool<T>::Id Pool<T>::GetId(const T* object) const
{
    MuliAssert(object != nullptr);

    uintptr_t address = (uintptr_t)object;
    size_t byteSize = size_t(chunkCapacity) * stride;

    for (int32 chunkIndex = 0; chunkIndex < int32(chunks.size()); ++chunkIndex)
    {
        uintptr_t begin = (uintptr_t)chunks[chunkIndex].blocks;
        if (address < begin || address - begin >= byteSize)
        {
            continue;
        }

        size_t offset = address - begin;
        MuliAssert(offset % stride == 0);

        int32 localIndex = int32(offset / stride);
        Id id = chunkIndex * chunkCapacity + localIndex;
        return id;
    }

    MuliAssert(false);
    return -1;
}

template <typename T>
inline bool Pool<T>::Contains(const T* object) const
{
    if (object == nullptr)
    {
        return false;
    }

    uintptr_t address = (uintptr_t)object;
    size_t byteSize = size_t(chunkCapacity) * stride;

    for (int32 i = 0; i < int32(chunks.size()); ++i)
    {
        uintptr_t begin = (uintptr_t)chunks[i].blocks;
        if (address < begin || address - begin >= byteSize)
        {
            continue;
        }

        size_t offset = address - begin;
        return offset % stride == 0;
    }

    return false;
}

template <typename T>
inline int32 Pool<T>::GetCapacity() const
{
    MuliAssert(chunks.size() <= size_t(std::numeric_limits<int32>::max()) / size_t(chunkCapacity));
    return int32(chunks.size()) * chunkCapacity;
}

template <typename T>
inline int32 Pool<T>::GetCount() const
{
    return allocationCount;
}

template <typename T>
inline void Pool<T>::Grow()
{
    MuliAssert(size_t(chunkCapacity) <= std::numeric_limits<size_t>::max() / stride);

    size_t byteSize = size_t(chunkCapacity) * stride;
    Block* blocks = (Block*)muli3::Alloc(byteSize);
    MuliAssert(blocks != nullptr);

    for (int32 i = 0; i < chunkCapacity - 1; ++i)
    {
        Block* block = (Block*)((int8*)blocks + size_t(i) * stride);
        block->next = (Block*)((int8*)blocks + size_t(i + 1) * stride);
    }

    Block* last = (Block*)((int8*)blocks + size_t(chunkCapacity - 1) * stride);
    last->next = freeList;

    chunks.push_back(Chunk{ blocks });
    freeList = blocks;

    MuliAssert(chunks.size() <= size_t(std::numeric_limits<int32>::max()) / size_t(chunkCapacity));
}

} // namespace muli3
