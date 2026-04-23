#include "muli3/block_allocator.h"
#include "muli3/math.h"

namespace muli3
{

BlockAllocator::BlockAllocator(int32 initialChunkSize)
    : blockCount{ 0 }
    , chunkCount{ 0 }
    , chunks{ nullptr }
{
    memset(freeList, 0, sizeof(freeList));

    for (int32 i = 0; i < block_size_count; ++i)
    {
        chunkSizes[i] = Max(initialChunkSize, (i + 1) * block_unit);
    }
}

BlockAllocator::~BlockAllocator()
{
    Clear();
}

void* BlockAllocator::Allocate(int32 size)
{
    if (size == 0)
    {
        return nullptr;
    }

    if (size > max_block_size)
    {
        return muli3::Alloc(size);
    }

    int32 index = (size - 1) / block_unit;
    if (freeList[index] == nullptr)
    {
        int32 blockSize = (index + 1) * block_unit;
        int32 chunkSize = chunkSizes[index];
        int32 capacity = chunkSize / blockSize;

        Block* blocks = (Block*)muli3::Alloc(capacity * blockSize);
        memset(blocks, 0, capacity * blockSize);

        for (int32 i = 0; i < capacity - 1; ++i)
        {
            Block* block = (Block*)((int8*)blocks + blockSize * i);
            Block* next = (Block*)((int8*)blocks + blockSize * (i + 1));
            block->next = next;
        }

        Block* last = (Block*)((int8*)blocks + blockSize * (capacity - 1));
        last->next = nullptr;

        Chunk* chunk = (Chunk*)muli3::Alloc(sizeof(Chunk));
        chunk->capacity = capacity;
        chunk->blockSize = blockSize;
        chunk->blocks = blocks;
        chunk->next = chunks;
        chunks = chunk;
        ++chunkCount;

        freeList[index] = chunk->blocks;
        chunkSizes[index] += chunkSizes[index] / 2;
    }

    Block* block = freeList[index];
    freeList[index] = block->next;
    ++blockCount;

    return block;
}

void BlockAllocator::Free(void* p, int32 size)
{
    if (p == nullptr)
    {
        return;
    }

    if (size > max_block_size)
    {
        muli3::Free(p);
        return;
    }

    MuliAssert(blockCount > 0);

    int32 index = (size - 1) / block_unit;
    Block* block = (Block*)p;
    block->next = freeList[index];
    freeList[index] = block;
    --blockCount;
}

void BlockAllocator::Clear()
{
    Chunk* chunk = chunks;
    while (chunk)
    {
        Chunk* c0 = chunk;
        chunk = c0->next;
        muli3::Free(c0->blocks);
        muli3::Free(c0);
    }

    chunks = nullptr;
    blockCount = 0;
    chunkCount = 0;
    memset(freeList, 0, sizeof(freeList));
}

void BlockAllocator::Clear(int32 initialChunkSize)
{
    Clear();

    for (int32 i = 0; i < block_size_count; ++i)
    {
        chunkSizes[i] = Max(initialChunkSize, (i + 1) * block_unit);
    }
}

int32 BlockAllocator::GetChunkSize(int32 size) const
{
    MuliAssert(0 < size && size <= max_block_size);
    return chunkSizes[(size - 1) / block_unit];
}

} // namespace muli3
