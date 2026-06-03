#include "muli3/parallel_for.h"

namespace muli3
{

void ParallelForLoop::RunStep(int32 worker_index)
{
    // Claim one block at a time so workers share the loop through an atomic counter.
    int32 block = next_block.fetch_add(1, std::memory_order_relaxed);
    if (block >= block_count)
    {
        return;
    }

    int32 index_begin = begin_index + block * block_size;
    int32 index_end = std::min(index_begin + block_size, end_index);
    func(index_begin, index_end, worker_index);
}

void ParallelFor(int32 begin, int32 end, int32 min_range, std::function<void(int32, int32, int32)> func, ThreadPool* thread_pool)
{
    if (begin == end)
    {
        return;
    }

    MuliAssert(begin < end);
    MuliAssert(min_range > 0);

    if (!thread_pool)
    {
        func(begin, end, 0);
        return;
    }

    int32 item_count = end - begin;
    if (item_count <= min_range)
    {
        func(begin, end, 0);
        return;
    }

    // Compute block size for parallel loop
    const int32 blocks_per_worker = 8;
    int32 max_block_count = std::max(1, blocks_per_worker * thread_pool->WorkerCount());
    int32 block_count = (item_count + min_range - 1) / min_range;
    block_count = std::max(1, std::min(block_count, max_block_count));

    int32 block_size = (item_count + block_count - 1) / block_count;

    // It's safe to allocate loop on the stack
    // Because this ParallelFor() call does not return until all work for the loop is done.
    ParallelForLoop loop(begin, end, block_size, block_count, std::move(func));
    thread_pool->AddJob(&loop);

    // The calling thread helps workers instead of sleeping on the loop.
    int32 spin = 1;
    int32 tries = 0;
    while (!loop.Finished())
    {
        if (thread_pool->WorkOrReturn(0))
        {
            spin = 2;
            tries = 0;
        }
        else if (tries < 128)
        {
            for (int32 i = 0; i < spin; ++i)
            {
                Pause();
            }

            if (spin < 64)
            {
                spin *= 2;
            }

            ++tries;
        }
        else
        {
            std::this_thread::yield();
        }
    }
}

void ParallelFor(int32 begin, int32 end, int32 min_range, std::function<void(int32, int32)> func, ThreadPool* thread_pool)
{
    ParallelFor(begin, end, min_range, [&func](int32 begin, int32 end, int32) { func(begin, end); }, thread_pool);
}

void ParallelFor(int32 begin, int32 end, std::function<void(int32, int32)> func, ThreadPool* thread_pool)
{
    ParallelFor(begin, end, 1, std::move(func), thread_pool);
}

} // namespace muli3
