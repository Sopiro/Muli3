#include "muli3/parallel_for.h"

namespace muli3
{

void ParallelForLoop::RunStep(std::unique_lock<std::mutex>* lock, int32 worker_index)
{
    lock->unlock();

    while (true)
    {
        int32 block = next_block.fetch_add(1, std::memory_order_relaxed);
        if (block >= block_count)
        {
            break;
        }

        int32 index_begin = begin_index + block * block_size;
        int32 index_end = std::min(index_begin + block_size, end_index);
        func(index_begin, index_end, worker_index);
    }

    lock->lock();

    // Remove job from list after all work has been claimed.
    if (removed == false && HaveWork() == false)
    {
        thread_pool->RemoveJob(this);
        removed = true;
    }

    lock->unlock();
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

    // Compute block size for parallel loop
    const int32 blocks_per_worker = 4;
    int32 max_block_count = blocks_per_worker * thread_pool->WorkerCount();

    int32 block_size;
    int32 block_count;
    if (item_count <= min_range * max_block_count)
    {
        block_size = min_range;
        block_count = (item_count + block_size - 1) / block_size;
    }
    else
    {
        block_size = (item_count + max_block_count - 1) / max_block_count;
        block_count = (item_count + block_size - 1) / block_size;
    }

    // It's safe to allocate loop on the stack
    // Because this ParallelFor() call does not return until all work for the loop is done.
    ParallelForLoop loop(begin, end, block_size, block_count, std::move(func));
    std::unique_lock<std::mutex> lock = thread_pool->AddJob(&loop);

    // Current thread also work on the job
    while (!loop.Finished())
    {
        thread_pool->WorkOrWait(&lock, 0);
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
