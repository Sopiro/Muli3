#pragma once

#include "parallel.h"

namespace muli3
{

class ParallelForLoop : public ParallelJob
{
public:
    ParallelForLoop(
        int32 begin_index, int32 end_index, int32 block_size, int32 block_count, std::function<void(int32, int32, int32)> func
    )
        : func{ std::move(func) }
        , begin_index{ begin_index }
        , end_index{ end_index }
        , block_size{ block_size }
        , block_count{ block_count }
    {
        MuliAssert(begin_index < end_index);
        MuliAssert(block_size > 0);
        MuliAssert(block_count > 0);
    }

    virtual bool HaveWork() const override
    {
        return next_block.load(std::memory_order_relaxed) < block_count;
    }

    virtual void RunStep(std::unique_lock<std::mutex>* lock, int32 worker_index) override;

private:
    std::function<void(int32, int32, int32)> func;
    std::atomic<int32> next_block = 0;

    const int32 begin_index;
    const int32 end_index;

    int32 block_size;
    int32 block_count;

    bool removed = false;
};

void ParallelFor(
    int32 begin,
    int32 end,
    int32 minRange,
    std::function<void(int32 begin, int32 end, int32 worker_index)> func,
    ThreadPool* thread_pool = ThreadPool::global_thread_pool.get()
);

void ParallelFor(
    int32 begin,
    int32 end,
    int32 minRange,
    std::function<void(int32 begin, int32 end)> func,
    ThreadPool* thread_pool = ThreadPool::global_thread_pool.get()
);

void ParallelFor(
    int32 begin,
    int32 end,
    std::function<void(int32 begin, int32 end)> func,
    ThreadPool* thread_pool = ThreadPool::global_thread_pool.get()
);

inline void ParallelFor(
    int32 begin, int32 end, std::function<void(int32 i)> func, ThreadPool* thread_pool = ThreadPool::global_thread_pool.get()
)
{
    ParallelFor(
        begin, end,
        [&func](int32 begin, int32 end) {
            for (int32 i = begin; i < end; ++i)
            {
                func(i);
            }
        },
        thread_pool
    );
}

inline void ParallelFor(
    int32 begin,
    int32 end,
    int32 minRange,
    std::function<void(int32 i)> func,
    ThreadPool* thread_pool = ThreadPool::global_thread_pool.get()
)
{
    ParallelFor(
        begin, end, minRange,
        [&func](int32 begin, int32 end) {
            for (int32 i = begin; i < end; ++i)
            {
                func(i);
            }
        },
        thread_pool
    );
}

} // namespace muli3
