#pragma once

#include "parallel.h"

namespace muli3
{

class ParallelForLoop : public ParallelJob
{
public:
    ParallelForLoop(int32 begin_index, int32 end_index, int32 chunk_size, std::function<void(int32, int32)> func)
        : func{ std::move(func) }
        , next_index{ begin_index }
        , end_index{ end_index }
        , chunk_size{ chunk_size }
    {
        MuliAssert(begin_index < end_index);
    }

    virtual bool HaveWork() const override
    {
        return next_index < end_index;
    }

    virtual void RunStep(std::unique_lock<std::mutex>* lock) override;

private:
    std::function<void(int32, int32)> func;
    int32 next_index;
    const int32 end_index;
    int32 chunk_size;
};

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

} // namespace muli3
