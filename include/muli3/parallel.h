#pragma once

#include "common.h"

#if defined(_M_IX86) || defined(_M_X64)
    #include <intrin.h>
#elif defined(__i386__) || defined(__x86_64__)
    #include <immintrin.h>
#endif

namespace muli3
{

class ThreadPool;

inline void Pause() noexcept
{
#if defined(_M_IX86) || defined(_M_X64) || defined(__i386__) || defined(__x86_64__)
    _mm_pause();
#elif defined(_M_ARM64) || defined(_M_ARM)
    __yield();
#elif defined(__aarch64__) || defined(__arm__)
    asm volatile("yield" ::: "memory");
#else
    std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}

class SpinLock
{
public:
    void lock()
    {
        int spin = 1;

        while (flag.test_and_set(std::memory_order_acquire))
        {
            while (flag.test(std::memory_order_relaxed))
            {
                for (int i = 0; i < spin; ++i)
                {
                    Pause();
                }

                if (spin < 64)
                {
                    spin *= 2;
                }
            }
        }
    }

    bool try_lock()
    {
        return !flag.test_and_set(std::memory_order_acquire);
    }

    void unlock()
    {
        flag.clear(std::memory_order_release);
    }

private:
    alignas(64) std::atomic_flag flag = ATOMIC_FLAG_INIT;
};

class ParallelJob
{
public:
    virtual ~ParallelJob() = default;

    virtual bool HaveWork() const = 0;
    virtual void RunStep(int32 worker_index) = 0;

    bool Finished() const
    {
        return completed.load(std::memory_order_acquire);
    }

protected:
    friend class ThreadPool;
    ThreadPool* thread_pool;

private:
    // Active threads working on this job
    std::atomic<int32> active_workers = 0;
    std::atomic_bool completed = false;

    // Links
    ParallelJob* prev = nullptr;
    ParallelJob* next = nullptr;
};

class ThreadPool
{
public:
    inline static std::unique_ptr<ThreadPool> global_thread_pool = nullptr;

    explicit ThreadPool(int32 worker_count);
    ~ThreadPool();

    bool WorkOrReturn(int32 worker_index = 0);

    void AddJob(ParallelJob* job);
    void RemoveJob(ParallelJob* job);

    bool SetSpinMode(bool enable);

    void ForEachThread(std::function<void(void)> func);

    int32 WorkerCount() const
    {
        return int32(threads.size() + 1);
    }

private:
    void Worker(int32 worker_index);
    bool TryRunJob(int32 worker_index);
    void WaitForNextJob(uint32 current_job);

    std::vector<std::thread> threads;

    std::atomic_bool shutdown = false;
    std::atomic_bool spin_mode = false;

    // Atomic counter for job update
    std::atomic<uint32> current_job = 0;

    SpinLock job_lock;
    std::condition_variable_any job_list_condition;
    ParallelJob* job_list = nullptr;
    ParallelJob* job_list_tail = nullptr;
};

class SpinScope
{
public:
    SpinScope(ThreadPool* threadPool)
        : threadPool{ threadPool }
        , oldSpinMode{ false }
    {
        if (threadPool)
        {
            oldSpinMode = threadPool->SetSpinMode(true);
        }
    }

    ~SpinScope()
    {
        Close();
    }

    void Close()
    {
        if (threadPool)
        {
            threadPool->SetSpinMode(oldSpinMode);
            threadPool = nullptr;
        }
    }

private:
    ThreadPool* threadPool;
    bool oldSpinMode;
};

template <typename T>
class ThreadLocal
{
public:
    ThreadLocal()
        : hash_table{ 4 * std::thread::hardware_concurrency() }
        , createFcn{ []() { return T(); } }
    {
    }

    ThreadLocal(std::function<T(void)> createFcn)
        : hash_table{ 4 * std::thread::hardware_concurrency() }
        , createFcn{ std::move(createFcn) }
    {
    }

    T& Get();

    void ForEach(std::function<void(std::thread::id tid, T& value)>&& callback);

private:
    struct Entry
    {
        std::thread::id tid;
        T value;
    };

    std::shared_mutex mutex;
    std::vector<std::optional<Entry>> hash_table;
    std::function<T(void)> createFcn;
};

template <typename T>
inline T& ThreadLocal<T>::Get()
{
    const std::thread::id tid = std::this_thread::get_id();
    size_t hash = std::hash<std::thread::id>()(tid);
    hash %= hash_table.size();

    int32 step = 1;
    int32 tries = 0;
    MuliNotUsed(tries);

    mutex.lock_shared();
    while (true)
    {
        MuliAssert(size_t(tries) < hash_table.size());

        if (hash_table[hash].has_value())
        {
            if (hash_table[hash]->tid == tid)
            {
                // Found
                T& local_value = hash_table[hash]->value;
                mutex.unlock_shared();
                return local_value;
            }
            else
            {
                // Check the next bucket
                hash += step;
                ++step;

                if (hash >= hash_table.size())
                {
                    hash %= hash_table.size();
                }

                ++tries;
                continue;
            }
        }
        else
        {
            // First access

            // We get exclusive lock before calling callback so that the user
            // doesn't have to worry about writing a thread-safe callback.
            mutex.unlock_shared();
            mutex.lock();

            T new_value = createFcn();

            if (hash_table[hash].has_value())
            {
                // Resolve hash collsion by linear probing.
                while (true)
                {
                    hash += step;
                    ++step;

                    if (hash >= hash_table.size())
                    {
                        hash %= hash_table.size();
                    }

                    if (!hash_table[hash].has_value())
                    {
                        break;
                    }
                }
            }

            hash_table[hash].emplace(tid, std::move(new_value));
            T& local_value = hash_table[hash]->value;

            mutex.unlock();

            return local_value;
        }
    }
}

template <typename T>
inline void ThreadLocal<T>::ForEach(std::function<void(std::thread::id tid, T& value)>&& callback)
{
    mutex.lock();
    for (auto& entry : hash_table)
    {
        if (entry.has_value())
        {
            callback(entry->tid, entry->value);
        }
    }
    mutex.unlock();
}

} // namespace muli3
