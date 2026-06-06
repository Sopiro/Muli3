#include "muli3/parallel.h"
#include "muli3/parallel_for.h"
#include "muli3/profile.h"

namespace muli3
{

ThreadPool::ThreadPool(int32 worker_count)
{
    worker_count = std::max(worker_count, 1);

    // Calling thread also participates in executing parallel work,
    // so we launches one fewer than the requested number of threads.
    for (int32 i = 0; i < worker_count - 1; ++i)
    {
        threads.emplace_back(&ThreadPool::Worker, this, i + 1);
    }
}

ThreadPool::~ThreadPool()
{
    if (threads.empty())
    {
        return;
    }

    shutdown.store(true, std::memory_order_release);
    current_job.fetch_add(1, std::memory_order_release);
    job_list_condition.notify_all();

    for (std::thread& thread : threads)
    {
        thread.join();
    }
}

void ThreadPool::Worker(int32 worker_index)
{
    MuliProfileSetThreadName("Worker");

    while (shutdown.load(std::memory_order_acquire) == false)
    {
        uint32 job = current_job.load(std::memory_order_acquire);

        if (TryRunJob(worker_index))
        {
            continue;
        }

        WaitForNextJob(job);
    }
}

bool ThreadPool::TryRunJob(int32 worker_index)
{
    ParallelJob* job = nullptr;

    {
        std::lock_guard<SpinLock> lock(job_lock);

        // The list lock is only held while choosing a job.
        // The actual work runs without holding the lock.
        for (ParallelJob* candidate = job_list; candidate; candidate = candidate->next)
        {
            if (candidate->HaveWork())
            {
                job = candidate;
                job->active_workers.fetch_add(1, std::memory_order_relaxed);
                break;
            }
        }
    }

    if (job == nullptr)
    {
        return false;
    }

    job->RunStep(worker_index);

    bool completed = false;

    // Keep the detach and completion test together so a stack job
    // cannot be destroyed while another worker still holds its pointer.
    {
        std::lock_guard<SpinLock> lock(job_lock);
        int32 workers = job->active_workers.fetch_sub(1, std::memory_order_acquire) - 1;
        if (workers == 0 && job->HaveWork() == false)
        {
            RemoveJob(job);
            job->completed.store(true, std::memory_order_release);
            completed = true;
        }
    }

    if (completed)
    {
        current_job.fetch_add(1, std::memory_order_release);
        job_list_condition.notify_all();
    }

    return true;
}

void ThreadPool::WaitForNextJob(uint32 job)
{
    // Spin briefly for the next solver job.
    for (int32 tries = 0; tries < 512; ++tries)
    {
        if (current_job.load(std::memory_order_acquire) != job || shutdown.load(std::memory_order_acquire))
        {
            return;
        }

        Pause();
    }

    if (spin_mode.load(std::memory_order_relaxed))
    {
        return;
    }

    // Workers sleep until a job/state change is published.
    std::unique_lock<SpinLock> lock(job_lock);
    while (current_job.load(std::memory_order_acquire) == job && !shutdown.load(std::memory_order_acquire))
    {
        job_list_condition.wait(lock);
    }
}

bool ThreadPool::WorkOrReturn(int32 worker_index)
{
    // Return false if we do nothing
    return TryRunJob(worker_index);
}

bool ThreadPool::SetSpinMode(bool enable)
{
    bool old_spin_mode = spin_mode.exchange(enable, std::memory_order_acq_rel);

    // Wake sleeping workers and release spinning workers when the mode changes.
    current_job.fetch_add(1, std::memory_order_release);
    job_list_condition.notify_all();

    return old_spin_mode;
}

void ThreadPool::AddJob(ParallelJob* job)
{
    job->thread_pool = this;
    job->active_workers.store(0, std::memory_order_relaxed);
    job->completed.store(false, std::memory_order_release);
    job->prev = nullptr;
    job->next = nullptr;

    {
        std::lock_guard<SpinLock> lock(job_lock);

        // New jobs are appended so older jobs keep priority.
        if (job_list_tail)
        {
            job_list_tail->next = job;
            job->prev = job_list_tail;
        }
        else
        {
            job_list = job;
        }

        job_list_tail = job;
    }

    // Wake sleeping workers or release spinning workers waiting for the next job.
    current_job.fetch_add(1, std::memory_order_release);
    job_list_condition.notify_all();
}

void ThreadPool::RemoveJob(ParallelJob* job)
{
    // The job lock must be held before calling this function
    if (job->prev)
    {
        job->prev->next = job->next;
    }
    else
    {
        job_list = job->next;
    }

    if (job->next)
    {
        job->next->prev = job->prev;
    }
    else
    {
        job_list_tail = job->prev;
    }

    job->prev = nullptr;
    job->next = nullptr;
}

void ThreadPool::ForEachThread(std::function<void(void)> func)
{
    int32 worker_count = WorkerCount();
    std::latch latch(worker_count);

    ParallelFor(
        0, worker_count,
        [&](int32) {
            func();
            latch.arrive_and_wait();
        },
        this
    );
}

} // namespace muli3
