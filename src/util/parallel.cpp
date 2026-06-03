#include "muli3/parallel.h"
#include "muli3/parallel_for.h"
#include "muli3/profile.h"

namespace muli3
{

ThreadPool::ThreadPool(int32 worker_count)
{
    worker_count = std::max(worker_count, 2);

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

    {
        std::lock_guard<std::mutex> lock(mutex);
        shutdown = true;
        job_list_condition.notify_all();
    }

    for (std::thread& thread : threads)
    {
        thread.join();
    }
}

void ThreadPool::Worker(int32 worker_index)
{
    MuliProfileSetThreadName("Worker");

    std::unique_lock<std::mutex> lock(mutex);

    while (!shutdown)
    {
        WorkOrWait(&lock, worker_index);
    }
}

void ThreadPool::WorkOrWait(std::unique_lock<std::mutex>* lock, int32 worker_index)
{
    MuliAssert(lock->owns_lock() == true);

    // Pick one job that still has work left
    ParallelJob* job = job_list;
    while (job && job->HaveWork() == false)
    {
        job = job->next;
    }

    if (job)
    {
        // Execute work for this job
        job->active_workers++;
        job->RunStep(lock, worker_index);

        // Detach from this job
        MuliAssert(lock->owns_lock() == false);
        lock->lock();
        job->active_workers--;

        // If the job is completed,
        // we must signal condition variable for the thread that initially add the work.
        // That initial thread may be waiting on the condition variable for other threads to finish their work on the job.
        if (job->Finished())
        {
            job_list_condition.notify_all();
        }
    }
    else
    {
        // MuliProfileZoneN(wait, "Wait", true);
        // Wait for new work to arrive or the job to finish
        job_list_condition.wait(*lock);
        // MuliProfileZoneEnd(wait);
    }
}

bool ThreadPool::WorkOrReturn(int32 worker_index)
{
    // Return false if we do nothing

    std::unique_lock<std::mutex> lock(mutex);

    ParallelJob* job = job_list;
    while (job && job->HaveWork() == false)
    {
        job = job->next;
    }

    if (job == nullptr)
    {
        return false;
    }

    job->active_workers++;
    job->RunStep(&lock, worker_index);

    MuliAssert(lock.owns_lock() == false);
    lock.lock();
    job->active_workers--;

    if (job->Finished())
    {
        job_list_condition.notify_all();
    }

    return true;
}

std::unique_lock<std::mutex> ThreadPool::AddJob(ParallelJob* job)
{
    job->thread_pool = this;

    std::unique_lock<std::mutex> lock(mutex);

    // Link job to tail of list
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

    // Notify to all workers
    job_list_condition.notify_all();

    return lock;
}

void ThreadPool::RemoveJob(ParallelJob* job)
{
    // The lock must be held before calling this function
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
