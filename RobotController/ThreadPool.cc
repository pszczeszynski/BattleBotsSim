// Generic ThreadPool Class
//
// Creates threads that can be used to run functions
// This is to minimzie overhead of creating new threads every frame.

#include "ThreadPool.h"

ThreadPool::ThreadPool(size_t num_threads) 
{
    // Create all the threads to be used later
    // If no threads specified, set to hardware max - 2
    if( num_threads <= 0)
    {
        num_threads = std::thread::hardware_concurrency();
        if( num_threads <= 0)
        {
            // If the system couldn't define number of threads, fix it to a default value of 4
            num_threads = 4;
        }
    }

    // Create all the threads
    for (size_t i = 0; i < num_threads; ++i) 
    {
        workers.emplace_back([this] 
        {
            for (;;) 
            {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                    if (this->stop && this->tasks.empty()) return;
                    task = std::move(this->tasks.front());
                    this->tasks.pop();
                }
                task();
            }
        });
    }
}

    
void ThreadPool::enqueue(const std::function<void()>& f) 
{
    { // Block so that destructor on mutex gets called
        std::unique_lock<std::mutex> lock(queue_mutex);
        tasks.emplace(f);
    }
    condition.notify_one();
}

ThreadPool::~ThreadPool() 
{
    { // Block so that destructor on mutex gets called
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread &worker: workers) worker.join();
}
