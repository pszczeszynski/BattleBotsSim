#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

// This class only does something when it is destroyed
// It locks the mutext, increments the count and unlocks the mutex.
// It also notifies_all the conditional variable when it is destroyed.
class MultiThreadCleanup
{
public:
    MultiThreadCleanup(int& count_to_increment, std::mutex& mutex, std::condition_variable_any& cv);
    ~MultiThreadCleanup();

private:
    int& count_to_increment_;
    std::condition_variable_any& cv_;
    std::mutex& mutex_;
};


// Our Threadpiil
class ThreadPool 
{
public:
    ThreadPool(size_t num_threads = 0);
    ~ThreadPool();
    void enqueue(const std::function<void()>& f);
    static ThreadPool myThreads;

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop = false;
};

