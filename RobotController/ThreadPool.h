#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


class ThreadPool 
{
public:
    ThreadPool(size_t num_threads = 0);
    ~ThreadPool();
    void enqueue(const std::function<void()>& f);

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop = false;
};
