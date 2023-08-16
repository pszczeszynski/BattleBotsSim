#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

class MatQueue
{
private:
    std::queue<cv::Mat> queue;
    std::mutex mtx;
    std::condition_variable cv;
    const size_t MAX_SIZE = 10; // Maximum number of Mats in the queue

public:
    void produce(const cv::Mat &mat)
    {
        std::unique_lock<std::mutex> lock(mtx);

        // Wait if the queue is full
        cv.wait(lock, [this]
                { return queue.size() < MAX_SIZE; });

        // Add the Mat to the queue
        queue.emplace(mat.clone());

        // Notify a waiting consumer
        cv.notify_one();
    }

    bool consume(cv::Mat &mat)
    {
        std::unique_lock<std::mutex> lock(mtx);

        // Wait if the queue is empty
        if (cv.wait_for(lock, std::chrono::seconds(1), [this]
                        { return !queue.empty(); }))
        {
            // Get the Mat from the queue
            mat = queue.front();
            queue.pop();

            // Notify a waiting producer
            cv.notify_one();

            return true;
        }

        return false;
    }

    bool consumeLatestAndClear(cv::Mat &mat)
    {
        std::unique_lock<std::mutex> lock(mtx);

        // Wait if the queue is empty
        if (cv.wait_for(lock, std::chrono::seconds(1), [this]
                        { return !queue.empty(); }))
        {
            // Get the Mat from the queue
            mat = queue.back();
            while (!queue.empty())
            {
                queue.back().release();
                queue.pop();
                // Notify a waiting producer
                cv.notify_one();
            }

            return true;
        }

        return false;
    }
};
