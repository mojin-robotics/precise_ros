#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

template <typename T>
class Queue
{
public:
    explicit Queue(int max_size = 0) : max_size_(max_size)
    {
    }

    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(lock);
        }
        auto item = queue_.front();
        queue_.pop();
        return item;
    }

    void pop(T &item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (queue_.empty())
        {
            cond_.wait(lock);
        }
        item = queue_.front();
        queue_.pop();
    }

    void push(const T &item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (max_size_ > 0 && queue_.size() >= max_size_)
            queue_.pop();

        queue_.push(item);
        lock.unlock();
        cond_.notify_one();
    }

    void push(T &&item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (max_size_ > 0 && queue_.size() >= max_size_)
            queue_.pop();

        queue_.push(std::move(item));
        lock.unlock();
        cond_.notify_one();
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while(queue_.size() > 0)
            queue_.pop();
    }

    void setMaxSize(int size)
    {
        max_size_ = size;
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    int max_size_;
};