// Copyright (c) 2026 Sun Yat-sen University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _THREAD_POOL_HPP_
#define _THREAD_POOL_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <stdexcept>
#include <atomic>
#include <utility>

class LightweightBarrier {
 private:
  std::atomic<int> _count;
  std::atomic<int> _generation;
  const int _expected;

 public:
  LightweightBarrier(int count)
      : _count(count), _generation(0), _expected(count) {}

  void wait() {
    int gen = _generation.load(std::memory_order_acquire);
    if (_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      _count.store(_expected, std::memory_order_release);
      _generation.fetch_add(1, std::memory_order_acq_rel);
    } else {
      while (_generation.load(std::memory_order_acquire) == gen) {
        std::this_thread::yield();
      }
    }
  }

  void reset() { _count.store(_expected, std::memory_order_release); }
};

class ThreadPool {
 public:
  ThreadPool(size_t num_threads);

  template <class F>
  void enqueue(F&& f);

  void wait_for_all();
  ~ThreadPool();

 private:
  std::vector<std::thread> workers;

  std::queue<std::function<void()>> tasks;

  std::mutex queue_mutex;
  std::condition_variable condition;
  std::atomic<bool> stop;

  std::atomic<int> active_tasks;
  std::atomic<int> pending_tasks;

  std::unique_ptr<LightweightBarrier> barrier;
};

inline ThreadPool::ThreadPool(size_t num_threads)
    : stop(false), active_tasks(0), pending_tasks(0) {
  for (size_t i = 0; i < num_threads; ++i) {
    workers.emplace_back([this] {
      for (;;) {
        std::function<void()> task;

        {
          std::unique_lock<std::mutex> lock(this->queue_mutex);
          this->condition.wait(lock, [this] {
            return this->stop.load(std::memory_order_acquire) ||
                   !this->tasks.empty();
          });

          if (this->stop.load(std::memory_order_acquire) && this->tasks.empty())
            return;

          task = std::move(this->tasks.front());
          this->tasks.pop();

          this->pending_tasks.fetch_sub(1, std::memory_order_relaxed);
          this->active_tasks.fetch_add(1, std::memory_order_relaxed);
        }

        task();

        this->active_tasks.fetch_sub(1, std::memory_order_release);
      }
    });
  }
}

template <class F>
void ThreadPool::enqueue(F&& f) {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // Don't allow enqueueing after stopping the pool
    if (stop.load(std::memory_order_acquire))
      throw std::runtime_error("enqueue on stopped ThreadPool");

    tasks.emplace(std::forward<F>(f));
    pending_tasks.fetch_add(1, std::memory_order_relaxed);
  }
  condition.notify_one();
}

inline void ThreadPool::wait_for_all() {
  while (pending_tasks.load(std::memory_order_acquire) > 0 ||
         active_tasks.load(std::memory_order_acquire) > 0) {
    std::this_thread::yield();
  }
}

inline ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop.store(true, std::memory_order_release);
  }
  condition.notify_all();
  for (std::thread& worker : workers)
    worker.join();
}

#endif  // _THREAD_POOL_HPP_
