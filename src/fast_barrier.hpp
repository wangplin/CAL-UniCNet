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

#ifndef _FAST_BARRIER_HPP_
#define _FAST_BARRIER_HPP_

#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386) || \
    defined(_M_IX86)
#if defined(_MSC_VER)
#include <intrin.h>
#define CPU_PAUSE() _mm_pause()
#elif defined(__GNUC__) || defined(__clang__)
#define CPU_PAUSE() __builtin_ia32_pause()
#else
#define CPU_PAUSE() asm volatile("pause" ::: "memory")
#endif
#elif defined(__aarch64__) || defined(_M_ARM64)
#define CPU_PAUSE() asm volatile("yield" ::: "memory")
#else
#define CPU_PAUSE() std::this_thread::yield()
#endif

class FastBarrier {
 private:
  std::atomic<int> _count;
  char _pad1[64 - sizeof(std::atomic<int>)];
  std::atomic<int> _generation;
  char _pad2[64 - sizeof(std::atomic<int>)];
  const int _expected;
  std::atomic<bool> _spinning;
  char _pad3[64 - sizeof(std::atomic<bool>)];

 public:
  explicit FastBarrier(int count)
      : _count(count), _generation(0), _expected(count), _spinning(true) {}

  void wait() {
    int gen = _generation.load(std::memory_order_acquire);

    if (_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      _count.store(_expected, std::memory_order_release);
      _generation.fetch_add(1, std::memory_order_acq_rel);
    } else {
      int spin_count = 0;
      const int MAX_SPIN = 1000;

      while (_generation.load(std::memory_order_acquire) == gen) {
        if (spin_count < MAX_SPIN) {
          for (int i = 0; i < 40; ++i) {
            CPU_PAUSE();
          }
          ++spin_count;
        } else {
          std::this_thread::yield();
        }
      }
    }
  }

  void reset() { _count.store(_expected, std::memory_order_release); }

  bool is_ready() const {
    return _count.load(std::memory_order_acquire) == _expected;
  }
};

class TaskCounter {
 private:
  std::atomic<int> _active_count;
  std::atomic<int> _pending_count;
  std::mutex _mutex;
  std::condition_variable _cv;

 public:
  TaskCounter() : _active_count(0), _pending_count(0) {}

  void add_task() { _pending_count.fetch_add(1, std::memory_order_relaxed); }

  void start_task() {
    _pending_count.fetch_sub(1, std::memory_order_relaxed);
    _active_count.fetch_add(1, std::memory_order_relaxed);
  }

  void finish_task() {
    int active = _active_count.fetch_sub(1, std::memory_order_acq_rel) - 1;
    if (active == 0 && _pending_count.load(std::memory_order_relaxed) == 0) {
      std::lock_guard<std::mutex> lock(_mutex);
      _cv.notify_one();
    }
  }

  void wait_all() {
    if (_pending_count.load(std::memory_order_acquire) == 0 &&
        _active_count.load(std::memory_order_acquire) == 0) {
      return;
    }

    std::unique_lock<std::mutex> lock(_mutex);
    _cv.wait(lock, [this]() {
      return _pending_count.load(std::memory_order_acquire) == 0 &&
             _active_count.load(std::memory_order_acquire) == 0;
    });
  }

  bool is_complete() const {
    return _pending_count.load(std::memory_order_acquire) == 0 &&
           _active_count.load(std::memory_order_acquire) == 0;
  }
};

#endif  // _FAST_BARRIER_HPP_
