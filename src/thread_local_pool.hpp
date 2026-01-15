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

#ifndef _THREAD_LOCAL_POOL_HPP_
#define _THREAD_LOCAL_POOL_HPP_

#include <stack>
#include <mutex>
#include <thread>

template <typename T>
class ThreadLocalPool {
 private:
  static thread_local std::stack<T*> _local_free;
  static thread_local bool _initialized;

  static std::stack<T*> _global_free;
  static std::mutex _global_mutex;
  static std::stack<T*> _all;

  static constexpr size_t BATCH_SIZE = 128;

  static void InitializeLocalPool() {
    if (_initialized)
      return;

    std::lock_guard<std::mutex> lock(_global_mutex);
    for (size_t i = 0; i < BATCH_SIZE; ++i) {
      T* obj = new T();
      _all.push(obj);
      _local_free.push(obj);
    }
    _initialized = true;
  }

  static void RefillLocalPool() {
    std::lock_guard<std::mutex> lock(_global_mutex);

    size_t transfer_count = std::min(BATCH_SIZE / 2, _global_free.size());
    for (size_t i = 0; i < transfer_count; ++i) {
      _local_free.push(_global_free.top());
      _global_free.pop();
    }

    size_t remaining = BATCH_SIZE / 2 - transfer_count;
    for (size_t i = 0; i < remaining; ++i) {
      T* obj = new T();
      _all.push(obj);
      _local_free.push(obj);
    }
  }

 public:
  static T* Allocate() {
    if (!_initialized) {
      InitializeLocalPool();
    }

    if (_local_free.empty()) {
      RefillLocalPool();
    }

    T* obj = _local_free.top();
    _local_free.pop();
    return obj;
  }

  static void Deallocate(T* obj) {
    if (!obj)
      return;

    if (!_initialized) {
      InitializeLocalPool();
    }

    if (_local_free.size() >= BATCH_SIZE * 3) {
      std::lock_guard<std::mutex> lock(_global_mutex);
      for (size_t i = 0; i < BATCH_SIZE; ++i) {
        _global_free.push(_local_free.top());
        _local_free.pop();
      }
    }

    _local_free.push(obj);
  }

  static void FreeAll() {
    std::lock_guard<std::mutex> lock(_global_mutex);
    while (!_all.empty()) {
      delete _all.top();
      _all.pop();
    }
    while (!_global_free.empty()) {
      _global_free.pop();
    }
  }

  static size_t GetTotalAllocated() {
    std::lock_guard<std::mutex> lock(_global_mutex);
    return _all.size();
  }
};

template <typename T>
thread_local std::stack<T*> ThreadLocalPool<T>::_local_free;

template <typename T>
thread_local bool ThreadLocalPool<T>::_initialized = false;

template <typename T>
std::stack<T*> ThreadLocalPool<T>::_global_free;

template <typename T>
std::mutex ThreadLocalPool<T>::_global_mutex;

template <typename T>
std::stack<T*> ThreadLocalPool<T>::_all;

#endif  // _THREAD_LOCAL_POOL_HPP_
