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

#ifndef _MEMORY_POOL_HPP_
#define _MEMORY_POOL_HPP_

#include <vector>
#include <map>
#include <set>
#include <mutex>
#include <memory>

template <typename T>
class ThreadSafeMemoryPool {
 private:
  std::vector<T> _pool;
  std::vector<bool> _used;
  std::mutex _mutex;
  size_t _next_free;

 public:
  ThreadSafeMemoryPool(size_t initial_size = 1000)
      : _pool(initial_size), _used(initial_size, false), _next_free(0) {}

  T* allocate() {
    std::lock_guard<std::mutex> lock(_mutex);

    for (size_t i = _next_free; i < _used.size(); ++i) {
      if (!_used[i]) {
        _used[i] = true;
        _next_free = i + 1;
        return &_pool[i];
      }
    }

    for (size_t i = 0; i < _next_free; ++i) {
      if (!_used[i]) {
        _used[i] = true;
        _next_free = i + 1;
        return &_pool[i];
      }
    }

    size_t old_size = _pool.size();
    _pool.resize(old_size * 2);
    _used.resize(old_size * 2, false);

    _used[old_size] = true;
    _next_free = old_size + 1;
    return &_pool[old_size];
  }

  void deallocate(T* ptr) {
    std::lock_guard<std::mutex> lock(_mutex);

    size_t index = ptr - &_pool[0];
    if (index < _used.size()) {
      _used[index] = false;
      if (index < _next_free) {
        _next_free = index;
      }
    }
  }
};

template <typename Key, typename Value>
class PreallocatedMap {
 private:
  std::vector<Value> _values;
  std::vector<unsigned char> _used;
  std::vector<int> _active_keys;
  size_t _size;

 public:
  PreallocatedMap(size_t max_size = 100) : _size(0) {
    _values.resize(max_size);
    _used.resize(max_size, 0);
    _active_keys.reserve(max_size);
  }

  Value& operator[](const Key& key) {
    int k = static_cast<int>(key);

    if (k >= static_cast<int>(_values.size())) {
      size_t new_size = k + 100;
      _values.resize(new_size);
      _used.resize(new_size, 0);
    }

    if (!_used[k]) {
      _used[k] = 1;
      _active_keys.push_back(k);
      _size++;
    }

    return _values[k];
  }

  size_t count(const Key& key) const {
    int k = static_cast<int>(key);
    if (k >= 0 && k < static_cast<int>(_used.size())) {
      return _used[k] ? 1 : 0;
    }
    return 0;
  }

  bool empty() const { return _size == 0; }

  size_t size() const { return _size; }

  void clear() {
    if (_size == 0) {
      return;
    }

    for (int key : _active_keys) {
      _used[key] = 0;
      _values[key] = Value();
    }
    _active_keys.clear();
    _size = 0;
  }

  class iterator {
   private:
    std::vector<Value>* _values;
    std::vector<int>* _active_keys;
    size_t _pos;

   public:
    iterator(std::vector<Value>* values, std::vector<int>* active_keys,
             size_t pos)
        : _values(values), _active_keys(active_keys), _pos(pos) {}

    bool operator!=(const iterator& other) const { return _pos != other._pos; }

    iterator& operator++() {
      _pos++;
      return *this;
    }

    std::pair<int, Value&> operator*() {
      int key = (*_active_keys)[_pos];
      return std::pair<int, Value&>(key, (*_values)[key]);
    }
  };

  class const_iterator {
   private:
    const std::vector<Value>* _values;
    const std::vector<int>* _active_keys;
    size_t _pos;

   public:
    const_iterator(const std::vector<Value>* values,
                   const std::vector<int>* active_keys, size_t pos)
        : _values(values), _active_keys(active_keys), _pos(pos) {}

    bool operator!=(const const_iterator& other) const {
      return _pos != other._pos;
    }

    const_iterator& operator++() {
      _pos++;
      return *this;
    }

    std::pair<int, const Value&> operator*() const {
      int key = (*_active_keys)[_pos];
      return std::pair<int, const Value&>(key, (*_values)[key]);
    }
  };

  iterator begin() { return iterator(&_values, &_active_keys, 0); }

  iterator end() {
    return iterator(&_values, &_active_keys, _active_keys.size());
  }

  const_iterator begin() const {
    return const_iterator(&_values, &_active_keys, 0);
  }

  const_iterator end() const {
    return const_iterator(&_values, &_active_keys, _active_keys.size());
  }
};

template <typename T>
class PreallocatedSet {
  static_assert(std::is_integral<T>::value,
                "PreallocatedSet requires integral keys");

 private:
  std::vector<T> _active;
  std::vector<int> _positions;

  void _ensure_capacity(size_t idx) {
    if (idx >= _positions.size()) {
      size_t new_size = idx + 64;
      _positions.resize(new_size, -1);
    }
  }

 public:
  PreallocatedSet(size_t max_size = 0) {
    size_t initial = max_size ? max_size : 1;
    _positions.assign(initial, -1);
    _active.reserve(initial);
  }

  void insert(const T& value) {
    size_t idx = static_cast<size_t>(value);
    _ensure_capacity(idx);
    if (_positions[idx] != -1) {
      return;
    }
    _positions[idx] = static_cast<int>(_active.size());
    _active.push_back(value);
  }

  void clear() {
    for (T value : _active) {
      size_t idx = static_cast<size_t>(value);
      if (idx < _positions.size()) {
        _positions[idx] = -1;
      }
    }
    _active.clear();
  }

  bool empty() const { return _active.empty(); }

  size_t size() const { return _active.size(); }

  class iterator {
   private:
    std::vector<T>* _values;
    size_t _pos;

   public:
    iterator(std::vector<T>* values, size_t pos) : _values(values), _pos(pos) {}

    bool operator!=(const iterator& other) const { return _pos != other._pos; }

    iterator& operator++() {
      ++_pos;
      return *this;
    }

    const T& operator*() const { return (*_values)[_pos]; }
  };

  class const_iterator {
   private:
    const std::vector<T>* _values;
    size_t _pos;

   public:
    const_iterator(const std::vector<T>* values, size_t pos)
        : _values(values), _pos(pos) {}

    bool operator!=(const const_iterator& other) const {
      return _pos != other._pos;
    }

    const_iterator& operator++() {
      ++_pos;
      return *this;
    }

    const T& operator*() const { return (*_values)[_pos]; }
  };

  iterator begin() { return iterator(&_active, 0); }

  iterator end() { return iterator(&_active, _active.size()); }

  const_iterator begin() const { return const_iterator(&_active, 0); }

  const_iterator end() const {
    return const_iterator(&_active, _active.size());
  }
};

#endif  // _MEMORY_POOL_HPP_
