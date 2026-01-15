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

#ifndef _PREALLOCATED_SEPARABLE_HPP_
#define _PREALLOCATED_SEPARABLE_HPP_

#include "allocator.hpp"
#include "../memory_pool.hpp"

#include <vector>

class Arbiter;

class PreallocatedSeparableAllocator : public Allocator {
 protected:
  PreallocatedSet<int> _in_occ;
  PreallocatedSet<int> _out_occ;

  std::vector<PreallocatedMap<int, sRequest>> _in_req;
  std::vector<PreallocatedMap<int, sRequest>> _out_req;

  vector<Arbiter*> _input_arb;
  vector<Arbiter*> _output_arb;

 public:
  PreallocatedSeparableAllocator(Module* parent, const string& name, int inputs,
                                 int outputs, const string& arb_type);

  virtual ~PreallocatedSeparableAllocator();

  void Clear();

  int ReadRequest(int in, int out) const;
  bool ReadRequest(sRequest& req, int in, int out) const;

  void AddRequest(int in, int out, int label = 1, int in_pri = 0,
                  int out_pri = 0);
  void RemoveRequest(int in, int out, int label = 1);

  bool OutputHasRequests(int out) const;
  bool InputHasRequests(int in) const;

  int NumOutputRequests(int out) const;
  int NumInputRequests(int in) const;

  void PrintRequests(ostream* os = NULL) const;
};

#endif  // _PREALLOCATED_SEPARABLE_HPP_
