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

#include "preallocated_separable_input_first.hpp"

#include "arbiter.hpp"
#include "../booksim.hpp"

#include <cassert>
#include <iostream>
#include <vector>

PreallocatedSeparableInputFirstAllocator::
    PreallocatedSeparableInputFirstAllocator(Module* parent, const string& name,
                                             int inputs, int outputs,
                                             const string& arb_type)
    : PreallocatedSeparableAllocator(parent, name, inputs, outputs, arb_type) {}

void PreallocatedSeparableInputFirstAllocator::Allocate() {
  for (auto it = _in_occ.begin(); it != _in_occ.end(); ++it) {
    const int& input = *it;

    for (auto req_it = _in_req[input].begin(); req_it != _in_req[input].end();
         ++req_it) {
      auto pair = *req_it;
      const sRequest& req = pair.second;

      _input_arb[input]->AddRequest(req.port, req.label, req.in_pri);
    }

    int label = -1;
    const int output = _input_arb[input]->Arbitrate(&label, NULL);
    assert(output > -1);

    const sRequest& req = _out_req[output][input];
    assert((req.port == input) && (req.label == label));

    _output_arb[output]->AddRequest(req.port, req.label, req.out_pri);
  }

  for (auto it = _out_occ.begin(); it != _out_occ.end(); ++it) {
    const int& output = *it;

    const int input = _output_arb[output]->Arbitrate(NULL, NULL);

    if (input > -1) {
      assert((_inmatch[input] == -1) && (_outmatch[output] == -1));

      _inmatch[input] = output;
      _outmatch[output] = input;
      _input_arb[input]->UpdateState();
      _output_arb[output]->UpdateState();
    }
  }
}
