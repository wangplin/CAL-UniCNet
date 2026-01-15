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

#include "preallocated_separable.hpp"

#include "arbiter.hpp"
#include "../booksim.hpp"

#include <cassert>
#include <iostream>
#include <sstream>

PreallocatedSeparableAllocator::PreallocatedSeparableAllocator(
    Module* parent, const string& name, int inputs, int outputs,
    const string& arb_type)
    : Allocator(parent, name, inputs, outputs),
      _in_occ(inputs),
      _out_occ(outputs) {
  _in_req.resize(_inputs);
  _out_req.resize(_outputs);

  for (int i = 0; i < _inputs; ++i) {
    _in_req[i] = PreallocatedMap<int, sRequest>(outputs);
  }

  for (int j = 0; j < _outputs; ++j) {
    _out_req[j] = PreallocatedMap<int, sRequest>(inputs);
  }

  _input_arb.resize(inputs);
  for (int i = 0; i < inputs; ++i) {
    ostringstream arb_name;
    arb_name << "arb_i" << i;
    _input_arb[i] =
        Arbiter::NewArbiter(this, arb_name.str(), arb_type, outputs);
  }

  _output_arb.resize(outputs);
  for (int i = 0; i < outputs; ++i) {
    ostringstream arb_name;
    arb_name << "arb_o" << i;
    _output_arb[i] =
        Arbiter::NewArbiter(this, arb_name.str(), arb_type, inputs);
  }
}

PreallocatedSeparableAllocator::~PreallocatedSeparableAllocator() {
  for (int i = 0; i < _inputs; ++i) {
    delete _input_arb[i];
  }

  for (int i = 0; i < _outputs; ++i) {
    delete _output_arb[i];
  }
}

void PreallocatedSeparableAllocator::Clear() {
  for (auto it = _in_occ.begin(); it != _in_occ.end(); ++it) {
    int i = *it;
    if (_input_arb[i]->_num_reqs) {
      _input_arb[i]->Clear();
    }
    _in_req[i].clear();
  }

  for (auto it = _out_occ.begin(); it != _out_occ.end(); ++it) {
    int o = *it;
    if (_output_arb[o]->_num_reqs) {
      _output_arb[o]->Clear();
    }
    _out_req[o].clear();
  }

  _in_occ.clear();
  _out_occ.clear();

  Allocator::Clear();
}

int PreallocatedSeparableAllocator::ReadRequest(int in, int out) const {
  sRequest r;

  if (!ReadRequest(r, in, out)) {
    return -1;
  }

  return r.label;
}

bool PreallocatedSeparableAllocator::ReadRequest(sRequest& req, int in,
                                                 int out) const {
  assert((in >= 0) && (in < _inputs));
  assert((out >= 0) && (out < _outputs));

  bool found;

  if (_in_req[in].count(out) > 0) {
    req = const_cast<PreallocatedMap<int, sRequest>&>(_in_req[in])[out];
    found = true;
  } else {
    found = false;
  }

  return found;
}

void PreallocatedSeparableAllocator::AddRequest(int in, int out, int label,
                                                int in_pri, int out_pri) {
  Allocator::AddRequest(in, out, label, in_pri, out_pri);

  sRequest req;
  req.label = label;
  req.in_pri = in_pri;
  req.out_pri = out_pri;

  if (_in_req[in].empty()) {
    _in_occ.insert(in);
  }
  req.port = out;
  _in_req[in][out] = req;

  if (_out_req[out].empty()) {
    _out_occ.insert(out);
  }
  req.port = in;
  _out_req[out][in] = req;
}

void PreallocatedSeparableAllocator::RemoveRequest(int in, int out, int label) {
  assert((in >= 0) && (in < _inputs));
  assert((out >= 0) && (out < _outputs));

  // if (_in_req[in].count(out) > 0) {}
}

bool PreallocatedSeparableAllocator::OutputHasRequests(int out) const {
  assert((out >= 0) && (out < _outputs));
  return !_out_req[out].empty();
}

bool PreallocatedSeparableAllocator::InputHasRequests(int in) const {
  assert((in >= 0) && (in < _inputs));
  return !_in_req[in].empty();
}

int PreallocatedSeparableAllocator::NumOutputRequests(int out) const {
  assert((out >= 0) && (out < _outputs));
  return _out_req[out].size();
}

int PreallocatedSeparableAllocator::NumInputRequests(int in) const {
  assert((in >= 0) && (in < _inputs));
  return _in_req[in].size();
}

void PreallocatedSeparableAllocator::PrintRequests(ostream* os) const {
  ostream* myos = os ? os : &cout;

  *myos << "Input requests:" << endl;
  for (int i = 0; i < _inputs; ++i) {
    if (!_in_req[i].empty()) {
      *myos << "  Input " << i << ": ";
      for (auto it = _in_req[i].begin(); it != _in_req[i].end(); ++it) {
        auto pair = *it;
        *myos << "(" << pair.first << "," << pair.second.label << ") ";
      }
      *myos << endl;
    }
  }

  *myos << "Output requests:" << endl;
  for (int j = 0; j < _outputs; ++j) {
    if (!_out_req[j].empty()) {
      *myos << "  Output " << j << ": ";
      for (auto it = _out_req[j].begin(); it != _out_req[j].end(); ++it) {
        auto pair = *it;
        *myos << "(" << pair.first << "," << pair.second.label << ") ";
      }
      *myos << endl;
    }
  }
}
