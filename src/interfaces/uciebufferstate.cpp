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

#include "uciebufferstate.hpp"
#include "buffer_state.hpp"

#include <algorithm>
#include <numeric>

UCIEBufferState::UCIEBufferState(const Configuration& config, Module* parent,
                                 const string& name, int num_vcs,
                                 int wait_for_tail_credit)
    : BufferState(config, parent, name, wait_for_tail_credit) {
  _vcs = num_vcs;
  _size = _vcs * config.GetInt("ucie_" + name + "_depth");

  _vc_occupancy.resize(_vcs, 0);
  _in_use_by.resize(_vcs, -1);
  _tail_sent.resize(_vcs, false);

  _last_id.resize(_vcs, -1);
  _last_pid.resize(_vcs, -1);

  _available_vcs.resize(_vcs);
  std::iota(_available_vcs.begin(), _available_vcs.end(), 0);
  _available_present.assign(_vcs, 1);
}

UCIEBufferState::~UCIEBufferState() {}

void UCIEBufferState::SendingUCIeFlit(Flit const* const f) {
  ++_occupancy;
  if (_occupancy > _size) {
    Error("Buffer overflow.");
  }

  ++_vc_occupancy[0];

  if (f->tail) {
    _tail_sent[0] = true;
    _in_use_by[0] = -1;
  }
  _last_id[0] = f->id;
  _last_pid[0] = f->pid;
}