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

#include "ucieretrybuffer.hpp"

UCIERetryBuffer::UCIERetryBuffer(const Configuration& config, int outputs,
                                 Module* parent, const string& name,
                                 int num_vcs)
    : UCIEBuffer(config, outputs, parent, name, num_vcs) {

  _size = min(config.GetInt("ucie_retry_buffer_depth"), 127);
  _wrptr = 1;
  _rdptr = 1;
  _seq_num = 0;
  _replay_state = ReplayState::idle;
}

UCIERetryBuffer::~UCIERetryBuffer() {}

void UCIERetryBuffer::AddUCIeFlit(Flit* f) {
  assert(_replay_state == ReplayState::idle);
  if (_occupancy >= _size) {
    Error("UCIERetryBuffer overflow.");
  }

  assert(f->seq_num == this->_seq_num);
  this->_seq_num++;

  _vc[0]->AddUCIeFlit(f);
  ++_occupancy;
  _wrptr++;
}

Flit* UCIERetryBuffer::RemoveUCIeFlit() {
  assert(_replay_state == ReplayState::idle);
  Flit* f = Buffer::RemoveFlit(0);
  _wrptr--;
  assert(_wrptr >= 1);
  return f;
}

bool UCIERetryBuffer::ClearFlitForNak(int seq_num) {
  assert(_replay_state == ReplayState::idle);
  _replay_state = ReplayState::replay;
  Flit* f = Buffer::FrontFlit(0);
  return (f->seq_num == seq_num + 1);
}

Flit* UCIERetryBuffer::ReplayFlitForNak() {
  assert(_replay_state == ReplayState::replay);
  while (_rdptr < _wrptr) {
    Flit* f = Buffer::FrontNumberFlit(0, _rdptr);
    _rdptr++;
    if (_rdptr == _wrptr) {
      _replay_state = ReplayState::idle;
      _rdptr = 1;
    }
    f->is_retry = true;
    return f;
  }
  return nullptr;
}