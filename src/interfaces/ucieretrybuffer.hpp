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

#ifndef _UCIE_RETRY_BUFFER_HPP_
#define _UCIE_RETRY_BUFFER_HPP_

#include "uciebuffer.hpp"

class UCIERetryBuffer : public UCIEBuffer {
 public:
  UCIERetryBuffer(const Configuration& config, int outputs, Module* parent,
                  const string& name, int num_vcs);
  ~UCIERetryBuffer();

  void AddUCIeFlit(Flit* f) override;
  Flit* RemoveUCIeFlit() override;

  bool ClearFlitForNak(int seq_num);
  Flit* ReplayFlitForNak();

  enum ReplayState {
    state_min = 0,
    idle = state_min,
    replay,
    state_max = replay
  };
  ReplayState _replay_state;

  inline ReplayState GetReplayState() const { return _replay_state; }

 protected:
  int _wrptr;
  int _rdptr;
  int _seq_num;
};

#endif