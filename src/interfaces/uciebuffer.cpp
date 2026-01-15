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

#include "uciebuffer.hpp"
#include "config_utils.hpp"
#include "vc.hpp"
#include "booksim.hpp"

#include <sstream>

UCIEBuffer::UCIEBuffer(const Configuration& config, int outputs, Module* parent,
                       const string& name, int num_vcs)
    : Buffer(config, outputs, parent, name) {
  _is_packet_in_progress = false;
  assert(num_vcs > 0);

  string buffer_name = "ucie_" + name + "_depth";
  _size = num_vcs * config.GetInt(buffer_name);

  // delete the old vcs
  for (VC* vc : _vc) {
    delete vc;
  }

  _vc.resize(num_vcs);

  for (int i = 0; i < num_vcs; ++i) {
    ostringstream vc_name;
    vc_name << "vc_" << i;
    _vc[i] = new VC(config, outputs, this, vc_name.str());
  }
}

UCIEBuffer::~UCIEBuffer() {}

void UCIEBuffer::SetState(int vc, VC::eVCState s) {
  VC::eVCState old_state = _vc[vc]->GetState();
  Buffer::SetState(vc, s);
  if (s == VC::active) {
    assert(!_is_packet_in_progress);
    _is_packet_in_progress = true;
  } else if (old_state == VC::active && (s == VC::idle || s == VC::vc_alloc)) {
    assert(_is_packet_in_progress);
    _is_packet_in_progress = false;
  }
}

void UCIEBuffer::AddUCIeFlit(Flit* f) {
  if (_occupancy >= _size) {
    Error("Flit buffer overflow.");
  }
  _vc[0]->AddUCIeFlit(f);
  _occupancy++;
}

Flit* UCIEBuffer::RemoveUCIeFlit() {
  return Buffer::RemoveFlit(0);
}
