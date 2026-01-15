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

#ifndef _UCIE_BUFFER_HPP_
#define _UCIE_BUFFER_HPP_

#include "buffer.hpp"

class UCIEBuffer : public Buffer {

 public:
  UCIEBuffer(const Configuration& config, int outputs, Module* parent,
             const string& name, int num_vcs);
  ~UCIEBuffer();

  void SetState(int vc, VC::eVCState s) override;

  inline bool IsPacketInProgress() const { return _is_packet_in_progress; }

  inline int GetSizeSent(int vc) const { return _vc[vc]->GetSizeSent(); }

  inline void SetSizeSent(int vc, int sizeSent) {
    _vc[vc]->SetSizeSent(sizeSent);
  }

  inline int GetFidSend(int vc) const { return _vc[vc]->GetFidSend(); }

  inline void ResetFidSend(int vc) { _vc[vc]->ResetFidSend(); }

  inline void UpdateFidSend(int vc) { _vc[vc]->UpdateFidSend(); }

  virtual void AddUCIeFlit(Flit* f);
  virtual Flit* RemoveUCIeFlit();

 protected:
  bool _is_packet_in_progress;
};

#endif