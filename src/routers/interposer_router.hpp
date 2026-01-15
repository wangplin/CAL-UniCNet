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

#ifndef _INTERPOSER_ROUTER_HPP_
#define _INTERPOSER_ROUTER_HPP_

#include "iq_router.hpp"

class InterposerRouter : public IQRouter {

 public:
  InterposerRouter(const Configuration& config, Module* parent,
                   const string& name, int id, int inputs, int outputs);
  ~InterposerRouter();

  void SetOutputVC(const Configuration& config,
                   const Configuration& downstream_config, int output) override;
  void ReinitVCAllocator(const Configuration& config) override;

  void SetInputPortRate();
  void SetOutputPortRate();

 protected:
  int _chiplet_flit_size;
  vector<int> _output_vcs;

  double _receive_rate;
  double _send_rate;
  vector<double> _input_receive_rates;
  vector<double> _output_send_rates;

  vector<double> _partial_input_cycles;
  vector<double> _partial_output_cycles;

  // Helper functions for encoding/decoding output+VC with variable VCs per output
  inline int _EncodeOutputVC(int output, int vc) const {
    int index = 0;
    for (int o = 0; o < output; ++o) {
      index += _output_vcs[o];
    }
    return index + vc;
  }

  inline void _DecodeOutputVC(int output_and_vc, int& output, int& vc) const {
    int index = 0;
    for (output = 0; output < _outputs; ++output) {
      if (index + _output_vcs[output] > output_and_vc) {
        vc = output_and_vc - index;
        return;
      }
      index += _output_vcs[output];
    }
    // Should not reach here
    assert(false && "Invalid output_and_vc in _DecodeOutputVC");
  }

  // Override VC allocation methods to handle variable VCs per output
  virtual void _VCAllocEvaluate() override;
  virtual void _VCAllocUpdate() override;

  virtual void _SWAllocUpdate() override;

  virtual void _SendFlits() override;
  virtual bool _ReceiveFlits() override;
};

#endif
