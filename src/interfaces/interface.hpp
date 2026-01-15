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

#ifndef _INTERFACE_HPP_
#define _INTERFACE_HPP_

#include "config_utils.hpp"
#include "credit.hpp"
#include "flit.hpp"
#include "flitchannel.hpp"
#include "creditchannel.hpp"

#include <queue>

class Interface : public TimedModule {

 protected:
  int _id;
  int _vcs;

  int _inputs;
  int _outputs;

  int _credit_delay;

  bool _active;

  vector<FlitChannel*> _input_channels;
  vector<CreditChannel*> _input_credits;
  vector<FlitChannel*> _output_channels;
  vector<CreditChannel*> _output_credits;

  vector<int> _input_receive_rate;
  vector<int> _output_send_rate;

  map<int, deque<Flit*>> _in_queue_flits;
  map<int, Credit*> _out_queue_credits;

  vector<queue<Flit*>> _output_buffer;
  vector<queue<Credit*>> _credit_buffer;

  deque<pair<int, pair<Credit*, int>>> _proc_credits_router;
  deque<pair<int, pair<Credit*, int>>> _proc_credirs_interface;

  virtual bool _ReceiveFlits();
  virtual bool _ReceiveCredits();

  virtual void _InternalStep();

  virtual void _InputQueuing();
  virtual void _OutputQueuing();

  virtual void _SendFlits();
  virtual void _SendCredits();

 public:
  Interface(const Configuration& config, Module* parent, const string& name,
            int id);
  static Interface* NewInterface(const Configuration& config, Module* parent,
                                 const string& name, int id);

  virtual void AddInputChannel(FlitChannel* channel,
                               CreditChannel* backchannel);
  virtual void AddOutputChannel(FlitChannel* channel,
                                CreditChannel* backchannel);

  virtual void SetFarstreamRouter(const Configuration& config,
                                  Router* farstream_router) {}
  virtual void SetNearstreamConfig(Configuration* nearstream_config) {}

  inline FlitChannel* GetInputChannel(int input) const {
    assert((input >= 0) && (input < _inputs));
    return _input_channels[input];
  }

  inline FlitChannel* GetOutputChannel(int output) const {
    assert((output >= 0) && (output < _outputs));
    return _output_channels[output];
  }

  void ReadInputs();
  void Evaluate();
  void WriteOutputs();

  inline int GetID() const { return _id; }
};

#endif