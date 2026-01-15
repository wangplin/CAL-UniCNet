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

#include "interface.hpp"

#include "ucieinterface.hpp"

Interface::Interface(const Configuration& config, Module* parent,
                     const string& name, int id)
    : TimedModule(parent, name),
      _id(id),
      _inputs(2),
      _outputs(2),
      _active(false) {

  _credit_delay = config.GetInt("credit_delay");

  _credit_buffer.resize(_inputs);
  _output_buffer.resize(_outputs);

  _input_receive_rate.resize(_inputs, 1);
  _output_send_rate.resize(_outputs, 1);
}

Interface* Interface::NewInterface(const Configuration& config, Module* parent,
                                   const string& name, int id) {
  string type = config.GetStr("interface");
  Interface* i = nullptr;
  if (type == "ucie") {
    i = new UCIEInterface(config, parent, name, id);
  } else {
    cerr << "Unknown interface type: " << type << endl;
  }
  return i;
}

void Interface::AddInputChannel(FlitChannel* channel,
                                CreditChannel* backchannel) {
  _input_channels.push_back(channel);
  _input_credits.push_back(backchannel);
  channel->SetInterfaceSink(this, _input_channels.size() - 1);
  backchannel->SetInterfaceSource(this, _input_credits.size() - 1);
}

void Interface::AddOutputChannel(FlitChannel* channel,
                                 CreditChannel* backchannel) {
  _output_channels.push_back(channel);
  _output_credits.push_back(backchannel);
  channel->SetInterfaceSource(this, _output_channels.size() - 1);
  backchannel->SetInterfaceSink(this, _output_credits.size() - 1);
}

bool Interface::_ReceiveFlits() {
  bool activity = false;
  for (int input = 0; input < _inputs; ++input) {
    Flit* const f = _input_channels[input]->Receive();
    if (f) {
#ifdef TRACK_FLOWS
      ++_received_flits[f->cl][input];
#endif

      if (f->watch) {
        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                   << "Received flit " << f->id << " from channel at input "
                   << input << "." << endl;
      }
      _in_queue_flits[input].push_back(f);
      activity = true;
    }
  }
  return activity;
}

bool Interface::_ReceiveCredits() {
  assert(false);
}

void Interface::Evaluate() {
  _InternalStep();
}

void Interface::_InputQueuing() {
  assert(false);
}

void Interface::_OutputQueuing() {
  assert(false);
}

void Interface::_InternalStep() {
  assert(false);
}

void Interface::ReadInputs() {
  bool have_flits = _ReceiveFlits();
  bool have_credits = _ReceiveCredits();
  _active = _active || have_flits || have_credits;
}

void Interface::_SendFlits() {
  for (int output = 0; output < _outputs; ++output) {
    if (!_output_buffer[output].empty()) {
      Flit* const f = _output_buffer[output].front();
      assert(f);
      _output_buffer[output].pop();

#ifdef TRACK_FLOWS
      ++_sent_flits[f->cl][output];
#endif

      if (f->watch)
        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                   << "Sending flit " << f->id << " to channel at output "
                   << output << "." << endl;
      if (gTrace) {
        cout << "Outport " << output << endl << "Stop Mark" << endl;
      }
      _output_channels[output]->Send(f);
    }
  }
}

void Interface::_SendCredits() {
  for (int input = 0; input < _inputs; ++input) {
    if (!_credit_buffer[input].empty()) {
      Credit* const c = _credit_buffer[input].front();
      assert(c);
      _credit_buffer[input].pop();
      _input_credits[input]->Send(c);
    }
  }
}

void Interface::WriteOutputs() {
  _SendFlits();
  _SendCredits();
}