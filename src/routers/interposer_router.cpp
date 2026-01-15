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

#include "interposer_router.hpp"

#include "allocator.hpp"
#include "buffer.hpp"
#include "buffer_monitor.hpp"
#include "buffer_state.hpp"
#include "uciebufferstate.hpp"
#include "globals.hpp"
#include "outputset.hpp"
#include "random_utils.hpp"
#include "roundrobin_arb.hpp"
#include "routefunc.hpp"
#include "switch_monitor.hpp"
#include "vc.hpp"
#include "chiplet_network.hpp"

#include <cassert>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>

InterposerRouter::InterposerRouter(const Configuration& config, Module* parent,
                                   const string& name, int id, int inputs,
                                   int outputs)
    : IQRouter(config, parent, name, id, inputs, outputs) {

  _chiplet_id = Chiplet_Network::router_to_chiplet[id];
  _chiplet_flit_size = config.GetInt("chiplet_flit_size");
  _output_vcs.resize(_outputs, _vcs);

  _send_rate = config.GetFloat("send_rate");
  _receive_rate = config.GetFloat("receive_rate");
  _input_receive_rates.resize(_inputs, 1.0);
  _output_send_rates.resize(_outputs, 1.0);
  _partial_input_cycles.resize(_inputs, 0.0);
  _partial_output_cycles.resize(_outputs, 0.0);
}

InterposerRouter::~InterposerRouter() {}

void InterposerRouter::SetOutputVC(const Configuration& config,
                                   const Configuration& downstream_config,
                                   int output) {
  assert(output >= 0 && output < _outputs);

  // Recreate BufferState with downstream router or inerface's configuration

  // the wait_for_tail_credit is the same as the original buffer state
  int original_wait_for_tail_credit =
      _next_buf[output]->WaitForTailCredit() ? 1 : 0;
  delete _next_buf[output];

  ostringstream module_name;

  if (config.GetStr("interface") == "ucie") {
    module_name << "tx_fifo";
    // tracking the _tx_fifo, num_vcs is the same as the downstream router, the buffer depth is ucie_tx_fifo_depth
    _next_buf[output] = new UCIEBufferState(config, this, module_name.str(),
                                            downstream_config.GetInt("num_vcs"),
                                            original_wait_for_tail_credit);
  } else {
    module_name << "next_vc_o" << output;
    _next_buf[output] =
        new BufferState(downstream_config, this, module_name.str(),
                        original_wait_for_tail_credit);
  }

  assert((int)_next_buf.size() == _outputs);

  // Update min_latency for the new BufferState
  FlitChannel* channel = _output_channels[output];
  CreditChannel* backchannel = _output_credits[output];
  int alloc_delay = _speculative ? max(_vc_alloc_delay, _sw_alloc_delay)
                                 : (_vc_alloc_delay + _sw_alloc_delay);
  int min_latency = 1 + _crossbar_delay + channel->GetLatency() +
                    _routing_delay + alloc_delay + backchannel->GetLatency() +
                    _credit_delay;
  _next_buf[output]->SetMinLatency(min_latency);

  _output_vcs[output] = config.GetInt("num_vcs");
  assert((int)_output_vcs.size() == _outputs);
}

// Reinitialize VC allocator after all SetOutputVC calls are done
void InterposerRouter::ReinitVCAllocator(const Configuration& config) {
  string vc_alloc_type = config.GetStr("vc_allocator");

  // Delete old allocator
  if (_vc_allocator) {
    delete _vc_allocator;
  }

  // Calculate total output VCs
  int outputs_sum = accumulate(_output_vcs.begin(), _output_vcs.end(), 0);

  // Create new allocator with correct output VC count
  _vc_allocator = Allocator::NewAllocator(this, "vc_allocator", vc_alloc_type,
                                          _vcs * _inputs, outputs_sum, &config);
  if (!_vc_allocator) {
    Error("Unknown vc_allocator type: " + vc_alloc_type);
  }
}

void InterposerRouter::_VCAllocEvaluate() {
  assert(_vc_allocator);

  int const current_time = GetSimTime();
  bool watched = false;

  auto iter_end = _vc_alloc_vcs.end();
  for (auto iter = _vc_alloc_vcs.begin(); iter != iter_end; ++iter) {

    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first.first;
    assert((input >= 0) && (input < _inputs));
    int const vc = iter->second.first.second;
    assert((vc >= 0) && (vc < _vcs));

    assert(iter->second.second == -1);

    Buffer const* const cur_buf = _buf[input];
    assert(!cur_buf->Empty(vc));
    assert(cur_buf->GetState(vc) == VC::vc_alloc);

    Flit const* const f = cur_buf->FrontFlit(vc);
    assert(f);
    assert(f->vc == vc);
    assert(f->head);

    if (f->watch) {
      *gWatchOut << current_time << " | " << FullName() << " | "
                 << "Beginning VC allocation for VC " << vc << " at input "
                 << input << " (front: " << f->id << ")." << endl;
    }

    OutputSet const* const route_set = cur_buf->GetRouteSet(vc);
    assert(route_set);

    int const out_priority = cur_buf->GetPriority(vc);
    const auto& setlist = route_set->GetSet();

    bool elig = false;
    bool cred = false;
    bool reserved = false;

    assert(!_noq || (setlist.size() == 1));

    for (const auto& iset : setlist) {

      int const out_port = iset.output_port;
      assert((out_port >= 0) && (out_port < _outputs));

      BufferState const* const dest_buf = _next_buf[out_port];

      int vc_start;
      int vc_end;

      if (_noq && _noq_next_output_port[input][vc] >= 0) {
        assert(!_routing_delay);
        vc_start = _noq_next_vc_start[input][vc];
        vc_end = _noq_next_vc_end[input][vc];
      } else {
        vc_start = iset.vc_start;
        vc_end = iset.vc_end;
      }

      // Clamp to the actual VCs available at this output
      int output_vcs = _output_vcs[out_port];
      assert(vc_start >= 0 && vc_start < output_vcs);
      if (vc_end >= output_vcs) {
        vc_end = output_vcs - 1;
      }
      assert(vc_end >= 0 && vc_end < output_vcs);
      assert(vc_end >= vc_start);

      if (f->watch) {
        for (int out_vc = vc_start; out_vc <= vc_end; ++out_vc) {
          assert((out_vc >= 0) && (out_vc < output_vcs));

          int in_priority = iset.pri;
          if (_vc_prioritize_empty && !dest_buf->IsEmptyFor(out_vc)) {
            assert(in_priority >= 0);
            in_priority += numeric_limits<int>::min();
          }

          if (!dest_buf->IsAvailableFor(out_vc)) {
            int const use_input_and_vc = dest_buf->UsedBy(out_vc);
            int const use_input = use_input_and_vc / _vcs;
            int const use_vc = use_input_and_vc % _vcs;
            *gWatchOut << current_time << " | " << FullName() << " | "
                       << "  VC " << out_vc << " at output " << out_port
                       << " is in use by VC " << use_vc << " at input "
                       << use_input;
            Flit* cf = _buf[use_input]->FrontFlit(use_vc);
            if (cf) {
              *gWatchOut << " (front flit: " << cf->id << ")";
            } else {
              *gWatchOut << " (empty)";
            }
            *gWatchOut << "." << endl;
          } else {
            elig = true;
            if (_vc_busy_when_full && dest_buf->IsFullFor(out_vc)) {
              *gWatchOut << current_time << " | " << FullName() << " | "
                         << "  VC " << out_vc << " at output " << out_port
                         << " is full." << endl;
              reserved |= !dest_buf->IsFull();
            } else {
              cred = true;
              *gWatchOut << current_time << " | " << FullName() << " | "
                         << "  Requesting VC " << out_vc << " at output "
                         << out_port << " (in_pri: " << in_priority
                         << ", out_pri: " << out_priority << ")." << endl;
              watched = true;
              int const input_and_vc = _vc_shuffle_requests
                                           ? (vc * _inputs + input)
                                           : (input * _vcs + vc);
              // Use custom encoding for output+VC
              _vc_allocator->AddRequest(input_and_vc,
                                        _EncodeOutputVC(out_port, out_vc), 0,
                                        in_priority, out_priority);
            }
          }
        }
      } else {
        const auto& available_vcs = dest_buf->AvailableVCs();
        auto lower = std::lower_bound(available_vcs.begin(),
                                      available_vcs.end(), vc_start);
        for (auto it = lower; it != available_vcs.end(); ++it) {
          int out_vc = *it;
          if (out_vc > vc_end) {
            break;
          }

          assert((out_vc >= 0) && (out_vc < output_vcs));

          int in_priority = iset.pri;
          if (_vc_prioritize_empty && !dest_buf->IsEmptyFor(out_vc)) {
            assert(in_priority >= 0);
            in_priority += numeric_limits<int>::min();
          }

          elig = true;
          if (_vc_busy_when_full && dest_buf->IsFullFor(out_vc)) {
            reserved |= !dest_buf->IsFull();
            continue;
          }

          cred = true;
          int const input_and_vc = _vc_shuffle_requests ? (vc * _inputs + input)
                                                        : (input * _vcs + vc);
          // Use custom encoding for output+VC
          _vc_allocator->AddRequest(input_and_vc,
                                    _EncodeOutputVC(out_port, out_vc), 0,
                                    in_priority, out_priority);
        }
      }
    }
    if (!elig) {
      iter->second.second = STALL_BUFFER_BUSY;
    } else if (_vc_busy_when_full && !cred) {
      iter->second.second =
          reserved ? STALL_BUFFER_RESERVED : STALL_BUFFER_FULL;
    }
  }

  if (watched) {
    *gWatchOut << current_time << " | " << _vc_allocator->FullName() << " | ";
    _vc_allocator->PrintRequests(gWatchOut);
  }

  _vc_allocator->Allocate();

  if (watched) {
    *gWatchOut << current_time << " | " << _vc_allocator->FullName() << " | ";
    _vc_allocator->PrintGrants(gWatchOut);
  }

  auto iter_end2 = _vc_alloc_vcs.end();
  for (auto iter = _vc_alloc_vcs.begin(); iter != iter_end2; ++iter) {

    int const time = iter->first;
    if (time >= 0) {
      break;
    }
    iter->first = current_time + _vc_alloc_delay - 1;

    int const input = iter->second.first.first;
    assert((input >= 0) && (input < _inputs));
    int const vc = iter->second.first.second;
    assert((vc >= 0) && (vc < _vcs));

    if (iter->second.second < -1) {
      continue;
    }

    assert(iter->second.second == -1);

    Buffer const* const cur_buf = _buf[input];
    assert(!cur_buf->Empty(vc));
    assert(cur_buf->GetState(vc) == VC::vc_alloc);

    Flit const* const f = cur_buf->FrontFlit(vc);
    assert(f);
    assert(f->vc == vc);
    assert(f->head);

    int const input_and_vc =
        _vc_shuffle_requests ? (vc * _inputs + input) : (input * _vcs + vc);
    int const output_and_vc = _vc_allocator->OutputAssigned(input_and_vc);

    if (output_and_vc >= 0) {

      // Use custom decoding for output+VC
      int match_output, match_vc;
      _DecodeOutputVC(output_and_vc, match_output, match_vc);
      assert((match_output >= 0) && (match_output < _outputs));
      assert((match_vc >= 0) && (match_vc < _output_vcs[match_output]));

      if (f->watch) {
        *gWatchOut << current_time << " | " << FullName() << " | "
                   << "Assigning VC " << match_vc << " at output "
                   << match_output << " to VC " << vc << " at input " << input
                   << "." << endl;
      }

      iter->second.second = output_and_vc;

    } else {

      iter->second.second = STALL_BUFFER_CONFLICT;
    }
  }
}

void InterposerRouter::_VCAllocUpdate() {
  assert(_vc_allocator);

  int const current_time = GetSimTime();

  while (!_vc_alloc_vcs.empty()) {

    pair<int, pair<pair<int, int>, int>> const& item = _vc_alloc_vcs.front();

    int const time = item.first;
    if ((time < 0) || (current_time < time)) {
      break;
    }
    assert(current_time == time);

    int const input = item.second.first.first;
    assert((input >= 0) && (input < _inputs));
    int const vc = item.second.first.second;
    assert((vc >= 0) && (vc < _vcs));

    assert(item.second.second != -1);

    Buffer* const cur_buf = _buf[input];
    assert(!cur_buf->Empty(vc));
    assert(cur_buf->GetState(vc) == VC::vc_alloc);

    Flit const* const f = cur_buf->FrontFlit(vc);
    assert(f);
    assert(f->vc == vc);
    assert(f->head);

    if (f->watch) {
      *gWatchOut << current_time << " | " << FullName() << " | "
                 << "Completed VC allocation for VC " << vc << " at input "
                 << input << " (front: " << f->id << ")." << endl;
    }

    int const output_and_vc = item.second.second;

    if (output_and_vc >= 0) {

      // Use custom decoding for output+VC
      int match_output, match_vc;
      _DecodeOutputVC(output_and_vc, match_output, match_vc);
      assert((match_output >= 0) && (match_output < _outputs));
      assert((match_vc >= 0) && (match_vc < _output_vcs[match_output]));

      if (f->watch) {
        *gWatchOut << current_time << " | " << FullName() << " | "
                   << "  Acquiring assigned VC " << match_vc << " at output "
                   << match_output << "." << endl;
      }

      BufferState* const dest_buf = _next_buf[match_output];
      assert(dest_buf->IsAvailableFor(match_vc));

      dest_buf->TakeBuffer(match_vc, input * _vcs + vc);

      cur_buf->SetOutput(vc, match_output, match_vc);
      cur_buf->SetState(vc, VC::active);
      if (!_speculative) {
        _sw_alloc_vcs.push_back(
            make_pair(-1, make_pair(item.second.first, -1)));
      }
    } else {
      if (f->watch) {
        *gWatchOut << current_time << " | " << FullName() << " | "
                   << "  No output VC allocated." << endl;
      }

#ifdef TRACK_STALLS
      assert((output_and_vc == STALL_BUFFER_BUSY) ||
             (output_and_vc == STALL_BUFFER_CONFLICT));
      if (output_and_vc == STALL_BUFFER_BUSY) {
        ++_buffer_busy_stalls[f->cl];
      } else if (output_and_vc == STALL_BUFFER_CONFLICT) {
        ++_buffer_conflict_stalls[f->cl];
      }
#endif

      _vc_alloc_vcs.push_back(make_pair(-1, make_pair(item.second.first, -1)));
    }
    _pop_front(_vc_alloc_vcs);
  }
}

void InterposerRouter::_SWAllocUpdate() {
  while (!_sw_alloc_vcs.empty()) {

    pair<int, pair<pair<int, int>, int>> const& item = _sw_alloc_vcs.front();

    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const input = item.second.first.first;
    assert((input >= 0) && (input < _inputs));
    int const vc = item.second.first.second;
    assert((vc >= 0) && (vc < _vcs));

    Buffer* const cur_buf = _buf[input];
    assert(!cur_buf->Empty(vc));
    assert((cur_buf->GetState(vc) == VC::active) ||
           (_speculative && (cur_buf->GetState(vc) == VC::vc_alloc)));

    Flit* const f = cur_buf->FrontFlit(vc);
    assert(f);
    assert(f->vc == vc);

    if (f->watch) {
      *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                 << "Completed switch allocation for VC " << vc << " at input "
                 << input << " (front: " << f->id << ")." << endl;
    }

    int const expanded_output = item.second.second;

    if (expanded_output >= 0) {

      int const expanded_input = input * _input_speedup + vc % _input_speedup;
      assert(_switch_hold_vc[expanded_input] < 0);
      assert(_switch_hold_in[expanded_input] < 0);
      assert(_switch_hold_out[expanded_output] < 0);

      int const output = expanded_output / _output_speedup;
      assert((output >= 0) && (output < _outputs));

      BufferState* const dest_buf = _next_buf[output];

      int match_vc;

      if (!_vc_allocator && (cur_buf->GetState(vc) == VC::vc_alloc)) {

        assert(f->head);

        int const cl = f->cl;
        assert((cl >= 0) && (cl < _classes));

        int const vc_offset = _vc_rr_offset[output * _classes + cl];

        match_vc = -1;
        int match_prio = numeric_limits<int>::min();

        const OutputSet* route_set = cur_buf->GetRouteSet(vc);
        std::vector<OutputSet::sSetElement> const& setlist =
            route_set->GetSet();

        assert(!_noq || (setlist.size() == 1));

        for (std::vector<OutputSet::sSetElement>::const_iterator iset =
                 setlist.begin();
             iset != setlist.end(); ++iset) {
          if (iset->output_port == output) {

            int vc_start;
            int vc_end;

            if (_noq && _noq_next_output_port[input][vc] >= 0) {
              assert(!_routing_delay);
              vc_start = _noq_next_vc_start[input][vc];
              vc_end = _noq_next_vc_end[input][vc];
            } else {
              vc_start = iset->vc_start;
              vc_end = iset->vc_end;
            }
            assert(vc_start >= 0 && vc_start < _vcs);
            assert(vc_end >= 0 && vc_end < _vcs);
            assert(vc_end >= vc_start);

            for (int out_vc = vc_start; out_vc <= vc_end; ++out_vc) {
              assert((out_vc >= 0) && (out_vc < _vcs));

              int vc_prio = iset->pri;
              if (_vc_prioritize_empty && !dest_buf->IsEmptyFor(out_vc)) {
                assert(vc_prio >= 0);
                vc_prio += numeric_limits<int>::min();
              }

              // FIXME: This check should probably be performed in Evaluate(),
              // not Update(), as the latter can cause the outcome to depend on
              // the order of evaluation!
              if (dest_buf->IsAvailableFor(out_vc) &&
                  !dest_buf->IsFullFor(out_vc) &&
                  ((match_vc < 0) || RoundRobinArbiter::Supersedes(
                                         out_vc, vc_prio, match_vc, match_prio,
                                         vc_offset, _vcs))) {
                match_vc = out_vc;
                match_prio = vc_prio;
              }
            }
          }
        }
        assert(match_vc >= 0);

        if (f->watch) {
          *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                     << "  Allocating VC " << match_vc << " at output "
                     << output << " via piggyback VC allocation." << endl;
        }

        cur_buf->SetState(vc, VC::active);
        cur_buf->SetOutput(vc, output, match_vc);
        dest_buf->TakeBuffer(match_vc, input * _vcs + vc);

        _vc_rr_offset[output * _classes + cl] = (match_vc + 1) % _vcs;

      } else {

        assert(cur_buf->GetOutputPort(vc) == output);

        match_vc = cur_buf->GetOutputVC(vc);
      }
      assert((match_vc >= 0) && (match_vc < _vcs));

      if (f->watch) {
        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                   << "  Scheduling switch connection from input " << input
                   << "." << (vc % _input_speedup) << " to output " << output
                   << "." << (expanded_output % _output_speedup) << "." << endl;
      }

      cur_buf->RemoveFlit(vc);

#ifdef TRACK_FLOWS
      --_stored_flits[f->cl][input];
      if (f->tail)
        --_active_packets[f->cl][input];
#endif

      _bufferMonitor->read(input, f);

      f->hops++;
      f->vc = match_vc;

      if (!_routing_delay && f->head) {
        const FlitChannel* channel = _output_channels[output];
        const Router* router = channel->GetSink();
        if (router == nullptr) {
          // used for interposer router to get the remote router sink
          router = channel->GetRemoteRouterSink();
        }
        if (router) {
          if (_noq) {
            if (f->watch) {
              *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                         << "Updating lookahead routing information for flit "
                         << f->id << " (NOQ)." << endl;
            }
            int next_output_port = _noq_next_output_port[input][vc];
            assert(next_output_port >= 0);
            _noq_next_output_port[input][vc] = -1;
            int next_vc_start = _noq_next_vc_start[input][vc];
            assert(next_vc_start >= 0 && next_vc_start < _vcs);
            _noq_next_vc_start[input][vc] = -1;
            int next_vc_end = _noq_next_vc_end[input][vc];
            assert(next_vc_end >= 0 && next_vc_end < _vcs);
            _noq_next_vc_end[input][vc] = -1;
            f->la_route_set.Clear();
            f->la_route_set.AddRange(next_output_port, next_vc_start,
                                     next_vc_end);
          } else {
            if (f->watch) {
              *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                         << "Updating lookahead routing information for flit "
                         << f->id << "." << endl;
            }
            int in_channel = channel->GetSinkPort();
            _rf(router, f, in_channel, &f->la_route_set, false);
          }
        } else {
          f->la_route_set.Clear();
        }
      }

#ifdef TRACK_FLOWS
      ++_outstanding_credits[f->cl][output];
      _outstanding_classes[output][f->vc].push(f->cl);
#endif

      dest_buf->SendingFlit(f);

      _crossbar_flits.push_back(make_pair(
          -1, make_pair(f, make_pair(expanded_input, expanded_output))));

      if (_out_queue_credits.count(input) == 0) {
        _out_queue_credits.insert(make_pair(input, Credit::New()));
      }
      _out_queue_credits.find(input)->second->vc.insert(vc);

      if (cur_buf->Empty(vc)) {
        if (f->tail) {
          cur_buf->SetState(vc, VC::idle);
        }
      } else {
        Flit* const nf = cur_buf->FrontFlit(vc);
        assert(nf);
        assert(nf->vc == vc);
        if (f->tail) {
          assert(nf->head);
          if (_routing_delay) {
            cur_buf->SetState(vc, VC::routing);
            _route_vcs.push_back(make_pair(-1, item.second.first));
          } else {
            if (nf->watch) {
              *gWatchOut
                  << GetSimTime() << " | " << FullName() << " | "
                  << "Using precomputed lookahead routing information for VC "
                  << vc << " at input " << input << " (front: " << nf->id
                  << ")." << endl;
            }
            cur_buf->SetRouteSet(vc, &nf->la_route_set);
            cur_buf->SetState(vc, VC::vc_alloc);
            if (_speculative) {
              _sw_alloc_vcs.push_back(
                  make_pair(-1, make_pair(item.second.first, -1)));
            }
            if (_vc_allocator) {
              _vc_alloc_vcs.push_back(
                  make_pair(-1, make_pair(item.second.first, -1)));
            }
            if (_noq) {
              _UpdateNOQ(input, vc, nf);
            }
          }
        } else {
          if (_hold_switch_for_packet) {
            if (f->watch) {
              *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                         << "Setting up switch hold for VC " << vc
                         << " at input " << input << "."
                         << (expanded_input % _input_speedup) << " to output "
                         << output << "." << (expanded_output % _output_speedup)
                         << "." << endl;
            }
            _switch_hold_vc[expanded_input] = vc;
            _switch_hold_in[expanded_input] = expanded_output;
            _switch_hold_out[expanded_output] = expanded_input;
            _sw_hold_vcs.push_back(
                make_pair(-1, make_pair(item.second.first, -1)));
          } else {
            _sw_alloc_vcs.push_back(
                make_pair(-1, make_pair(item.second.first, -1)));
          }
        }
      }
    } else {
      if (f->watch) {
        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                   << "  No output port allocated." << endl;
      }

#ifdef TRACK_STALLS
      assert((expanded_output ==
              -1) ||  // for stalls that are accounted for in VC allocation path
             (expanded_output == STALL_BUFFER_BUSY) ||
             (expanded_output == STALL_BUFFER_CONFLICT) ||
             (expanded_output == STALL_BUFFER_FULL) ||
             (expanded_output == STALL_BUFFER_RESERVED) ||
             (expanded_output == STALL_CROSSBAR_CONFLICT));
      if (expanded_output == STALL_BUFFER_BUSY) {
        ++_buffer_busy_stalls[f->cl];
      } else if (expanded_output == STALL_BUFFER_CONFLICT) {
        ++_buffer_conflict_stalls[f->cl];
      } else if (expanded_output == STALL_BUFFER_FULL) {
        ++_buffer_full_stalls[f->cl];
      } else if (expanded_output == STALL_BUFFER_RESERVED) {
        ++_buffer_reserved_stalls[f->cl];
      } else if (expanded_output == STALL_CROSSBAR_CONFLICT) {
        ++_crossbar_conflict_stalls[f->cl];
      }
#endif

      _sw_alloc_vcs.push_back(make_pair(-1, make_pair(item.second.first, -1)));
    }
    _pop_front(_sw_alloc_vcs);
  }
}

void InterposerRouter::_SendFlits() {
  for (int output = 0; output < _outputs; ++output) {
    double send_rate = _output_send_rates[output];

    if (!_output_buffer[output].empty()) {
      _partial_output_cycles[output] += send_rate;
      while (_partial_output_cycles[output] >= 1.0 &&
             !_output_buffer[output].empty()) {
        Flit* const f = _output_buffer[output].front();
        assert(f);
        _output_buffer[output].pop();
        _output_channels[output]->Send(f);
        _partial_output_cycles[output] -= 1.0;
      }
    }
  }
}

bool InterposerRouter::_ReceiveFlits() {
  bool activity = false;

  for (int input = 0; input < _inputs; ++input) {
    double receive_rate = _input_receive_rates[input];

    _partial_input_cycles[input] += receive_rate;
    while (_partial_input_cycles[input] >= 1.0) {
      Flit* const f = _input_channels[input]->Receive();
      if (f) {
        _in_queue_flits.emplace(input, f);
        activity = true;
      }
      _partial_input_cycles[input] -= 1.0;
    }
  }

  return activity;
}

void InterposerRouter::SetInputPortRate() {
  _input_receive_rates[_input_channels.size() - 1] = _receive_rate;
}

void InterposerRouter::SetOutputPortRate() {
  _output_send_rates[_output_channels.size() - 1] = _send_rate;
}
