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

#include "chiplettrafficmanager.hpp"
#include "chiplet_network.hpp"
#include "packet_reply_info.hpp"
#include "random_utils.hpp"
#include "injection.hpp"

#include <limits>
#include <sstream>
#include <cmath>
#include <algorithm>

ChipletTrafficManager::ChipletTrafficManager(const Configuration& config,
                                             const vector<Network*>& net)
    : TrafficManager(config, net) {

  // discarded _rf in TrafficManager, use _rf_chiplets instead

  Chiplet_Network* chiplet_net = dynamic_cast<Chiplet_Network*>(net[0]);
  if (chiplet_net == nullptr) {
    Error("ChipletTrafficManager requires a Chiplet_Network");
  }

  _is_active_interposer = chiplet_net->IsActiveInterposer();
  _global_routing_table = &(Chiplet_Network::GetGlobalRoutingTable());

  _chiplet_num = config.GetInt("chiplet_num");
  _enable_interface = config.GetStr("interface") != "";

  // _rf_chiplets[0] - _rf_chiplets[_chiplet_num-1] are for chiplets
  // _rf_chiplets[_chiplet_num] is for interposer
  _rf_chiplets.resize(_chiplet_num + (_is_active_interposer ? 1 : 0));
  _lookahead_routing_chiplets.resize(_chiplet_num +
                                     (_is_active_interposer ? 1 : 0));
  // we do not use _noq for chiplet traffic manager
  _noq = false;

  // Get configurations from network
  _chiplet_configs = chiplet_net->GetChipletConfigs();
  if (_is_active_interposer) {
    _interposer_config = chiplet_net->GetInterposerConfig();
  }

  // Initialize routing functions for each chiplet
  for (int i = 0; i < _chiplet_num; ++i) {
    string rf =
        _chiplet_configs[i].GetStr("routing_function") + "_chiplet_network";
    map<string, tRoutingFunction>::const_iterator rf_iter =
        gRoutingFunctionMap.find(rf);
    if (rf_iter == gRoutingFunctionMap.end()) {
      Error("Invalid chiplet routing function: " + rf);
    }
    _rf_chiplets[i] = rf_iter->second;
    cout << "Chiplet " << i << " routing function: " << rf << endl;

    _lookahead_routing_chiplets[i] =
        !_chiplet_configs[i].GetInt("routing_delay");
  }

  // Initialize routing function for interposer if active or chiplet-level if passive
  if (_is_active_interposer) {
    string rf =
        _interposer_config.GetStr("routing_function") + "_chiplet_network";
    map<string, tRoutingFunction>::const_iterator rf_iter =
        gRoutingFunctionMap.find(rf);
    if (rf_iter == gRoutingFunctionMap.end()) {
      Error("Invalid interposer routing function: " + rf);
    }
    _rf_chiplets[_chiplet_num] = rf_iter->second;
    cout << "Interposer routing function: " << rf << endl;

    _lookahead_routing_chiplets[_chiplet_num] =
        !_interposer_config.GetInt("routing_delay");
  } else {

    string rf = config.GetStr("routing_function_chiplet") + "_chiplet_network";
    map<string, tRoutingFunction>::const_iterator rf_iter =
        gRoutingFunctionMap.find(rf);
    if (rf_iter == gRoutingFunctionMap.end()) {
      Error("Invalid interposer routing function: " + rf);
    }
    _rf_passive = rf_iter->second;
  }

  // Read chiplet flit size configuration (in bytes) for each chiplet
  _chiplet_flit_sizes.resize(_chiplet_num + (_is_active_interposer ? 1 : 0));
  for (int i = 0; i < _chiplet_num; ++i) {
    _chiplet_flit_sizes[i] = _chiplet_configs[i].GetInt("chiplet_flit_size");
    if (_chiplet_flit_sizes[i] <= 0) {
      ostringstream err;
      err << "chiplet_flit_size must be a positive integer for chiplet " << i;
      Error(err.str());
    }
    cout << "Chiplet " << i << " flit size: " << _chiplet_flit_sizes[i]
         << " bytes" << endl;
  }

  // Read interposer flit size if active
  if (_is_active_interposer) {
    _chiplet_flit_sizes[_chiplet_num] =
        _interposer_config.GetInt("chiplet_flit_size");
    if (_chiplet_flit_sizes[_chiplet_num] <= 0) {
      Error("chiplet_flit_size must be a positive integer for interposer");
    }
    cout << "Interposer flit size: " << _chiplet_flit_sizes[_chiplet_num]
         << " bytes" << endl;
  }

  if (!_enable_interface) {
    for (int i = 1; i < _chiplet_num + (_is_active_interposer ? 1 : 0); ++i) {
      assert(_chiplet_flit_sizes[i] == _chiplet_flit_sizes[0] &&
             "All chiplet_flit_size must be the same when interface is not "
             "enabled");
    }
  }

  // Calculate average flit size across all chiplets (weighted by number of nodes)
  double total_flit_size_weighted = 0.0;
  int total_nodes = 0;
  for (int source = 0; source < _nodes; ++source) {
    int router = Chiplet_Network::NodeToRouter(source);
    int chiplet = Chiplet_Network::RouterToChiplet(router);
    total_flit_size_weighted += _chiplet_flit_sizes[chiplet];
    total_nodes++;
  }
  double average_flit_size = total_flit_size_weighted / total_nodes;
  cout << "Average flit size across all nodes: " << average_flit_size
       << " bytes" << endl;

  // Recalculate injection rates if they were specified in flits
  // In TrafficManager constructor, _load was incorrectly divided by packet size in bytes
  // We need to correct this to account for byte-to-flit conversion
  if (config.GetInt("injection_rate_uses_flits")) {
    for (int c = 0; c < _classes; ++c) {
      double avg_packet_size_bytes = _GetAveragePacketSize(c);
      // Undo the incorrect division by bytes in TrafficManager constructor
      _load[c] *= avg_packet_size_bytes;
      // Apply correct division: convert packet size in bytes to number of flits
      double avg_packet_size_flits = avg_packet_size_bytes / average_flit_size;
      _load[c] /= avg_packet_size_flits;
      cout << "Class " << c << " adjusted injection rate: " << _load[c]
           << " (packet size: " << avg_packet_size_bytes << " bytes, "
           << avg_packet_size_flits << " flits)" << endl;
    }

    // Recreate InjectionProcess objects with the corrected load values
    vector<string> injection_process = config.GetStrArray("injection_process");
    injection_process.resize(_classes, injection_process.back());

    for (int c = 0; c < _classes; ++c) {
      // Delete the old InjectionProcess object created in TrafficManager constructor
      delete _injection_process[c];
      // Create new InjectionProcess with corrected load
      _injection_process[c] = InjectionProcess::New(injection_process[c],
                                                    _nodes, _load[c], &config);
      cout << "Class " << c
           << " InjectionProcess recreated with load: " << _load[c] << endl;
    }
  }

  // Initialize buffer states for each node and subnet according to the chiplet
  for (int source = 0; source < _nodes; ++source) {
    _buf_states[source].resize(_subnets);
    _last_class[source].resize(_subnets, 0);
    _last_vc[source].resize(_subnets);

    int router = Chiplet_Network::NodeToRouter(source);
    int chiplet = Chiplet_Network::RouterToChiplet(router);

    // Select the appropriate config based on chiplet ID
    // chiplet == chiplet_num means interposer
    const BookSimConfig& cur_config = (chiplet == _chiplet_num)
                                          ? _interposer_config
                                          : _chiplet_configs[chiplet];

    for (int subnet = 0; subnet < _subnets; ++subnet) {
      ostringstream tmp_name;
      tmp_name << "terminal_buf_state_" << source << "_" << subnet;
      BufferState* bs = new BufferState(cur_config, this, tmp_name.str());
      int vc_alloc_delay = cur_config.GetInt("vc_alloc_delay");
      int sw_alloc_delay = cur_config.GetInt("sw_alloc_delay");
      int router_latency = cur_config.GetInt("routing_delay") +
                           (cur_config.GetInt("speculative")
                                ? max(vc_alloc_delay, sw_alloc_delay)
                                : (vc_alloc_delay + sw_alloc_delay));
      int min_latency = 1 + _net[subnet]->GetInject(source)->GetLatency() +
                        router_latency +
                        _net[subnet]->GetInjectCred(source)->GetLatency();
      bs->SetMinLatency(min_latency);
      _buf_states[source][subnet] = bs;
      _last_vc[source][subnet].resize(_classes, -1);
    }
  }

  _total_in_flight_packets.resize(_classes);
}

ChipletTrafficManager::~ChipletTrafficManager() {}

void ChipletTrafficManager::_GeneratePacket(int source, int stype, int cl,
                                            int time) {
  assert(stype != 0);

  // Get the chiplet ID for the source node to determine flit size
  int router = Chiplet_Network::NodeToRouter(source);
  int chiplet = Chiplet_Network::RouterToChiplet(router);
  int chiplet_flit_size = _chiplet_flit_sizes[chiplet];

  Flit::FlitType packet_type = Flit::ANY_TYPE;
  int packet_size_bytes = _GetNextPacketSize(cl);  //input size in bytes
  // Convert packet size from bytes to number of flits
  // Formula: num_flits = ceil(packet_size_bytes / chiplet_flit_size_bytes)
  int size = (int)ceil((double)packet_size_bytes / chiplet_flit_size);

  int pid = _cur_pid++;
  assert(_cur_pid);
  int packet_destination = _traffic_pattern[cl]->dest(source);
  bool record = false;
  bool watch = gWatchOut && (_packets_to_watch.count(pid) > 0);
  if (_use_read_write[cl]) {
    if (stype > 0) {
      if (stype == 1) {
        packet_type = Flit::READ_REQUEST;
        packet_size_bytes = _read_request_size[cl];
      } else if (stype == 2) {
        packet_type = Flit::WRITE_REQUEST;
        packet_size_bytes = _write_request_size[cl];
      } else {
        ostringstream err;
        err << "Invalid packet type: " << packet_type;
        Error(err.str());
      }
    } else {
      PacketReplyInfo* rinfo = _repliesPending[source].front();
      if (rinfo->type == Flit::READ_REQUEST) {  //read reply
        packet_size_bytes = _read_reply_size[cl];
        packet_type = Flit::READ_REPLY;
      } else if (rinfo->type == Flit::WRITE_REQUEST) {  //write reply
        packet_size_bytes = _write_reply_size[cl];
        packet_type = Flit::WRITE_REPLY;
      } else {
        ostringstream err;
        err << "Invalid packet type: " << rinfo->type;
        Error(err.str());
      }
      packet_destination = rinfo->source;
      time = rinfo->time;
      record = rinfo->record;
      _repliesPending[source].pop_front();
      rinfo->Free();
    }
    // Recalculate size based on the specific packet type size
    size = (int)ceil((double)packet_size_bytes / chiplet_flit_size);
  }

  if ((packet_destination < 0) || (packet_destination >= _nodes)) {
    ostringstream err;
    err << "Incorrect packet destination " << packet_destination
        << " for stype " << packet_type;
    Error(err.str());
  }

  if ((_sim_state == running) ||
      ((_sim_state == draining) && (time < _drain_time))) {
    record = _measure_stats[cl];
  }

  int subnetwork = ((packet_type == Flit::ANY_TYPE) ? RandomInt(_subnets - 1)
                                                    : _subnet[packet_type]);

  if (watch) {
    *gWatchOut << GetSimTime() << " | " << "node" << source << " | "
               << "Enqueuing packet " << pid << " at time " << time << "."
               << endl;
  }

  for (int i = 0; i < size; ++i) {
    Flit* f = Flit::New();
    f->id = _cur_id++;
    assert(_cur_id);
    f->fid = i;
    f->psize = packet_size_bytes;
    f->fsize = chiplet_flit_size;
    f->pid = pid;
    f->watch = watch | (gWatchOut && (_flits_to_watch.count(f->id) > 0));
    f->subnetwork = subnetwork;
    f->src = source;
    f->src_router = Chiplet_Network::NodeToRouter(source);
    f->src_chiplet = Chiplet_Network::RouterToChiplet(f->src_router);
    f->ctime = time;
    f->record = record;
    f->cl = cl;

    _total_in_flight_flits[f->cl].insert(make_pair(f->id, f));
    if (record) {
      _measured_in_flight_flits[f->cl].insert(make_pair(f->id, f));
    }

    if (gTrace) {
      cout << "New Flit " << f->src << endl;
    }
    f->type = packet_type;

    if (i == 0) {  // Head flit
      f->head = true;
      //packets are only generated to nodes smaller or equal to limit
      f->dest = packet_destination;
      f->dest_router = Chiplet_Network::NodeToRouter(packet_destination);
      f->dest_chiplet = Chiplet_Network::RouterToChiplet(f->dest_router);

      f->ph_chiplet =
          (f->src_chiplet == f->dest_chiplet) ? Flit::Intra : Flit::Outbound;

      if (_total_in_flight_packets[f->cl].count(f->pid) == 0) {
        int _dest_chiplet_flit_size = _chiplet_flit_sizes[f->dest_chiplet];
        _total_in_flight_packets[f->cl].insert(make_pair(
            f->pid, ceil((double)packet_size_bytes / _dest_chiplet_flit_size)));
      }
    } else {
      f->head = false;
      f->dest = -1;
      f->dest_router = -1;
      f->dest_chiplet = -1;
    }
    switch (_pri_type) {
      case class_based:
        f->pri = _class_priority[cl];
        assert(f->pri >= 0);
        break;
      case age_based:
        f->pri = numeric_limits<int>::max() - time;
        assert(f->pri >= 0);
        break;
      case sequence_based:
        f->pri = numeric_limits<int>::max() - _packet_seq_no[source];
        assert(f->pri >= 0);
        break;
      default:
        f->pri = 0;
    }
    if (i == (size - 1)) {  // Tail flit
      f->tail = true;
    } else {
      f->tail = false;
    }

    f->vc = -1;

    // need to matain the routing reachability for the entire network under modular routing
    // the handling for active interposer and passive interposer are different
    // the routing dependency can be decomposed into several sub-relationships in a way that original routing relationship to be naturally generated by the connection of these sub-relationships
    // active interposer did while passive interposer did not

    if (f->ph_chiplet == Flit::Outbound) {
      if (_is_active_interposer) {
        _BoundaryBinding(f);
      } else {
        _PassiveBinding(f);
      }
    }

    if (f->watch) {
      *gWatchOut << GetSimTime() << " | " << "node" << source << " | "
                 << "Enqueuing flit " << f->id << " (packet " << f->pid
                 << ") at time " << time << "." << endl;
    }

    _partial_packets[source][cl].push_back(f);
  }
}

void ChipletTrafficManager::_Step() {
  _Step_DeadlockChecking();
  vector<map<int, Flit*>> flits = _Step_Ejection();

  if (!_empty_network) {
    _Inject();
  }

  _Step_Injection();

  _Step_Retirement_Advanced(flits);

  ++_time;
  assert(_time);
  if (gTrace) {
    cout << "TIME " << _time << endl;
  }
}

void ChipletTrafficManager::_Step_DeadlockChecking() {
  bool flits_in_flight = false;
  if (_enable_interface) {
    for (int c = 0; c < _classes; ++c) {
      flits_in_flight |= !_total_in_flight_packets[c].empty();
    }
  } else {
    for (int c = 0; c < _classes; ++c) {
      flits_in_flight |= !_total_in_flight_flits[c].empty();
    }
  }

  if (flits_in_flight && (_deadlock_timer++ >= _deadlock_warn_timeout)) {
    _deadlock_timer = 0;
    cout << "WARNING: Possible network deadlock.\n";
  }
}

void ChipletTrafficManager::_Step_Injection() {
  for (int subnet = 0; subnet < _subnets; ++subnet) {
    for (int n = 0; n < _nodes; ++n) {

      int router = Chiplet_Network::NodeToRouter(n);
      int chiplet = Chiplet_Network::RouterToChiplet(router);

      tRoutingFunction rf = _rf_chiplets[chiplet];

      Flit* f = nullptr;

      BufferState* const dest_buf = _buf_states[n][subnet];

      int const last_class = _last_class[n][subnet];

      int class_limit = _classes;

      if (_hold_switch_for_packet) {
        list<Flit*> const& pp = _partial_packets[n][last_class];
        if (!pp.empty() && !pp.front()->head &&
            !dest_buf->IsFullFor(pp.front()->vc)) {
          f = pp.front();
          assert(f->vc == _last_vc[n][subnet][last_class]);

          // if we're holding the connection, we don't need to check that class
          // again in the for loop
          --class_limit;
        }
      }

      for (int i = 1; i <= class_limit; ++i) {
        int const c = (last_class + i) % _classes;

        list<Flit*> const& pp = _partial_packets[n][c];

        if (pp.empty()) {
          continue;
        }

        Flit* const cf = pp.front();
        assert(cf);
        assert(cf->cl == c);

        if (cf->subnetwork != subnet) {
          continue;
        }

        if (f && (f->pri >= cf->pri)) {
          continue;
        }

        if (cf->head && cf->vc == -1) {  // Find first available VC

          OutputSet route_set;
          rf(NULL, cf, -1, &route_set, true);
          std::vector<OutputSet::sSetElement> const& os = route_set.GetSet();
          assert(os.size() == 1);
          OutputSet::sSetElement const& se = *os.begin();
          assert(se.output_port == -1);
          int vc_start = se.vc_start;
          int vc_end = se.vc_end;
          int vc_count = vc_end - vc_start + 1;
          if (_noq) {
            assert(false &&
                   "NOQ is not supported for current implementation of chiplet "
                   "traffic manager");
            assert(_lookahead_routing_chiplets[chiplet]);
            const FlitChannel* inject = _net[subnet]->GetInject(n);
            const Router* router = inject->GetSink();
            assert(router);
            int in_channel = inject->GetSinkPort();

            // NOTE: Because the lookahead is not for injection, but for the
            // first hop, we have to temporarily set cf's VC to be non-negative
            // in order to avoid seting of an assertion in the routing function.
            cf->vc = vc_start;
            rf(router, cf, in_channel, &cf->la_route_set, false);
            cf->vc = -1;

            if (cf->watch) {
              *gWatchOut << GetSimTime() << " | " << "node" << n << " | "
                         << "Generating lookahead routing info for flit "
                         << cf->id << " (NOQ)." << endl;
            }
            std::vector<OutputSet::sSetElement> const& sl =
                cf->la_route_set.GetSet();
            assert(sl.size() == 1);
            int next_output = sl.begin()->output_port;
            vc_count /= router->NumOutputs();
            vc_start += next_output * vc_count;
            vc_end = vc_start + vc_count - 1;
            assert(vc_start >= se.vc_start && vc_start <= se.vc_end);
            assert(vc_end >= se.vc_start && vc_end <= se.vc_end);
            assert(vc_start <= vc_end);
          }
          if (cf->watch) {
            *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                       << "Finding output VC for flit " << cf->id << ":"
                       << endl;
          }
          for (int i = 1; i <= vc_count; ++i) {
            int const lvc = _last_vc[n][subnet][c];
            int const vc = (lvc < vc_start || lvc > vc_end)
                               ? vc_start
                               : (vc_start + (lvc - vc_start + i) % vc_count);
            assert((vc >= vc_start) && (vc <= vc_end));
            if (!dest_buf->IsAvailableFor(vc)) {
              if (cf->watch) {
                *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                           << "  Output VC " << vc << " is busy." << endl;
              }
            } else {
              if (dest_buf->IsFullFor(vc)) {
                if (cf->watch) {
                  *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                             << "  Output VC " << vc << " is full." << endl;
                }
              } else {
                if (cf->watch) {
                  *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                             << "  Selected output VC " << vc << "." << endl;
                }
                cf->vc = vc;
                break;
              }
            }
          }
        }

        if (cf->vc == -1) {
          if (cf->watch) {
            *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                       << "No output VC found for flit " << cf->id << "."
                       << endl;
          }
        } else {
          if (dest_buf->IsFullFor(cf->vc)) {
            if (cf->watch) {
              *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                         << "Selected output VC " << cf->vc
                         << " is full for flit " << cf->id << "." << endl;
            }
          } else {
            f = cf;
          }
        }
      }

      if (f) {

        assert(f->subnetwork == subnet);

        int const c = f->cl;

        if (f->head) {

          if (_lookahead_routing_chiplets[chiplet]) {
            if (!_noq) {
              const FlitChannel* inject = _net[subnet]->GetInject(n);
              const Router* router = inject->GetSink();
              assert(router);
              int in_channel = inject->GetSinkPort();
              rf(router, f, in_channel, &f->la_route_set, false);
              if (f->watch) {
                *gWatchOut << GetSimTime() << " | " << "node" << n << " | "
                           << "Generating lookahead routing info for flit "
                           << f->id << "." << endl;
              }
            } else if (f->watch) {
              *gWatchOut << GetSimTime() << " | " << "node" << n << " | "
                         << "Already generated lookahead routing info for flit "
                         << f->id << " (NOQ)." << endl;
            }
          } else {
            f->la_route_set.Clear();
          }

          dest_buf->TakeBuffer(f->vc);
          _last_vc[n][subnet][c] = f->vc;
        }

        _last_class[n][subnet] = c;

        _partial_packets[n][c].pop_front();

#ifdef TRACK_FLOWS
        ++_outstanding_credits[c][subnet][n];
        _outstanding_classes[n][subnet][f->vc].push(c);
#endif

        dest_buf->SendingFlit(f);

        if (_pri_type == network_age_based) {
          f->pri = numeric_limits<int>::max() - _time;
          assert(f->pri >= 0);
        }

        if (f->watch) {
          *gWatchOut << GetSimTime() << " | " << "node" << n << " | "
                     << "Injecting flit " << f->id << " into subnet " << subnet
                     << " at time " << _time << " with priority " << f->pri
                     << "." << endl;
        }
        f->itime = _time;

        // Pass VC "back"
        if (!_partial_packets[n][c].empty() && !f->tail) {
          Flit* const nf = _partial_packets[n][c].front();
          nf->vc = f->vc;
        }

        if ((_sim_state == warming_up) || (_sim_state == running)) {
          ++_sent_flits[c][n];
          if (f->head) {
            ++_sent_packets[c][n];
          }
        }

#ifdef TRACK_FLOWS
        ++_injected_flits[c][n];
#endif

        _net[subnet]->WriteFlit(f, n);
      }
    }
  }
}

bool ChipletTrafficManager::Run() {
  for (int sim = 0; sim < _total_sims; ++sim) {

    _time = 0;

    //remove any pending request from the previous simulations
    _requestsOutstanding.assign(_nodes, 0);
    for (int i = 0; i < _nodes; i++) {
      while (!_repliesPending[i].empty()) {
        _repliesPending[i].front()->Free();
        _repliesPending[i].pop_front();
      }
    }

    //reset queuetime for all sources
    for (int s = 0; s < _nodes; ++s) {
      _qtime[s].assign(_classes, 0);
      _qdrained[s].assign(_classes, false);
    }

    // warm-up ...
    // reset stats, all packets after warmup_time marked
    // converge
    // draing, wait until all packets finish
    _sim_state = warming_up;

    _ClearStats();

    for (int c = 0; c < _classes; ++c) {
      _traffic_pattern[c]->reset();
      _injection_process[c]->reset();
    }

    if (!_SingleSim()) {
      cout << "Simulation unstable, ending ..." << endl;
      return false;
    }

    // Empty any remaining packets
    cout << "Draining remaining packets ..." << endl;
    _empty_network = true;
    int empty_steps = 0;

    bool packets_left = false;
    if (_enable_interface) {
      for (int c = 0; c < _classes; ++c) {
        packets_left |= !_total_in_flight_packets[c].empty();
      }
    } else {
      for (int c = 0; c < _classes; ++c) {
        packets_left |= !_total_in_flight_flits[c].empty();
      }
    }

    while (packets_left) {
      _Step();

      ++empty_steps;

      if (empty_steps % 1000 == 0) {
        _DisplayRemaining();
      }

      packets_left = false;
      if (_enable_interface) {
        for (int c = 0; c < _classes; ++c) {
          packets_left |= !_total_in_flight_packets[c].empty();
        }
      } else {
        for (int c = 0; c < _classes; ++c) {
          packets_left |= !_total_in_flight_flits[c].empty();
        }
      }
    }
    //wait until all the credits are drained as well
    while (Credit::OutStanding() != 0) {
      _Step();
    }
    _empty_network = false;

    //for the love of god don't ever say "Time taken" anywhere else
    //the power script depend on it
    cout << "Time taken is " << _time << " cycles" << endl;

    if (_stats_out) {
      WriteStats(*_stats_out);
    }
    _UpdateOverallStats();
  }

  DisplayOverallStats();
  if (_print_csv_results) {
    DisplayOverallStatsCSV();
  }

  return true;
}

void ChipletTrafficManager::_PassiveBinding(Flit* f) {
  // we only use _rf_passive to indicate the next chiplet
  // other dynamic binding is in routing algothims
  assert(f->intm_chiplet.empty());
  const Router* r = _net[0]->GetRouter(f->src_router);
  _rf_passive(r, f, -1, nullptr, false);
}

void ChipletTrafficManager::_BoundaryBinding(Flit* f) {
  assert(f->intm_chiplet.empty());

  bool src_is_interposer = (f->src_chiplet == _chiplet_num);
  bool dest_is_interposer = (f->dest_chiplet == _chiplet_num);

  const auto& boundary_connections = Chiplet_Network::router_list[1];

  // Case 1: src is chiplet, dest is chiplet (chiplet -> interposer -> chiplet)
  if (!src_is_interposer && !dest_is_interposer) {
    // Get nearest boundary routers
    const vector<int>& src_boundaries =
        Chiplet_Network::GetNearestBoundaryRouters(f->src_chiplet,
                                                   f->src_router);
    const vector<int>& dest_boundaries =
        Chiplet_Network::GetNearestBoundaryRouters(f->dest_chiplet,
                                                   f->dest_router);

    if (src_boundaries.empty() || dest_boundaries.empty()) {
      assert(false &&
             "No boundary routers found for chiplet-to-chiplet routing");
      return;
    }

    // Select the first boundary router
    int src_boundary = src_boundaries[0];
    int dest_boundary = dest_boundaries[0];

    int src_interposer_router = -1;
    int dest_interposer_router = -1;

    // Find source interposer router
    auto src_it = boundary_connections.find(src_boundary);
    if (src_it != boundary_connections.end()) {
      for (const auto& conn : src_it->second) {
        int connected_router = conn.first;
        assert(Chiplet_Network::RouterToChiplet(connected_router) ==
               _chiplet_num);
        src_interposer_router = connected_router;
        break;
      }
    }

    // Find destination interposer router
    auto dest_it = boundary_connections.find(dest_boundary);
    if (dest_it != boundary_connections.end()) {
      for (const auto& conn : dest_it->second) {
        int connected_router = conn.first;
        assert(Chiplet_Network::RouterToChiplet(connected_router) ==
               _chiplet_num);
        dest_interposer_router = connected_router;
        break;
      }
    }

    // Build routing path as pairs: (src, dest) for each hop
    // Path: src_boundary -> src_interposer -> dest_interposer -> dest_boundary
    f->intm_chiplet.push_back({src_boundary, src_interposer_router});
    f->intm_chiplet.push_back({dest_interposer_router, dest_boundary});
  }
  // Case 2: src is interposer, dest is chiplet (interposer -> chiplet)
  else if (src_is_interposer && !dest_is_interposer) {
    const vector<int>& dest_boundaries =
        Chiplet_Network::GetNearestBoundaryRouters(f->dest_chiplet,
                                                   f->dest_router);

    if (dest_boundaries.empty()) {
      assert(false &&
             "No boundary routers found for interposer-to-chiplet routing");
      return;
    }

    int dest_boundary = dest_boundaries[0];
    int dest_interposer_router = -1;

    // Find destination interposer router (connected to dest boundary)
    auto dest_it = boundary_connections.find(dest_boundary);
    if (dest_it != boundary_connections.end()) {
      for (const auto& conn : dest_it->second) {
        int connected_router = conn.first;
        assert(Chiplet_Network::RouterToChiplet(connected_router) ==
               _chiplet_num);
        dest_interposer_router = connected_router;
        break;
      }
    }

    // Build routing path: dest_interposer -> dest_boundary
    f->intm_chiplet.push_back({dest_interposer_router, dest_boundary});
  }
  // Case 3: src is chiplet, dest is interposer (chiplet -> interposer)
  else if (!src_is_interposer && dest_is_interposer) {
    const vector<int>& src_boundaries =
        Chiplet_Network::GetNearestBoundaryRouters(f->src_chiplet,
                                                   f->src_router);

    if (src_boundaries.empty()) {
      assert(false &&
             "No boundary routers found for chiplet-to-interposer routing");
      return;
    }

    int src_boundary = src_boundaries[0];
    int src_interposer_router = -1;

    // Find source interposer router (connected to src boundary)
    auto src_it = boundary_connections.find(src_boundary);
    if (src_it != boundary_connections.end()) {
      for (const auto& conn : src_it->second) {
        int connected_router = conn.first;
        assert(Chiplet_Network::RouterToChiplet(connected_router) ==
               _chiplet_num);
        src_interposer_router = connected_router;
        break;
      }
    }

    // Build routing path: src_boundary -> src_interposer
    f->intm_chiplet.push_back({src_boundary, src_interposer_router});
  } else {
    assert(false && "Invalid routing case");
  }
}

void ChipletTrafficManager::_RetireFlit_DeadlockUpdate(Flit* f) {

  if (_enable_interface) {
    assert(_total_in_flight_packets[f->cl].count(f->pid) > 0);
    _total_in_flight_packets[f->cl][f->pid]--;
    if (_total_in_flight_packets[f->cl][f->pid] == 0) {
      _total_in_flight_packets[f->cl].erase(f->pid);
    }
  } else {
    assert(_total_in_flight_flits[f->cl].count(f->id) > 0);
    _total_in_flight_flits[f->cl].erase(f->id);

    if (f->record) {
      assert(_measured_in_flight_flits[f->cl].count(f->id) > 0);
      _measured_in_flight_flits[f->cl].erase(f->id);
    }
  }
}
