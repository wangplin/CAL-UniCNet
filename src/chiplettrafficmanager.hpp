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

#ifndef _CHIPLETTRAFFICMANAGER_HPP_
#define _CHIPLETTRAFFICMANAGER_HPP_

#include "trafficmanager.hpp"
#include "booksim_config.hpp"
#include <unordered_map>
class ChipletTrafficManager : public TrafficManager {
 public:
  ChipletTrafficManager(const Configuration& config,
                        const vector<Network*>& net);
  ~ChipletTrafficManager();

 protected:
  void _GeneratePacket(int source, int stype, int cl, int time) override;
  // void _RetireFlit( Flit *f, int dest ) override;
  void _Step() override;
  void _Step_DeadlockChecking() override;
  void _Step_Injection() override;

  void _RetireFlit_DeadlockUpdate(Flit* f) override;

  bool Run() override;

  void _BoundaryBinding(Flit* f);
  void _PassiveBinding(Flit* f);

 private:
  bool _is_active_interposer;
  int _chiplet_num;
  bool _enable_interface;

  // _rf_chiplets[0] - _rf_chiplets[_chiplet_num-1] are for chiplets
  // _rf_chiplets[_chiplet_num] is for interposer
  vector<tRoutingFunction> _rf_chiplets;
  vector<bool> _lookahead_routing_chiplets;
  tRoutingFunction _rf_passive;

  // Cache global routing table for faster access
  const unordered_map<int, unordered_map<int, int>>* _global_routing_table;

  BookSimConfig _interposer_config;
  vector<BookSimConfig> _chiplet_configs;

  // Flit size in bytes for each chiplet and interposer
  // _chiplet_flit_sizes[0] - _chiplet_flit_sizes[_chiplet_num-1] are for chiplets
  // _chiplet_flit_sizes[_chiplet_num] is for interposer (if active)
  vector<int> _chiplet_flit_sizes;

  // <class, <pid>>
  // since the interface will change the flit id and fid, we here check whether the entire packet has been received
  // this is only used for deadlock detection if interface is enable
  // <pid, ceil(psize/chiplet_filt_size)>
  vector<map<int, int>> _total_in_flight_packets;
};

#endif