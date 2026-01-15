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

#ifndef _ACTIVE_CHIPLET_HPP
#define _ACTIVE_CHIPLET_HPP

#include "network.hpp"
#include "booksim_config.hpp"
#include "routefunc.hpp"
#include <unordered_map>
class Chiplet_Network : public Network {
 public:
  Chiplet_Network(const Configuration& config, const string& name);
  ~Chiplet_Network();

  static int chiplet_num;
  static int interface_num;
  bool is_active_interposer;

  // [link type][src router][dest router]=(port, latency)
  // [0]: node to router
  // [1]: interposer(boundary) router to boundary router
  // [2]-[chiplet_num+1]: chiplet router to chiplet router

  // optional: [chiplet_num+2]: interposer router to interposer router
  static vector<map<int, map<int, pair<int, int>>>> router_list;

  // [node] = port
  static vector<map<int, int>> node_port_map;

  // [node] = router_id
  map<int, int> node_list;

  // [node] = router_id (fast lookup using unordered_map)
  static unordered_map<int, int> node_to_router_map;

  // [router] = chiplet_id
  static unordered_map<int, int> router_to_chiplet;

  // Routing tables (using unordered_map for O(1) lookup performance)
  // [current_router][next_router] = output_port (router-to-router next hop)
  static unordered_map<int, unordered_map<int, int>> global_routing_table;

  // [chiplet_id][router_id] = vector of nearest boundary router IDs
  static vector<unordered_map<int, vector<int>>> boundary_table;

  // [cur_chiplet_id][next_chiplet_id] = <router_id, router_id>
  static vector<unordered_map<int, unordered_map<int, pair<int, int>>>>
      passive_routing_table;

  string integration_file;
  string interposer_file;
  BookSimConfig interposer_config;
  vector<string> chiplet_files;
  vector<BookSimConfig> chiplet_configs;

  // Store offset for each chiplet and interposer (unified structure)
  // Index [0..chiplet_num-1]: chiplet offsets
  // Index [chiplet_num]: interposer offset (if active)
  // Index [chiplet_num+1 or last]: total number of routers (one past the end)
  // This allows calculating number of routers: router_offsets[i+1] - router_offsets[i]
  static vector<int> router_offsets;

  vector<int> chiplet_node_offsets;  // node_offset for each chiplet
  int interposer_node_offset;        // node_offset for interposer

  void _ComputeSize(const Configuration& config);
  void _BuildNet(const Configuration& config);
  void _Alloc();

  // chiplet_id = -1: read interposer file
  // chiplet_id >= 0: read chiplet file
  void readFile(const string& file_name, int chiplet_id = -1);
  void readIntegrationFile(const string& file_name);

  static void RegisterRoutingFunctions();

  // Routing table construction functions
  void BuildGlobalRoutingTable();
  void BuildBoundaryRoutingTable();
  void BuildChipletRoutingTable();
  int ComputeRouterDistance(
      int src_router, int dest_router,
      const map<int, map<int, pair<int, int>>>& chiplet_graph);

  inline bool IsActiveInterposer() const { return is_active_interposer; }

  // Inline helper functions for fast access
  static inline int NodeToRouter(int node) { return node_to_router_map[node]; }

  static inline int NodeToPort(int node) {
    int router = NodeToRouter(node);
    return node_port_map[router][node];
  }

  static inline int RouterToChiplet(int router) {
    return router_to_chiplet[router];
  }

  // Routing table access functions
  // Get output port to reach adjacent next_router from current_router
  static inline int GetGlobalRoutingPort(int current_router, int next_router) {
    auto it = global_routing_table.find(current_router);
    if (it != global_routing_table.end()) {
      auto port_it = it->second.find(next_router);
      if (port_it != it->second.end()) {
        return port_it->second;
      }
    }
    assert(false && "No direct connection found");
  }

  // Check if two routers are directly connected
  static inline bool AreRoutersAdjacent(int router1, int router2) {
    return global_routing_table[router1].count(router2) > 0;
  }

  static inline const vector<int>& GetNearestBoundaryRouters(int chiplet_id,
                                                             int router_id) {
    return boundary_table[chiplet_id][router_id];
  }

  // Get all adjacent routers of a given router
  static inline const unordered_map<int, int>& GetAdjacentRouters(
      int router_id) {
    static const unordered_map<int, int> empty_map;
    auto it = global_routing_table.find(router_id);
    if (it != global_routing_table.end()) {
      return it->second;
    }
    return empty_map;
  }

  // Configuration accessors for traffic manager
  inline const BookSimConfig& GetInterposerConfig() const {
    return interposer_config;
  }

  inline const vector<BookSimConfig>& GetChipletConfigs() const {
    return chiplet_configs;
  }

  inline const BookSimConfig& GetChipletConfig(int chiplet_id) const {
    assert(chiplet_id >= 0 && chiplet_id < (int)chiplet_configs.size());
    return chiplet_configs[chiplet_id];
  }

  // Routing table accessor
  static inline const unordered_map<int, unordered_map<int, int>>&
  GetGlobalRoutingTable() {
    return global_routing_table;
  }

  // Offset accessors for routing
  static inline int GetRouterOffset(int index) {
    assert(index >= 0 && index < (int)router_offsets.size());
    return router_offsets[index];
  }

  // Get number of routers in a chiplet/interposer
  static inline int GetNumRoutersInChiplet(int index) {
    assert(index >= 0 && index + 1 < (int)router_offsets.size());
    return router_offsets[index + 1] - router_offsets[index];
  }
};

void min_chiplet_network(const Router* r, const Flit* f, int in_channel,
                         OutputSet* outputs, bool inject);
void dor_mesh_chiplet_network(const Router* r, const Flit* f, int in_channel,
                              OutputSet* outputs, bool inject);
void chip_level_dor_mesh_chiplet_network(const Router* r, const Flit* f,
                                         int in_channel, OutputSet* outputs,
                                         bool inject);

int dim_order_mesh(int cur_router, int dest_router, int chiplet_id, int n = 2,
                   bool descending = false);
#endif