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

#include "chiplet_network.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <cmath>

unordered_map<int, int> Chiplet_Network::node_to_router_map;
vector<map<int, int>> Chiplet_Network::node_port_map;
unordered_map<int, int> Chiplet_Network::router_to_chiplet;
vector<map<int, map<int, pair<int, int>>>> Chiplet_Network::router_list;

// Initialize static routing tables and offsets
unordered_map<int, unordered_map<int, int>>
    Chiplet_Network::global_routing_table;
vector<unordered_map<int, vector<int>>> Chiplet_Network::boundary_table;
vector<unordered_map<int, unordered_map<int, pair<int, int>>>>
    Chiplet_Network::passive_routing_table;
vector<int> Chiplet_Network::router_offsets;
int Chiplet_Network::chiplet_num;
int Chiplet_Network::interface_num;

Chiplet_Network::Chiplet_Network(const Configuration& config,
                                 const string& name)
    : Network(config, name) {
  chiplet_num = config.GetInt("chiplet_num");
  chiplet_files.resize(chiplet_num);
  chiplet_configs.resize(chiplet_num);
  chiplet_node_offsets.resize(chiplet_num);
  interposer_node_offset = 0;

  // used for interface
  _enable_interface = config.GetStr("interface") != "";
  interface_num = 0;

  // Check if interposer config is provided
  string interposer_config_file = config.GetStr("interposer_config");
  if (interposer_config_file != "") {
    cout << "Parsing interposer config file: " << interposer_config_file
         << endl;
    interposer_config.ParseFile(interposer_config_file);
    interposer_file = interposer_config.GetStr("network_file");
    is_active_interposer = true;
  } else {
    is_active_interposer = false;
  }

  // Initialize router_offsets vector
  // Size: chiplet_num (chiplet offsets) + 1 (total count) + (interposer offset if active)
  router_offsets.resize(chiplet_num + 1 + (is_active_interposer ? 1 : 0));
  router_offsets[0] = 0;  // First chiplet starts at 0

  // If active interposer: need [0][1][2..chiplet_num+1][chiplet_num+2] = chiplet_num + 3
  // If passive interposer: need [0][1][2..chiplet_num+1] = chiplet_num + 2
  router_list.resize(chiplet_num + (is_active_interposer ? 3 : 2));

  _ComputeSize(config);
  node_port_map.resize(_size);
  _Alloc();
  _BuildNet(config);
}

Chiplet_Network::~Chiplet_Network() {
  for (Interface* i : _interfaces) {
    delete i;
  }
}

void Chiplet_Network::_ComputeSize(const Configuration& config) {

  vector<string> chiplet_config_files = config.GetStrArray("chiplet_configs");
  assert(chiplet_config_files.size() == (size_t)chiplet_num);

  for (size_t i = 0; i < chiplet_config_files.size(); i++) {
    cout << "Parsing chiplet config file: " << chiplet_config_files[i] << endl;
    chiplet_configs[i].ParseFile(chiplet_config_files[i]);

    chiplet_files[i] = chiplet_configs[i].GetStr("network_file");
    if (chiplet_files[i] == "") {
      cout << "No network file name provided" << endl;
      exit(-1);
    }

    readFile(chiplet_files[i], i);

    // Initialize VC configuration for this chiplet
    InitChipletVCConfig(chiplet_configs[i], i);

    // add routing_function_chiplet to each chiplet_configs
    if (!is_active_interposer) {
      chiplet_configs[i].Assign("routing_function_chiplet",
                                config.GetStr("routing_function_chiplet"));
    }
    chiplet_configs[i].Assign("enable_parallel",
                              config.GetInt("enable_parallel"));
  }

  if (interposer_file != "") {
    cout << "Parsing interposer file: " << interposer_file << endl;
    readFile(interposer_file);

    interposer_config.Assign("enable_parallel",
                             config.GetInt("enable_parallel"));
    // Initialize VC configuration for interposer (chiplet_id = chiplet_num)
    InitChipletVCConfig(interposer_config, chiplet_num);
  }

  integration_file = config.GetStr("integration_file");
  if (integration_file == "") {
    cout << "No integration file name provided" << endl;
    exit(-1);
  }
  readIntegrationFile(integration_file);

  // Set the last element of router_offsets to total number of routers
  router_offsets.back() = router_list[0].size();

  _channels = 0;
  cout << "========================Network File Parsed=================\n";
  cout << "******************node listing**********************\n";
  map<int, int>::iterator iter;
  for (iter = node_list.begin(); iter != node_list.end(); iter++) {
    cout << "Node " << iter->first;
    cout << "\tRouter " << iter->second << endl;
  }

  map<int, map<int, pair<int, int>>>::iterator iter3;
  cout << "\n****************router to node listing*************\n";
  for (iter3 = router_list[0].begin(); iter3 != router_list[0].end(); iter3++) {
    cout << "Router " << iter3->first << endl;
    map<int, pair<int, int>>::iterator iter2;
    for (iter2 = iter3->second.begin(); iter2 != iter3->second.end(); iter2++) {
      cout << "\t Node " << iter2->first << " lat " << iter2->second.second
           << endl;
    }
  }

  // Print router connections and count channels
  for (size_t type = 1; type < router_list.size(); type++) {
    // Print section header based on type
    if (type == 1) {
      cout << "\n*****************Interposer router to Boundary router "
              "listing************\n";
    } else if (type == (size_t)(chiplet_num + 2)) {
      cout << "\n*****************Interposer router to Interposer router "
              "listing************\n";
    } else {
      cout << "\n*****************Internal Chiplet " << (type - 2)
           << " router listing************\n";
    }

    // Iterate through all routers of this type
    for (iter3 = router_list[type].begin(); iter3 != router_list[type].end();
         iter3++) {
      if (iter3->second.size() == 0) {
        continue;  // Skip routers with no connections
      }

      // Print router header
      const char* router_prefix = (type == 1) ? "Boundary Router"
                                  : (type == (size_t)(chiplet_num + 2))
                                      ? "Interposer Router"
                                      : "Chiplet Router";
      cout << router_prefix << " " << iter3->first << endl;

      // Iterate through all connections for this router
      map<int, pair<int, int>>::iterator iter2;
      for (iter2 = iter3->second.begin(); iter2 != iter3->second.end();
           iter2++) {
        cout << "\t" << router_prefix << " " << iter2->first << " lat "
             << iter2->second.second << endl;

        // Count channels
        if (_enable_interface && type == 1) {
          // upstream router ---> upstream interface ---> downstream interface ---> downstream router: 3 channels
          _channels += 3;
          interface_num++;
        } else {
          // upstream router ---> downstream router: 1 channel
          _channels++;
        }
      }
    }
  }

  _size = router_list[0].size();  // number of router
  _nodes = node_list.size();      // number of node

  // Fill node_to_router_map from node_list for fast O(1) lookup
  for (map<int, int>::iterator i = node_list.begin(); i != node_list.end();
       i++) {
    node_to_router_map[i->first] = i->second;
  }
}

void Chiplet_Network::readFile(const string& file_name, int chiplet_id) {
  static int router_offset = 0;
  static int node_offset = 0;

  // Record current offset for this chiplet/interposer
  // Update router_offsets
  if (chiplet_id == -1) {
    // Interposer: store at index chiplet_num
    router_offsets[chiplet_num] = router_offset;
    interposer_node_offset = node_offset;
  } else {
    // Chiplet: store at index chiplet_id
    router_offsets[chiplet_id] = router_offset;
    chiplet_node_offsets[chiplet_id] = node_offset;
  }

  ifstream network_list;
  string line;
  enum ParseState { HEAD_TYPE = 0, HEAD_ID, BODY_TYPE, BODY_ID, LINK_WEIGHT };
  enum ParseType { NODE = 0, ROUTER, UNKNOWN };

  network_list.open(file_name.c_str());
  if (!network_list.is_open()) {
    cout << "Chiplet_Network: can't open network file " << file_name << endl;
    exit(-1);
  }

  // chiplet_id == -1: interposer file -> router_list[chiplet_num+2]
  // chiplet_id >= 0: chiplet file -> router_list[chiplet_id+2]
  int type = (chiplet_id == -1) ? (chiplet_num + 2) : (chiplet_id + 2);

  int max_local_router_id = -1;
  int max_local_node_id = -1;

  // loop through the entire file
  while (!network_list.eof()) {
    getline(network_list, line);
    if (line == "") {
      continue;
    }

    ParseState state = HEAD_TYPE;
    int pos = 0;
    int next_pos = -1;
    string temp;

    int head_id = -1;
    ParseType head_type = UNKNOWN;
    ParseType body_type = UNKNOWN;
    int body_id = -1;
    int link_weight = 1;
    int global_head_id = 0;
    int global_head = 0;
    int global_body = 0;

    do {
      // skip empty spaces
      next_pos = line.find(" ", pos);
      temp = line.substr(pos, next_pos - pos);
      pos = next_pos + 1;
      if (temp == "" || temp == " ") {
        continue;
      }

      switch (state) {
        case HEAD_TYPE:
          if (temp == "router") {
            head_type = ROUTER;
          } else if (temp == "node") {
            head_type = NODE;
          } else {
            cout << "Chiplet_Network: Unknown head of line type " << temp
                 << "\n";
            assert(false);
          }
          state = HEAD_ID;
          break;

        case HEAD_ID:
          head_id = atoi(temp.c_str());
          max_local_router_id = max(max_local_router_id, head_id);

          // initialize router structures with offset
          global_head_id = head_id + router_offset;
          if (router_list[0].count(global_head_id) == 0) {
            router_list[0][global_head_id] = map<int, pair<int, int>>();
          }
          if (router_list[type].count(global_head_id) == 0) {
            router_list[type][global_head_id] = map<int, pair<int, int>>();
          }

          // Record router to chiplet mapping
          // chiplet_id == -1 means interposer, store as chiplet_num
          router_to_chiplet[global_head_id] =
              (chiplet_id == -1) ? chiplet_num : chiplet_id;

          state = BODY_TYPE;
          break;

        case LINK_WEIGHT:
          if (temp == "router" || temp == "node") {
            // ignore, this is the start of next connection
          } else {
            link_weight = atoi(temp.c_str());
            global_head = head_id + router_offset;
            global_body = (body_type == NODE) ? (body_id + node_offset)
                                              : (body_id + router_offset);

            if (body_type == ROUTER) {
              router_list[type][global_head][global_body].second = link_weight;
            } else {
              router_list[0][global_head][global_body].second = link_weight;
            }
            break;
          }
          // intentionally letting it flow through

        case BODY_TYPE:
          if (temp == "router") {
            body_type = ROUTER;
          } else if (temp == "node") {
            body_type = NODE;
          } else {
            cout << "Chiplet_Network: Unknown body type " << temp << "\n";
            assert(false);
          }
          state = BODY_ID;
          break;

        case BODY_ID:
          body_id = atoi(temp.c_str());

          global_head = head_id + router_offset;

          if (body_type == NODE) {
            max_local_node_id = max(max_local_node_id, body_id);
            global_body = body_id + node_offset;

            // Check if node already connected to a different router
            if (node_list.count(global_body) != 0 &&
                node_list[global_body] != global_head) {
              cout << "Chiplet_Network: Node " << global_body
                   << " trying to connect to multiple routers " << global_head
                   << " and " << node_list[global_body] << endl;
              assert(false);
            }
            node_list[global_body] = global_head;

            // router to node connection (node port is -1, meaning ejection port)
            router_list[0][global_head][global_body] = pair<int, int>(-1, 1);

          } else if (body_type == ROUTER) {
            max_local_router_id = max(max_local_router_id, body_id);
            global_body = body_id + router_offset;

            // initialize body router if necessary
            if (router_list[0].count(global_body) == 0) {
              router_list[0][global_body] = map<int, pair<int, int>>();
            }
            if (router_list[type].count(global_body) == 0) {
              router_list[type][global_body] = map<int, pair<int, int>>();
            }

            // Record router to chiplet mapping
            // chiplet_id == -1 means interposer, store as chiplet_num
            router_to_chiplet[global_body] =
                (chiplet_id == -1) ? chiplet_num : chiplet_id;

            // router to router connection (bidirectional)
            router_list[type][global_head][global_body] = pair<int, int>(-1, 1);
            if (router_list[type][global_body].count(global_head) == 0) {
              router_list[type][global_body][global_head] =
                  pair<int, int>(-1, 1);
            }
          }

          if (head_type == NODE && body_type == NODE) {
            cout << "Chiplet_Network: Cannot connect node to node " << temp
                 << "\n";
            assert(false);
          } else if (head_type == NODE && body_type == ROUTER) {
            cout << "Chiplet_Network: Node cannot be head (router should be "
                    "head)"
                 << endl;
            assert(false);
          }

          state = LINK_WEIGHT;
          break;

        default:
          cout << "Chiplet_Network: Unknown parse state\n";
          assert(false);
          break;
      }

    } while (pos != 0);

    if (state != LINK_WEIGHT && state != BODY_TYPE) {
      cout << "Chiplet_Network: Incomplete parse of the line: " << line << endl;
    }
  }

  network_list.close();

  // Update offsets for next chiplet/interposer
  router_offset += (max_local_router_id + 1);
  node_offset += (max_local_node_id + 1);

  if (chiplet_id == -1) {
    cout << "Interposer: " << (max_local_router_id + 1) << " routers, "
         << (max_local_node_id + 1) << " nodes" << endl;
  } else {
    cout << "Chiplet " << chiplet_id << ": " << (max_local_router_id + 1)
         << " routers, " << (max_local_node_id + 1) << " nodes" << endl;
  }
  cout << "Current router_offset: " << router_offset
       << ", node_offset: " << node_offset << endl;
}

void Chiplet_Network::readIntegrationFile(const string& file_name) {
  ifstream integration_list;
  string line;

  integration_list.open(file_name.c_str());
  if (!integration_list.is_open()) {
    cout << "Chiplet_Network: can't open integration file " << file_name
         << endl;
    exit(-1);
  }

  cout << "Reading integration file: " << file_name << endl;

  // loop through the entire file
  while (!integration_list.eof()) {
    getline(integration_list, line);
    if (line == "") {
      continue;
    }

    // Parse format: chiplet <id> router <router_id> chiplet/interposer <id> router <router_id> <latency>
    istringstream iss(line);
    string word;
    vector<string> tokens;

    while (iss >> word) {
      tokens.push_back(word);
    }

    if (tokens.size() < 7) {
      cout << "Chiplet_Network: Invalid integration line format: " << line
           << endl;
      continue;
    }

    // Parse first endpoint: chiplet <id> router <router_id>
    if (tokens[0] != "chiplet") {
      cout << "Chiplet_Network: Expected 'chiplet', got: " << tokens[0] << endl;
      continue;
    }

    int chiplet1_id = atoi(tokens[1].c_str());
    if (tokens[2] != "router") {
      cout << "Chiplet_Network: Expected 'router', got: " << tokens[2] << endl;
      continue;
    }
    int router1_local_id = atoi(tokens[3].c_str());

    // Parse second endpoint: chiplet/interposer <id> router <router_id>
    bool is_interposer = (tokens[4] == "interposer");
    int chiplet2_id = -1;
    int router2_local_id;
    int latency = 1;

    if (is_interposer) {
      // Format: chiplet <id> router <router_id> interposer router <router_id> <latency>
      if (tokens[5] != "router") {
        cout << "Chiplet_Network: Expected 'router', got: " << tokens[5]
             << endl;
        continue;
      }
      router2_local_id = atoi(tokens[6].c_str());
      if (tokens.size() >= 8) {
        latency = atoi(tokens[7].c_str());
      }
    } else {
      // Format: chiplet <id> router <router_id> chiplet <id> router <router_id> <latency>
      if (tokens[4] != "chiplet") {
        cout << "Chiplet_Network: Expected 'chiplet', got: " << tokens[4]
             << endl;
        continue;
      }
      chiplet2_id = atoi(tokens[5].c_str());
      if (tokens[6] != "router") {
        cout << "Chiplet_Network: Expected 'router', got: " << tokens[6]
             << endl;
        continue;
      }
      router2_local_id = atoi(tokens[7].c_str());
      if (tokens.size() >= 9) {
        latency = atoi(tokens[8].c_str());
      }
    }

    // Convert to global IDs using router_offsets
    int global_router1 = router_offsets[chiplet1_id] + router1_local_id;
    int global_router2;

    if (is_interposer) {
      global_router2 = router_offsets[chiplet_num] + router2_local_id;
    } else {
      global_router2 = router_offsets[chiplet2_id] + router2_local_id;
    }

    // Initialize if necessary
    if (router_list[1].count(global_router1) == 0) {
      router_list[1][global_router1] = map<int, pair<int, int>>();
    }
    if (router_list[1].count(global_router2) == 0) {
      router_list[1][global_router2] = map<int, pair<int, int>>();
    }

    // Add bidirectional connection to router_list[1] (I2B - interposer/boundary connections)
    router_list[1][global_router1][global_router2] =
        pair<int, int>(-1, latency);
    router_list[1][global_router2][global_router1] =
        pair<int, int>(-1, latency);

    // if (is_interposer) {
    //     cout << "  chiplet " << chiplet1_id << " router " << router1_local_id
    //          << " (global " << global_router1 << ") <-> "
    //          << "interposer router " << router2_local_id
    //          << " (global " << global_router2 << "), latency: " << latency << endl;
    // } else {
    //     cout << "  chiplet " << chiplet1_id << " router " << router1_local_id
    //          << " (global " << global_router1 << ") <-> "
    //          << "chiplet " << chiplet2_id << " router " << router2_local_id
    //          << " (global " << global_router2 << "), latency: " << latency << endl;
    // }
  }

  integration_list.close();
  cout << "Integration file reading completed." << endl;
}

void Chiplet_Network::RegisterRoutingFunctions() {
  gRoutingFunctionMap["min_chiplet_network"] = &min_chiplet_network;
  gRoutingFunctionMap["dor_mesh_chiplet_network"] = &dor_mesh_chiplet_network;

  gRoutingFunctionMap["chip_level_dor_mesh_chiplet_network"] =
      &chip_level_dor_mesh_chiplet_network;
}

void chip_level_dor_mesh_chiplet_network(const Router* r, const Flit* f,
                                         int in_channel, OutputSet* outputs,
                                         bool inject) {

  // we here only modify the f->intm_chiplet list here
  assert(outputs == nullptr);
  assert(inject == false);
  assert(r != nullptr);  // we need the r->GetChipletID()
  assert(f != nullptr);  // we need the f->dest_chiplet

  // 1. Calculate next chiplet using dimension-order routing for 2D mesh topology
  int cur_chiplet_id = r->GetChipletID();
  int dest_chiplet = f->dest_chiplet;
  int chiplet_num = Chiplet_Network::chiplet_num;
  bool descending = false;

  assert(cur_chiplet_id >= 0 && cur_chiplet_id < chiplet_num);
  assert(dest_chiplet >= 0 && dest_chiplet < chiplet_num);
  assert(cur_chiplet_id != dest_chiplet &&
         "Current and destination chiplet must be different");

  // For 2D mesh: chiplet_num = k^2, so k = sqrt(chiplet_num)
  int n = 2;  // 2D mesh
  int k = (int)round(sqrt(chiplet_num));

  int dim_left;
  int cur = cur_chiplet_id;
  int dest_copy = dest_chiplet;

  // Dimension-order routing logic
  if (descending) {
    // Start from highest dimension
    for (dim_left = (n - 1); dim_left > 0; --dim_left) {
      if ((cur * k / chiplet_num) != (dest_copy * k / chiplet_num)) {
        break;
      }
      cur = (cur * k) % chiplet_num;
      dest_copy = (dest_copy * k) % chiplet_num;
    }
    cur = (cur * k) / chiplet_num;
    dest_copy = (dest_copy * k) / chiplet_num;
  } else {
    // Start from lowest dimension
    for (dim_left = 0; dim_left < (n - 1); ++dim_left) {
      if ((cur % k) != (dest_copy % k)) {
        break;
      }
      cur /= k;
      dest_copy /= k;
    }
    cur %= k;
    dest_copy %= k;
  }

  // Determine direction in the selected dimension
  bool positive_dir = (cur < dest_copy);

  // Calculate next chiplet ID - move one step in the selected dimension
  int dim_size = 1;
  for (int i = 0; i < dim_left; ++i) {
    dim_size *= k;
  }

  int next_chiplet_id = cur_chiplet_id;
  if (positive_dir) {
    next_chiplet_id += dim_size;  // Move in positive direction
  } else {
    next_chiplet_id -= dim_size;  // Move in negative direction
  }

  // 2. add the <intermediate_router, intermediate_router> to the f->intm_chiplet list
  // Get the connection between current chiplet and next chiplet from passive_routing_table

  auto chiplet_it = Chiplet_Network::passive_routing_table[cur_chiplet_id].find(
      next_chiplet_id);
  if (chiplet_it !=
      Chiplet_Network::passive_routing_table[cur_chiplet_id].end()) {
    // Found connections between cur_chiplet and next_chiplet
    const auto& router_pairs = chiplet_it->second;

    if (!router_pairs.empty()) {
      // Select the first available router pair
      // router_pairs: [router_in_cur_chiplet] = {router_in_cur, router_in_next}
      const auto& first_pair = router_pairs.begin()->second;
      int router_in_cur = first_pair.first;
      int router_in_next = first_pair.second;

      // Add to intermediate chiplet list

      f->intm_chiplet.push_back({router_in_cur, router_in_next});
    } else {
      assert(false && "No router pairs found between chiplets");
    }
  } else {
    assert(false &&
           "No connection found between current and next chiplet in passive "
           "routing table");
  }
}

void min_chiplet_network(const Router* r, const Flit* f, int in_channel,
                         OutputSet* outputs, bool inject) {
  assert(false);
}

int dim_order_mesh(int cur_router, int dest_router, int chiplet_id, int n,
                   bool descending) {
  int router_offset = Chiplet_Network::GetRouterOffset(chiplet_id);
  int num_routers = Chiplet_Network::GetNumRoutersInChiplet(chiplet_id);

  assert(n == 2 && "Only 2D mesh is supported");
  int k = (int)round(sqrt(num_routers));

  // Convert global router IDs to local IDs within the chiplet
  int cur_local = cur_router - router_offset;
  int dest_local = dest_router - router_offset;

  assert(cur_local != dest_local);

  int dim_left;
  int cur = cur_local;
  int dest_local_copy = dest_local;

  // Total routers in this chiplet = k^n
  int total_routers = num_routers;

  // Dimension-order routing logic (same as dor_next_mesh)
  if (descending) {
    // Start from highest dimension
    for (dim_left = (n - 1); dim_left > 0; --dim_left) {
      if ((cur * k / total_routers) != (dest_local_copy * k / total_routers)) {
        break;
      }
      cur = (cur * k) % total_routers;
      dest_local_copy = (dest_local_copy * k) % total_routers;
    }
    cur = (cur * k) / total_routers;
    dest_local_copy = (dest_local_copy * k) / total_routers;
  } else {
    // Start from lowest dimension
    for (dim_left = 0; dim_left < (n - 1); ++dim_left) {
      if ((cur % k) != (dest_local_copy % k)) {
        break;
      }
      cur /= k;
      dest_local_copy /= k;
    }
    cur %= k;
    dest_local_copy %= k;
  }

  // Determine direction in the selected dimension
  bool positive_dir = (cur < dest_local_copy);

  // Calculate next router local ID
  // Move one step in the selected dimension
  int dim_size = 1;
  for (int i = 0; i < dim_left; ++i) {
    dim_size *= k;
  }

  int next_router_local = cur_local;
  if (positive_dir) {
    next_router_local += dim_size;  // Move in positive direction
  } else {
    next_router_local -= dim_size;  // Move in negative direction
  }

  // Convert back to global router ID
  int next_router_global = next_router_local + router_offset;

  return next_router_global;
}

void dor_mesh_chiplet_network(const Router* r, const Flit* f, int in_channel,
                              OutputSet* outputs, bool inject) {
  int out_port = -1;

  if (!inject) {

    int next_router = -1;

    if (f->ph_chiplet == Flit::Inbound || f->ph_chiplet == Flit::Intra) {

      if (f->dest_router == r->GetID()) {
        // reach the destination router, no need to route
        out_port = Chiplet_Network::NodeToPort(f->dest);
      } else {
        next_router =
            dim_order_mesh(r->GetID(), f->dest_router, r->GetChipletID());
        out_port =
            Chiplet_Network::GetGlobalRoutingPort(r->GetID(), next_router);
      }

    } else if (f->ph_chiplet == Flit::Outbound) {

      if (f->intm_chiplet.empty()) {
        // only passive interposer will enter here
        tRoutingFunction rf_chiplet = r->GetChipletRoutingFunction();
        if (rf_chiplet) {
          rf_chiplet(r, f, in_channel, nullptr, false);
        }
      }

      assert(!f->intm_chiplet.empty() &&
             "Outbound flit must have intermediate routers");

      // Get the next hop from the routing path
      auto next_hop = f->intm_chiplet.front();
      int intm_router = next_hop.first;

      if (r->GetID() == intm_router) {
        // reach the boundary router, find the next router
        next_router = next_hop.second;
        f->intm_chiplet.pop_front();

        int next_chiplet_id = Chiplet_Network::RouterToChiplet(next_router);
        f->ph_chiplet = (next_chiplet_id == f->dest_chiplet) ? Flit::Inbound
                                                             : Flit::Outbound;

      } else {
        // not reach the boundary router, continue to the next intermediate router
        next_router =
            dim_order_mesh(r->GetID(), intm_router, r->GetChipletID());
      }

      out_port = Chiplet_Network::GetGlobalRoutingPort(r->GetID(), next_router);

    } else {
      assert(false);
    }

    assert(out_port != -1);
  }

  int chiplet_id = (r == nullptr) ? f->src_chiplet : r->GetChipletID();
  const ChipletVC& vc_config = gChipletVCConfigs[chiplet_id];
  int vc_start = 0;
  int vc_end = vc_config.NumVCs - 1;

  if (f->type == Flit::READ_REQUEST) {
    vc_start = vc_config.ReadReqBeginVC;
    vc_end = vc_config.ReadReqEndVC;
  } else if (f->type == Flit::WRITE_REQUEST) {
    vc_start = vc_config.WriteReqBeginVC;
    vc_end = vc_config.WriteReqEndVC;
  } else if (f->type == Flit::READ_REPLY) {
    vc_start = vc_config.ReadReplyBeginVC;
    vc_end = vc_config.ReadReplyEndVC;
  } else if (f->type == Flit::WRITE_REPLY) {
    vc_start = vc_config.WriteReplyBeginVC;
    vc_end = vc_config.WriteReplyEndVC;
  }

  outputs->Clear();
  outputs->AddRange(out_port, vc_start, vc_end);
}

void Chiplet_Network::_BuildNet(const Configuration& config) {
  int* outport = (int*)malloc(sizeof(int) * _size);
  for (int i = 0; i < _size; i++) {
    outport[i] = 0;
  }

  cout << "==========================Node to Router =====================\n";

  // Helper function to determine router type based on router_id using router_to_chiplet
  auto get_router_type = [&](int router_id) -> Router::ChipletRouterType {
    // Get chiplet id directly from router_to_chiplet
    auto iter = router_to_chiplet.find(router_id);
    if (iter == router_to_chiplet.end()) {
      // Router not found in mapping
      return Router::DEFAULT;
    }

    int chiplet_id = iter->second;

    // Interposer router: chiplet_id == chiplet_num
    if (chiplet_id == chiplet_num) {
      return Router::INTERPOSER;
    }

    // Boundary router: in router_list[1] (boundary connections)
    if (router_list[1].count(router_id) == 1) {
      return Router::BOUNDARY;
    }

    // Chiplet router: chiplet_id in [0, chiplet_num-1]
    if (chiplet_id >= 0 && chiplet_id <= chiplet_num - 1) {
      return Router::CHIPLET;
    }

    // Unknown/Default type
    return Router::DEFAULT;
  };

  // Iterate through all routers (router_list[0] contains all routers with nodes)
  map<int, map<int, pair<int, int>>>::iterator niter;
  for (niter = router_list[0].begin(); niter != router_list[0].end(); niter++) {
    int router_id = niter->first;
    int radix = niter->second.size();  // Number of nodes connected to router

    // Calculate total radix: nodes + router connections
    for (size_t i = 1; i < router_list.size(); i++) {
      if (router_list[i].count(router_id) == 1) {
        map<int, map<int, pair<int, int>>>::iterator riter_data =
            router_list[i].find(router_id);
        radix += riter_data->second.size();
      }
    }

    // Determine router type
    Router::ChipletRouterType router_type = get_router_type(router_id);

    // Get chiplet_id for this router
    int chiplet_id = router_to_chiplet[router_id];

    // Select appropriate config based on chiplet_id
    const Configuration* router_config;
    if (chiplet_id == chiplet_num) {
      // Interposer router
      router_config = &interposer_config;
    } else if (chiplet_id >= 0 && chiplet_id <= chiplet_num - 1) {
      // Chiplet router or boundary router
      router_config = &chiplet_configs[chiplet_id];
    } else {
      cout << "Error: Invalid chiplet_id " << chiplet_id << " for router "
           << router_id << endl;
      assert(false);
    }

    // Convert router type to string for output
    const char* type_str;
    switch (router_type) {
      case Router::INTERPOSER:
        type_str = "Interposer";
        break;
      case Router::BOUNDARY:
        type_str = "Boundary";
        break;
      case Router::CHIPLET:
        type_str = "Chiplet";
        break;
      case Router::DEFAULT:
        type_str = "Default";
        break;
      default:
        type_str = "Unknown";
        break;
    }

    cout << type_str << " Router " << router_id << " (chiplet " << chiplet_id
         << ") radix " << radix << endl;

    // Create router
    ostringstream router_name;
    router_name << "router_" << router_id;

    if (router_type == Router::DEFAULT) {
      cout << "Error: Unknown router type for router " << router_id << endl;
      assert(false);
    }

    _routers[router_id] =
        Router::NewRouter(*router_config, this, router_name.str(), router_id,
                          radix, radix, router_type);
    _timed_modules.push_back(_routers[router_id]);

    // Add injection/ejection channels for nodes connected to this router
    map<int, pair<int, int>>::iterator nniter;
    for (nniter = niter->second.begin(); nniter != niter->second.end();
         nniter++) {
      int node_id = nniter->first;

      // Assign outport for this connection
      (niter->second)[node_id].first = outport[router_id];
      outport[router_id]++;

      cout << "\t connected to node " << node_id << " at outport "
           << nniter->second.first << " lat " << nniter->second.second << endl;

      // Set channel latencies
      _inject[node_id]->SetLatency(nniter->second.second);
      _inject_cred[node_id]->SetLatency(nniter->second.second);
      _eject[node_id]->SetLatency(nniter->second.second);
      _eject_cred[node_id]->SetLatency(nniter->second.second);

      // Connect channels to router
      _routers[router_id]->AddInputChannel(_inject[node_id],
                                           _inject_cred[node_id]);
      _routers[router_id]->AddOutputChannel(_eject[node_id],
                                            _eject_cred[node_id]);

      // Record node port mapping
      node_port_map[router_id][node_id] = nniter->second.first;
    }
  }

  cout << "==========================Router to Router =====================\n";
  cout << endl;

  int channel_count = 0;

  // Interface management for boundary connections
  int interface_id_counter = 0;
  map<pair<int, int>, int>
      interface_router_map;  // Maps (router1, router2) -> interface_id

  // Lambda function to get or create interface pair for a router connection
  auto get_or_create_interface_pair = [&](int router_id, int other_router_id,
                                          int& upstream_id,
                                          int& downstream_id) {
    auto forward_key = make_pair(router_id, other_router_id);
    auto reverse_key = make_pair(other_router_id, router_id);

    if (interface_router_map.find(forward_key) != interface_router_map.end()) {
      // Already created
      upstream_id = interface_router_map[forward_key];
      downstream_id = interface_router_map[reverse_key];
    } else {
      // Create new interface pair
      // Ensure _interfaces has enough space
      if (interface_id_counter >= (int)_interfaces.size()) {
        _interfaces.resize(interface_id_counter + 10);  // Allocate extra space
      }

      // Upstream interface (connected to router_id)
      ostringstream interface_name;
      interface_name << "interface_" << interface_id_counter;
      _interfaces[interface_id_counter] = Interface::NewInterface(
          config, this, interface_name.str(), interface_id_counter);
      _timed_modules.push_back(_interfaces[interface_id_counter]);
      upstream_id = interface_id_counter++;

      // Ensure space for downstream interface
      if (interface_id_counter > (int)_interfaces.size()) {
        assert(false);
      }

      // Downstream interface (connected to other_router_id)
      interface_name.str("");
      interface_name << "interface_" << interface_id_counter;
      _interfaces[interface_id_counter] = Interface::NewInterface(
          config, this, interface_name.str(), interface_id_counter);
      _timed_modules.push_back(_interfaces[interface_id_counter]);
      downstream_id = interface_id_counter++;

      // Store mapping
      interface_router_map[forward_key] = upstream_id;
      interface_router_map[reverse_key] = downstream_id;
    }
  };

  // Lambda function to create direct router-to-router connection
  auto connect_routers_direct = [&](int router_id, int other_router_id,
                                    int outport_num, int latency) {
    cout << "\t connected to router " << other_router_id << " (chiplet "
         << router_to_chiplet[other_router_id] << ")" << " using link "
         << channel_count << " at outport " << outport_num << " lat " << latency
         << endl;

    _chan[channel_count]->SetLatency(latency);
    _chan_cred[channel_count]->SetLatency(latency);

    _routers[router_id]->AddOutputChannel(_chan[channel_count],
                                          _chan_cred[channel_count]);
    _routers[other_router_id]->AddInputChannel(_chan[channel_count],
                                               _chan_cred[channel_count]);
    channel_count++;
  };

  // Lambda function to create interface-mediated connection
  auto connect_routers_with_interface = [&](int router_id, int other_router_id,
                                            int outport_num, int latency) {
    int upstream_id, downstream_id;
    get_or_create_interface_pair(router_id, other_router_id, upstream_id,
                                 downstream_id);

    // Connection 1: router -> upstream interface (latency = 1)
    cout << "\t ----> upstream interface " << upstream_id << " using link "
         << channel_count << endl;

    _chan[channel_count]->SetLatency(1);
    _chan_cred[channel_count]->SetLatency(1);
    _routers[router_id]->AddOutputChannel(_chan[channel_count],
                                          _chan_cred[channel_count]);
    _interfaces[upstream_id]->AddInputChannel(_chan[channel_count],
                                              _chan_cred[channel_count]);
    _chan[channel_count]->SetRemoteRouterSink(_routers[other_router_id]);
    _interfaces[upstream_id]->SetFarstreamRouter(config,
                                                 _routers[other_router_id]);
    channel_count++;

    // Connection 2: upstream interface -> downstream interface (latency = link_latency)
    cout << "\t ----> downstream interface " << downstream_id << " using link "
         << channel_count << " lat " << latency << endl;

    _chan[channel_count]->SetLatency(latency);
    _chan_cred[channel_count]->SetLatency(latency);
    _interfaces[downstream_id]->AddInputChannel(_chan[channel_count],
                                                _chan_cred[channel_count]);
    _interfaces[upstream_id]->AddOutputChannel(_chan[channel_count],
                                               _chan_cred[channel_count]);
    channel_count++;

    // Connection 3: downstream interface -> other router (latency = 1)
    cout << "\t ----> router " << other_router_id << " using link "
         << channel_count << endl;

    _chan[channel_count]->SetLatency(1);
    _chan_cred[channel_count]->SetLatency(1);
    _interfaces[downstream_id]->AddOutputChannel(_chan[channel_count],
                                                 _chan_cred[channel_count]);
    _routers[other_router_id]->AddInputChannel(_chan[channel_count],
                                               _chan_cred[channel_count]);
    // Set nearstream config for downstream interface
    Configuration* nearstream_config =
        router_to_chiplet[other_router_id] == chiplet_num
            ? &interposer_config
            : &chiplet_configs[router_to_chiplet[other_router_id]];
    _interfaces[downstream_id]->SetNearstreamConfig(nearstream_config);
    channel_count++;
  };

  // Process all router-to-router connections
  // router_list[0]: node to router (already processed)
  // router_list[1]: boundary connections (interposer <-> boundary router)
  // router_list[2] to [chiplet_num+1]: chiplet internal connections
  // router_list[chiplet_num+2]: optional interposer internal connections

  for (size_t list_idx = 1; list_idx < router_list.size(); list_idx++) {

    // Print section header
    if (list_idx == 1) {
      cout << "\n==========================Boundary Router "
              "Connections=====================\n";
    } else if (list_idx >= 2 && list_idx <= (size_t)(chiplet_num + 1)) {
      int chiplet_id = list_idx - 2;
      cout << "\n==========================Chiplet " << chiplet_id
           << " Internal Connections=====================\n";
    } else if (list_idx == (size_t)(chiplet_num + 2)) {
      cout << "\n==========================Interposer Internal "
              "Connections=====================\n";
    }

    // Determine if this is a boundary connection that needs interface support
    bool use_interface = (list_idx == 1) && _enable_interface;

    // Iterate through all routers in this connection type
    for (auto& router_pair : router_list[list_idx]) {
      int router_id = router_pair.first;
      auto& connections = router_pair.second;

      cout << "Router " << router_id << " (chiplet "
           << router_to_chiplet[router_id] << ")" << endl;

      // Process each connection from this router
      for (auto& conn_pair : connections) {
        int other_router_id = conn_pair.first;
        int& outport_num = conn_pair.second.first;
        int latency = conn_pair.second.second;

        // Assign output port number
        outport_num = outport[router_id];
        outport[router_id]++;

        // Create connection based on whether interface is enabled
        if (use_interface) {
          connect_routers_with_interface(router_id, other_router_id,
                                         outport_num, latency);
        } else {
          connect_routers_direct(router_id, other_router_id, outport_num,
                                 latency);
        }

        // Special handling for boundary connections
        if (list_idx == 1) {

          // Set output and input port rates for boundary connections
          _routers[router_id]->SetOutputPortRate();
          _routers[other_router_id]->SetInputPortRate();

          // For boundary connections, reset output VC with downstream chiplet's config
          int downstream_chiplet_id = router_to_chiplet[other_router_id];
          const Configuration& downstream_config =
              (downstream_chiplet_id == chiplet_num)
                  ? interposer_config
                  : chiplet_configs[downstream_chiplet_id];
          _routers[router_id]->SetOutputVC(config, downstream_config,
                                           outport_num);
        }
      }

      // Reinitialize VC allocator only for boundary/interposer routers
      if (list_idx == 1) {
        int chiplet_id = router_to_chiplet[router_id];
        const Configuration& router_config = (chiplet_id == chiplet_num)
                                                 ? interposer_config
                                                 : chiplet_configs[chiplet_id];
        _routers[router_id]->ReinitVCAllocator(router_config);
        cout << "Reinitialized VC allocator for router " << router_id << endl;
      }
    }
  }

  BuildGlobalRoutingTable();

  if (is_active_interposer) {
    // used staic boundary router binding
    BuildBoundaryRoutingTable();
  } else {
    BuildChipletRoutingTable();
  }

  free(outport);
}

void Chiplet_Network::_Alloc() {
  Network::_Alloc();
  _interface_size = interface_num;
  if (_enable_parallel && _num_threads > 1) {
    _item_per_task =
        std::max(_min_item_per_task, (_size + _interface_size) / _num_threads);
  }
  _interfaces.resize(interface_num);
}

// Build complete routing table for all routers
void Chiplet_Network::BuildGlobalRoutingTable() {
  cout << "========================== Building Global Routing Table (Adjacent "
          "Routers) =====================\n";

  // Step 1: Merge all router connections into a unified graph
  map<int, map<int, pair<int, int>>> unified_graph;

  // Merge router-to-router connections from all types
  for (size_t type_idx = 1; type_idx < router_list.size(); type_idx++) {
    for (const auto& router_item : router_list[type_idx]) {
      const auto& router_id = router_item.first;
      const auto& connections = router_item.second;
      if (unified_graph.find(router_id) == unified_graph.end()) {
        unified_graph[router_id] = connections;
      } else {
        // Merge connections
        for (const auto& conn_item : connections) {
          const auto& neighbor_id = conn_item.first;
          const auto& port_latency = conn_item.second;
          unified_graph[router_id][neighbor_id] = port_latency;
        }
      }
    }
  }

  cout << "Unified graph created with " << unified_graph.size() << " routers\n";

  // Step 2: Build routing table - directly map adjacent routers to output ports
  // global_routing_table[current_router][next_router] = output_port
  // where next_router is directly connected to current_router
  global_routing_table.clear();

  for (const auto& router_item : unified_graph) {
    const auto& router_id = router_item.first;
    const auto& neighbors = router_item.second;
    for (const auto& neighbor_item : neighbors) {
      const auto& neighbor_id = neighbor_item.first;
      const auto& port_latency = neighbor_item.second;
      int output_port = port_latency.first;  // port number
      global_routing_table[router_id][neighbor_id] = output_port;
    }
  }

  cout << "Global routing table built successfully.\n";
  cout << "Total routers: " << global_routing_table.size() << endl;

  int total_entries = 0;
  for (const auto& src_item : global_routing_table) {
    const auto& neighbors = src_item.second;
    total_entries += neighbors.size();
  }
  cout << "Total adjacent connections: " << total_entries << endl;
}

// Build chiplet-internal routing tables
// Build boundary router table: find nearest boundary router for each router
void Chiplet_Network::BuildBoundaryRoutingTable() {
  cout << "========================== Building Boundary Routing Table "
          "=====================\n";

  // Clear and resize boundary table
  boundary_table.clear();
  boundary_table.resize(chiplet_num);

  // Step 1: Identify boundary routers for each chiplet
  vector<vector<int>> boundary_routers(chiplet_num);

  for (int chiplet_id = 0; chiplet_id < chiplet_num; chiplet_id++) {
    int list_idx = chiplet_id + 2;  // router_list[2] = chiplet 0, etc.

    for (const auto& router_item : router_list[list_idx]) {
      const auto& router_id = router_item.first;
      // Check if this router is also in boundary connections (router_list[1])
      if (router_list[1].find(router_id) != router_list[1].end()) {
        boundary_routers[chiplet_id].push_back(router_id);
      }
    }

    cout << "Chiplet " << chiplet_id << " has "
         << boundary_routers[chiplet_id].size() << " boundary routers\n";
  }

  // Step 2: For each router, find nearest boundary router(s)
  for (int chiplet_id = 0; chiplet_id < chiplet_num; chiplet_id++) {
    int list_idx = chiplet_id + 2;
    const auto& chiplet_graph = router_list[list_idx];

    for (const auto& router_item : chiplet_graph) {
      const auto& router_id = router_item.first;
      vector<int> distances;

      // Compute distance to each boundary router
      for (int boundary_id : boundary_routers[chiplet_id]) {
        int distance =
            ComputeRouterDistance(router_id, boundary_id, chiplet_graph);
        distances.push_back(distance);
      }

      // Find minimum distance
      if (!distances.empty()) {
        int min_dist = *min_element(distances.begin(), distances.end());

        // Collect all boundary routers with minimum distance
        vector<int> nearest_boundaries;
        for (size_t i = 0; i < distances.size(); i++) {
          if (distances[i] == min_dist) {
            nearest_boundaries.push_back(boundary_routers[chiplet_id][i]);
          }
        }

        boundary_table[chiplet_id][router_id] = nearest_boundaries;
      }
    }
  }

  cout << "Boundary routing table built successfully.\n";
}

// Compute distance between two routers in a chiplet
int Chiplet_Network::ComputeRouterDistance(
    int src_router, int dest_router,
    const map<int, map<int, pair<int, int>>>& chiplet_graph) {
  if (src_router == dest_router) {
    return 0;
  }

  // Use Dijkstra to find shortest path
  auto get_neighbors = [&](int node) {
    vector<int> neighbors;
    if (chiplet_graph.find(node) != chiplet_graph.end()) {
      for (const auto& neighbor_item : chiplet_graph.at(node)) {
        neighbors.push_back(neighbor_item.first);
      }
    }
    return neighbors;
  };

  auto get_edge_cost = [&](int from, int to) {
    return chiplet_graph.at(from).at(to).second;  // latency
  };

  map<int, int> distance;
  for (const auto& router_item : chiplet_graph) {
    const auto& router = router_item.first;
    distance[router] = std::numeric_limits<int>::max();
  }
  distance[src_router] = 0;

  priority_queue<pair<int, int>, vector<pair<int, int>>,
                 greater<pair<int, int>>>
      pq;
  pq.push({0, src_router});

  while (!pq.empty()) {
    auto top_item = pq.top();
    int curr_dist = top_item.first;
    int curr_node = top_item.second;
    pq.pop();

    if (curr_node == dest_router) {
      return curr_dist;
    }

    if (curr_dist > distance[curr_node]) {
      continue;
    }

    for (int neighbor : get_neighbors(curr_node)) {
      int edge_cost = get_edge_cost(curr_node, neighbor);
      int new_dist = distance[curr_node] + edge_cost;

      if (new_dist < distance[neighbor]) {
        distance[neighbor] = new_dist;
        pq.push({new_dist, neighbor});
      }
    }
  }

  return distance[dest_router];
}

// Build chiplet-to-chiplet routing table for passive interposer mode
// Maps direct connections between chiplets through boundary routers
void Chiplet_Network::BuildChipletRoutingTable() {
  cout << "========================== Building Chiplet Routing Table (Passive "
          "Interposer) =====================\n";

  // Clear and resize passive routing table
  passive_routing_table.clear();
  passive_routing_table.resize(chiplet_num);

  // Iterate through all boundary connections in router_list[1]
  // These connections link boundary routers from different chiplets
  const auto& boundary_connections = router_list[1];

  for (const auto& router_item : boundary_connections) {
    const auto& router1 = router_item.first;
    const auto& connections = router_item.second;
    int chiplet1 = router_to_chiplet[router1];

    for (const auto& conn_item : connections) {
      const auto& router2 = conn_item.first;
      // const auto& port_latency = conn_item.second;  // Unused
      int chiplet2 = router_to_chiplet[router2];

      // Only store inter-chiplet connections (not intra-chiplet)
      if (chiplet1 != chiplet2) {
        // Store bidirectional connections
        // chiplet1 -> chiplet2: router1 (in chiplet1) connects to router2 (in chiplet2)
        passive_routing_table[chiplet1][chiplet2][router1] = {router1, router2};

        // cout << "Chiplet " << chiplet1 << " -> Chiplet " << chiplet2
        //      << ": Router " << router1 << " -> Router " << router2 << endl;
      }
    }
  }
}
