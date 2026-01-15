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

#include "chiplet_router.hpp"

#include "allocator.hpp"
#include "buffer.hpp"
#include "buffer_monitor.hpp"
#include "buffer_state.hpp"
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

ChipletRouter::ChipletRouter(const Configuration& config, Module* parent,
                             const string& name, int id, int inputs,
                             int outputs)
    : IQRouter(config, parent, name, id, inputs, outputs) {

  _chiplet_id = Chiplet_Network::router_to_chiplet[id];
  _chiplet_flit_size = config.GetInt("chiplet_flit_size");
}

ChipletRouter::~ChipletRouter() {}