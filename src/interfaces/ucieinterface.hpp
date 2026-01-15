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

#ifndef _UCIE_INTERFACE_HPP_
#define _UCIE_INTERFACE_HPP_

#include "interface.hpp"
#include "uciebuffer.hpp"
#include "uciebufferstate.hpp"
#include "ucieretrybuffer.hpp"
#include "buffer_state.hpp"
class Router;
class Allocator;
class Arbiter;

class UCIEInterface : public Interface {

 public:
  UCIEInterface(const Configuration& config, Module* parent, const string& name,
                int id);
  ~UCIEInterface();

  void AddInputChannel(FlitChannel* channel,
                       CreditChannel* backchannel) override;
  void AddOutputChannel(FlitChannel* channel,
                        CreditChannel* backchannel) override;

  void SetFarstreamRouter(const Configuration& config,
                          Router* farstream_router) override;
  void SetNearstreamConfig(Configuration* nearstream_config) override;

 protected:
  int _tx_input;  // TX: from upstream router
  int _rx_input;  // RX: from upstream tx interface

  // Parameters for uice interface
  // timing parameters is aligned with UCIe Controller RTL IP

  // TX Protocol layer

  int _ucie_flit_size;  // ucie contoller datapath width, in bytes

  int _tx_arbiter_delay;

  int _tx_packing_delay;
  int _tx_unpacking_delay;
  int _tx_nopacking_delay;

  int _tx_pipe_fifo_delay;

  // TX Adapter layer (Normal mode)

  int _next_tx_seq_num;  // TX sequence number for the next flit to be generated

  int _tx_fdi_fifo_delay;
  int _tx_adapter_packer_delay;
  int _tx_crc_delay;

  // TX Adapter layer (Replay mode)

  int _tx_replay_delay;
  int _tx_retry_buffer_delay;

  // TX PHY layer

  int _serdes_ratio;   // TX serdes ratio
  double _lane_width;  // lane width degration
  double _partial_lane_width;

  int _tx_serdes_delay;

  // RX PHY layer

  int _rx_serdes_delay;

  // RX Adapter layer

  int _next_expected_rx_seq_num;  // RX sequence number for the next flit expected to be received
  double _error_rate;  // error rate for the received flit
  int _replay_num;     // retry times for error flit

  int _rx_retry_delay;
  int _rx_adapter_unpacker_delay;
  int _rx_fdi_delay;

  // RX Protocol layer

  int _next_chiplet_flit_size;  // flit size for the next chiplet

  int _rx_packing_delay;
  int _rx_unpacking_delay;
  int _rx_nopacking_delay;

  queue<Flit*> _rx_serdes_queue;

  enum TX_retry_state {
    state_min = 0,
    main = state_min,  // normal
    replay_pending,    // receive nak, stall fdi fifo, until pipeline emtpy
    replay,            // real replay mode
    state_max = replay
  };

  TX_retry_state _tx_retry_state;

  enum RX_retry_state {
    state_minimum = 0,
    idle = state_minimum,  // idle state
    nak_scheduled,         // receive nak, schedule nak flit
    state_maximum = nak_scheduled
  };

  RX_retry_state _rx_retry_state;

  // used for tx for configure the _tx_fifo vcs
  // upstream(tx) interface ---> downstream(rx) router
  Router* _farstream_router;

  // used for rx to configure the _next_chiplet_flit_size and _downstream_router_state
  // downstream(rx) interface ---> downstream(rx) router
  Configuration* _nearstream_config;

  // TX Protocol Layer
  UCIEBuffer* _tx_fifo;
  UCIEBuffer* _tx_pipe_fifo;

  UCIEBuffer* _tx_fdi_fifo;
  UCIERetryBuffer* _retry_buffer;

  UCIEBuffer* _rx_fifo;

  // <time, <vc, output>>
  deque<pair<int, pair<int, int>>> _tx_fifo_switching_vcs;

  // <time, vc>
  deque<pair<int, int>> _tx_fifo_arbiter_vcs;

  // <time, <vc, <consumed_flits, ucie_flits_to_generate>>>>
  deque<pair<int, pair<int, pair<int, int>>>> _tx_packing_vcs;

  // <time, <vc, num_flits>>
  deque<pair<int, pair<int, int>>> _tx_unpacking_vcs;
  // <time, vc>
  deque<pair<int, int>> _tx_nopacking_vcs;

  // <time, input>
  deque<pair<int, int>> _tx_pipe_fifo_flits;

  // <time, input>
  deque<pair<int, int>> _tx_fdi_fifo_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _tx_adapter_packer_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _tx_crc_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _tx_retry_normal_flits;
  deque<pair<int, pair<int, Flit*>>> _tx_retry_replay_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _tx_rdi_flits;

  // <time, input>
  deque<pair<int, int>> _tx_retry_buffer_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _tx_serilazie_flits;

  // <time, <input, queue<Flit*>>>
  deque<pair<int, pair<int, queue<Flit*>>>> _rx_deserialize_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _rx_retry_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _rx_adapter_unpacker_flits;

  // <time, <input, Flit*>>
  deque<pair<int, pair<int, Flit*>>> _rx_fdi_flits;

  // <time, <int, num_flits>>
  deque<pair<int, pair<int, int>>> _rx_fifo_flits;

  // <time, <consumed_flits, target_flits_to_generate>>
  deque<pair<int, pair<int, int>>> _rx_packing_flits;

  // <time, target_flits_possible>
  deque<pair<int, int>> _rx_unpacking_flits;

  // <time, vc>
  deque<pair<int, int>> _rx_nopacking_flits;

  // TX VC Arbiter
  Arbiter* _tx_vc_arbiter;

  UCIEBufferState* _rx_fifo_state;
  UCIEBufferState* _tx_pipe_fifo_state;
  UCIEBufferState* _retry_buf_state;
  BufferState* _downstream_router_state;

  bool _ReceiveFlits() override;
  bool _ReceiveCredits() override;

  void _InternalStep() override;

  void _InputQueuing() override;
  void _OutputQueuing() override;

  void _SendFlits() override;
  void _SendCredits() override;

  // void _TX_FIFO_SwitchingEvaluate();

  void _TX_FIFO_ArbiterEvaluate();
  void _TX_FIFO_ArbiterUpdate();

  void _TX_FIFO_PackingEvaluate();
  void _TX_FIFO_PackingUpdate();

  void _TX_FIFO_UnpackingEvaluate();
  void _TX_FIFO_UnpackingUpdate();

  void _TX_FIFO_NoPackingEvaluate();
  void _TX_FIFO_NoPackingUpdate();

  void _TX_Pipe_FIFOEvaluate();
  void _TX_Pipe_FIFOUpdate();

  void _TX_FDI_FIFOEvaluate();
  void _TX_FDI_FIFOUpdate();

  void _TX_Adapter_PackerEvaluate();
  void _TX_Adapter_PackerUpdate();

  void _TX_CRCEvaluate();
  void _TX_CRCUpdate();

  void _TX_Retry_NormalEvaluate();
  void _TX_Retry_NormalUpdate();

  void _TX_RetryBufferEvaluate();
  void _TX_RetryBufferUpdate();

  void _TX_RDI_Evaluate();
  void _TX_RDI_Update();

  void _TX_SerializeEvaluate();
  void _TX_SerializeUpdate();

  void _RX_DeserializeEvaluate();
  void _RX_DeserializeUpdate();

  void _RX_RetryEvaluate();
  void _RX_RetryUpdate();

  void _RX_Adapter_UnpackerEvaluate();
  void _RX_Adapter_UnpackerUpdate();

  void _RX_FDI_Evaluate();
  void _RX_FDI_Update();

  void _RX_FIFOEvaluate();

  void _RX_PackingUpdate();
  void _RX_UnpackingUpdate();
  void _RX_NoPackingUpdate();
};

#endif