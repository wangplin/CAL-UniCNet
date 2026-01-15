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

#include "ucieinterface.hpp"

#include "buffer.hpp"
#include "allocator.hpp"
#include "arbiter.hpp"
#include "uciebufferstate.hpp"
#include "random_utils.hpp"
#include <cmath>

UCIEInterface::UCIEInterface(const Configuration& config, Module* parent,
                             const string& name, int id)
    : Interface(config, parent, name, id) {

  // TX Protocol Layer
  _ucie_flit_size = config.GetInt("ucie_flit_size");
  _tx_arbiter_delay = config.GetInt("ucie_tx_arbiter_delay");

  _tx_packing_delay = config.GetInt("ucie_tx_packing_delay");
  _tx_unpacking_delay = config.GetInt("ucie_tx_unpacking_delay");
  _tx_nopacking_delay = config.GetInt("ucie_tx_nopacking_delay");

  _tx_pipe_fifo_delay = config.GetInt("ucie_tx_pipe_fifo_delay");

  // TX Adapter Layer (Normal mode)
  _next_tx_seq_num = 0;

  _tx_fdi_fifo_delay = config.GetInt("ucie_tx_fdi_fifo_delay");
  _tx_adapter_packer_delay = config.GetInt("ucie_tx_adapter_packer_delay");
  _tx_crc_delay = config.GetInt("ucie_tx_crc_delay");

  // TX Adapter Layer (Replay mode)
  _tx_replay_delay = config.GetInt("ucie_tx_replay_delay");
  _tx_retry_buffer_delay = config.GetInt("ucie_tx_retry_buffer_delay");

  // TX PHY Layer
  _serdes_ratio = config.GetInt("ucie_serdes_ratio");
  assert(_ucie_flit_size % _serdes_ratio == 0 &&
         "serdes ratio must be a divisor of ucie flit size");
  _lane_width = config.GetFloat("ucie_lane_width");
  assert(_lane_width <= 1.0);
  assert(_serdes_ratio * _lane_width >= 1.0 &&
         "serdes ratio * lane width must be greater than or equal to 1.0");
  _partial_lane_width = 0.0;

  _tx_serdes_delay = config.GetInt("ucie_tx_serdes_delay");

  // RX PHY Layer
  _rx_serdes_delay = config.GetInt("ucie_rx_serdes_delay");

  // RX Adapter Layer
  _next_expected_rx_seq_num = 0;
  _error_rate = config.GetFloat("ucie_error_rate");
  assert(_error_rate >= 0.0 && _error_rate < 1.0 &&
         "error rate must be between 0.0 and 1.0");
  _replay_num = 0;

  _rx_retry_delay = config.GetInt("ucie_rx_retry_delay");
  _rx_adapter_unpacker_delay = config.GetInt("ucie_rx_adapter_unpacker_delay");
  _rx_fdi_delay = config.GetInt("ucie_rx_fdi_delay");

  // RX Protocol Layer
  _rx_packing_delay = config.GetInt("ucie_rx_packing_delay");
  _rx_unpacking_delay = config.GetInt("ucie_rx_unpacking_delay");
  _rx_nopacking_delay = config.GetInt("ucie_rx_nopacking_delay");

  // only one vc for the entire ucie controller except the tx_fifo
  _tx_pipe_fifo = new UCIEBuffer(config, 1, this, "tx_pipe_fifo", 1);
  _tx_fdi_fifo = new UCIEBuffer(config, 1, this, "tx_fdi_fifo", 1);
  _retry_buffer = new UCIERetryBuffer(config, 1, this, "retry_buffer", 1);
  _rx_fifo = new UCIEBuffer(config, 1, this, "rx_fifo", 1);

  _rx_fifo_state = new UCIEBufferState(config, this, "rx_fifo", 1, -1);
  _tx_pipe_fifo_state =
      new UCIEBufferState(config, this, "tx_pipe_fifo", 1, -1);
  _retry_buf_state = new UCIEBufferState(config, this, "retry_buffer", 1, -1);

  _tx_retry_state = TX_retry_state::main;
  _rx_retry_state = RX_retry_state::idle;
}

void UCIEInterface::SetFarstreamRouter(const Configuration& config,
                                       Router* farstream_router) {
  _farstream_router = farstream_router;

  _vcs = _farstream_router->NumVcs();

  _tx_fifo = new UCIEBuffer(config, 1, this, "tx_fifo", _vcs);

  // Create Round Robin arbiter for TX VC selection
  _tx_vc_arbiter = Arbiter::NewArbiter(this, "tx_vc_arb", "round_robin", _vcs);
}

void UCIEInterface::SetNearstreamConfig(Configuration* nearstream_config) {
  _nearstream_config = nearstream_config;

  _next_chiplet_flit_size = _nearstream_config->GetInt("chiplet_flit_size");

  _downstream_router_state =
      new BufferState(*_nearstream_config, this, "downstream_router");
}

UCIEInterface::~UCIEInterface() {
  delete _tx_pipe_fifo;
  delete _tx_fdi_fifo;
  delete _retry_buffer;
  delete _rx_fifo;

  delete _rx_fifo_state;
  delete _tx_pipe_fifo_state;
  delete _retry_buf_state;

  delete _tx_fifo;
  delete _tx_vc_arbiter;

  delete _downstream_router_state;
}

bool UCIEInterface::_ReceiveFlits() {
  bool activity = false;

  for (int input = 0; input < _inputs; ++input) {
    int receive_rate = _input_receive_rate[input];
    for (int i = 0; i < receive_rate; ++i) {
      Flit* const f = _input_channels[input]->Receive();
      if (f) {
        _in_queue_flits[input].push_back(f);
        if (_input_channels[input]->GetSourcePort() != -1) {
          assert(input == _tx_input);
        }
        activity = true;
      } else {
        break;
      }
    }
  }

  return activity;
}

bool UCIEInterface::_ReceiveCredits() {
  bool activity = false;
  for (int output = 0; output < _outputs; ++output) {
    Credit* const c = _output_credits[output]->Receive();
    if (c) {
      if (_output_credits[output]->GetSourcePort() != -1) {
        // credit from downstream router
        _proc_credits_router.push_back(
            make_pair(GetSimTime() + _credit_delay, make_pair(c, output)));

      } else {
        // credit from downstream interface
        _proc_credirs_interface.push_back(
            make_pair(GetSimTime() + _credit_delay, make_pair(c, output)));
      }
      activity = true;
    }
  }
  return activity;
}

void UCIEInterface::_InputQueuing() {
  for (map<int, deque<Flit*>>::iterator iter = _in_queue_flits.begin();
       iter != _in_queue_flits.end(); ++iter) {
    int const input = iter->first;
    assert((input >= 0) && (input < _inputs));
    deque<Flit*>& flitQueue = iter->second;
    if (flitQueue.empty()) {
      continue;
    }

    if (_input_channels[input]->GetSourcePort() != -1) {
      // from upstream router
      // (un)packing to ucie flit
      assert(input == _tx_input);

      UCIEBuffer* const cur_buf = _tx_fifo;

      assert(flitQueue.size() == 1);

      Flit* const f = flitQueue.front();
      assert(f);
      int const vc = f->vc;

      // add to corrsponding vc
      cur_buf->AddFlit(vc, f);

      flitQueue.pop_front();

      // Determine flit processing type
      enum FlitProcessType { UNPACKING, PACKING, DIRECT_SEND };
      FlitProcessType process_type;

      if (f->fsize > _ucie_flit_size) {
        process_type = UNPACKING;
      } else if (f->fsize < _ucie_flit_size) {
        process_type = PACKING;
      } else {
        process_type = DIRECT_SEND;
      }

      // Handle VC state
      VC::eVCState vc_state = cur_buf->GetState(vc);

      if (vc_state == VC::idle) {
        // Common handling for idle state
        assert(cur_buf->GetOccupancy(vc) == 1);
        assert(cur_buf->GetPackingState(vc) == VC::not_started);
        _tx_fifo_arbiter_vcs.push_back(make_pair(-1, vc));
        cur_buf->SetState(vc, VC::vc_alloc);

      } else if (vc_state == VC::active &&
                 cur_buf->GetPackingState(vc) == VC::not_started) {
        // Handle active VCs based on processing type
        switch (process_type) {
          case UNPACKING:
            _tx_unpacking_vcs.push_back(make_pair(-1, make_pair(vc, -1)));
            break;
          case PACKING:
            _tx_packing_vcs.push_back(
                make_pair(-1, make_pair(vc, make_pair(-1, -1))));
            break;
          case DIRECT_SEND:
            _tx_nopacking_vcs.push_back(make_pair(-1, vc));
            break;
        }
        cur_buf->SetPackingState(vc, VC::waiting);
      }

    } else {
      assert(_input_channels[input]->GetInterfaceSourcePort() != -1);

      assert(input == _rx_input);

      for (int i = 0; i < _serdes_ratio; i++) {
        if (!flitQueue.empty()) {
          Flit* f = flitQueue.front();
          assert(f);
          int const vc = f->vc;
          assert((vc >= 0) && (vc < _vcs));
          _rx_serdes_queue.push(f);

          flitQueue.pop_front();
        } else {
          // land width degradation is enabled
          break;
        }
      }

      // need to wait until _serdes_ratio flits are received
      if (_rx_serdes_queue.size() >= (size_t)_serdes_ratio) {
        queue<Flit*> flit_queue;
        for (int i = 0; i < _serdes_ratio; i++) {
          Flit* f = _rx_serdes_queue.front();
          assert(f);
          flit_queue.push(f);
          _rx_serdes_queue.pop();
        }
        _rx_deserialize_flits.push_back(
            make_pair(-1, make_pair(input, flit_queue)));
      }
    }
  }
  _in_queue_flits.clear();

  while (!_proc_credits_router.empty()) {
    pair<int, pair<Credit*, int>> const& item = _proc_credits_router.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }

    Credit* const c = item.second.first;
    assert(c);

    int const output = item.second.second;
    assert(output == _rx_input);

    _downstream_router_state->ProcessCredit(c);
    c->Free();

    _proc_credits_router.pop_front();
  }

  while (!_proc_credirs_interface.empty()) {
    pair<int, pair<Credit*, int>> const& item = _proc_credirs_interface.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }

    Credit* const c = item.second.first;
    assert(c);

    int const output = item.second.second;
    assert(output == _tx_input);

    _rx_fifo_state->ProcessCredit(c);
    c->Free();

    _proc_credirs_interface.pop_front();
  }
}

void UCIEInterface::_InternalStep() {
  if (!_active) {
    return;
  }

  _InputQueuing();

  bool activity =
      !_proc_credirs_interface.empty() || !_proc_credits_router.empty();

  // ----- TX Protocol Layer ----- //

  if (!_tx_fifo_arbiter_vcs.empty()) {
    _TX_FIFO_ArbiterEvaluate();
  }

  if (!_tx_packing_vcs.empty()) {
    _TX_FIFO_PackingEvaluate();
  }

  if (!_tx_unpacking_vcs.empty()) {
    _TX_FIFO_UnpackingEvaluate();
  }

  if (!_tx_nopacking_vcs.empty()) {
    _TX_FIFO_NoPackingEvaluate();
  }

  if (!_tx_pipe_fifo_flits.empty()) {
    _TX_Pipe_FIFOEvaluate();
  }

  // ----- TX Adapter Layer ----- //

  if (!_tx_fdi_fifo_flits.empty()) {
    _TX_FDI_FIFOEvaluate();
  }

  if (!_tx_adapter_packer_flits.empty()) {
    _TX_Adapter_PackerEvaluate();
  }

  if (!_tx_crc_flits.empty()) {
    _TX_CRCEvaluate();
  }

  if (!_tx_retry_normal_flits.empty() || !_tx_retry_replay_flits.empty()) {
    _TX_Retry_NormalEvaluate();
  }

  if (!_tx_retry_buffer_flits.empty()) {
    _TX_RetryBufferEvaluate();
  }

  if (!_tx_rdi_flits.empty()) {
    _TX_RDI_Evaluate();
  }

  // ----- TX PHY Layer ----- //

  if (!_tx_serilazie_flits.empty()) {
    _TX_SerializeEvaluate();
  }

  // ----- RX PHY Layer ----- //

  if (!_rx_deserialize_flits.empty()) {
    _RX_DeserializeEvaluate();
  }

  // ----- RX Adapter Layer ----- //

  if (!_rx_retry_flits.empty()) {
    _RX_RetryEvaluate();
  }

  if (!_rx_adapter_unpacker_flits.empty()) {
    _RX_Adapter_UnpackerEvaluate();
  }

  if (!_rx_fdi_flits.empty()) {
    _RX_FDI_Evaluate();
  }

  // ----- RX Protocol Layer ----- //

  if (!_rx_fifo_flits.empty()) {
    _RX_FIFOEvaluate();
  }

  if (!_tx_fifo_arbiter_vcs.empty()) {
    _TX_FIFO_ArbiterUpdate();
    activity = activity || !_tx_fifo_arbiter_vcs.empty();
  }

  if (!_tx_packing_vcs.empty()) {
    _TX_FIFO_PackingUpdate();
    activity = activity || !_tx_packing_vcs.empty();
  }

  if (!_tx_unpacking_vcs.empty()) {
    _TX_FIFO_UnpackingUpdate();
    activity = activity || !_tx_unpacking_vcs.empty();
  }

  if (!_tx_nopacking_vcs.empty()) {
    _TX_FIFO_NoPackingUpdate();
    activity = activity || !_tx_nopacking_vcs.empty();
  }

  if (!_tx_pipe_fifo_flits.empty()) {
    _TX_Pipe_FIFOUpdate();
    activity = activity || !_tx_pipe_fifo_flits.empty();
  }

  if (!_tx_fdi_fifo_flits.empty()) {
    _TX_FDI_FIFOUpdate();
    activity = activity || !_tx_fdi_fifo_flits.empty();
  }

  if (!_tx_adapter_packer_flits.empty()) {
    _TX_Adapter_PackerUpdate();
    activity = activity || !_tx_adapter_packer_flits.empty();
  }

  if (!_tx_crc_flits.empty()) {
    _TX_CRCUpdate();
    activity =
        activity || !_tx_crc_flits.empty() || !_tx_retry_normal_flits.empty();
  }

  if (!_tx_retry_replay_flits.empty()) {
    _TX_Retry_NormalUpdate();
    activity = activity || !_tx_retry_normal_flits.empty() ||
               !_tx_retry_replay_flits.empty() ||
               !_tx_retry_buffer_flits.empty();
  }

  if (!_tx_retry_buffer_flits.empty()) {
    _TX_RetryBufferUpdate();
    activity = activity || !_tx_retry_buffer_flits.empty() ||
               !_tx_adapter_packer_flits.empty();
  }

  if (!_tx_rdi_flits.empty()) {
    _TX_RDI_Update();
    activity = activity || !_tx_rdi_flits.empty();
  }

  if (!_tx_serilazie_flits.empty()) {
    _TX_SerializeUpdate();
    activity = activity || !_tx_serilazie_flits.empty();
  }

  if (!_rx_deserialize_flits.empty()) {
    _RX_DeserializeUpdate();
    activity = activity || !_rx_deserialize_flits.empty();
  }

  if (!_rx_retry_flits.empty()) {
    _RX_RetryUpdate();
    activity = activity || !_rx_retry_flits.empty() ||
               !_tx_retry_replay_flits.empty() ||
               !_tx_adapter_packer_flits.empty();
  }

  if (!_rx_adapter_unpacker_flits.empty()) {
    _RX_Adapter_UnpackerUpdate();
    activity = activity || !_rx_adapter_unpacker_flits.empty();
  }

  if (!_rx_fdi_flits.empty()) {
    _RX_FDI_Update();
    activity = activity || !_rx_fdi_flits.empty();
  }

  if (!_rx_packing_flits.empty()) {
    _RX_PackingUpdate();
    activity = activity || !_rx_packing_flits.empty();
  }

  if (!_rx_unpacking_flits.empty()) {
    _RX_UnpackingUpdate();
    activity = activity || !_rx_unpacking_flits.empty();
  }

  if (!_rx_nopacking_flits.empty()) {
    _RX_NoPackingUpdate();
    activity = activity || !_rx_nopacking_flits.empty();
  }

  _active = activity;

  _OutputQueuing();
}

void UCIEInterface::_TX_FIFO_ArbiterEvaluate() {
  assert(_tx_vc_arbiter);

  if (_tx_fifo->IsPacketInProgress()) {
    // only one packet in progress at a time
    return;
  }

  _tx_vc_arbiter->Clear();

  deque<pair<int, int>>::iterator iter = _tx_fifo_arbiter_vcs.begin();

  while (iter != _tx_fifo_arbiter_vcs.end()) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    int const vc = iter->second;
    assert((vc >= 0) && (vc < _vcs));

    Buffer const* const cur_buf = _tx_fifo;
    assert(cur_buf->GetState(vc) == VC::vc_alloc);

    Flit const* const f = cur_buf->FrontFlit(vc);
    assert(f);
    assert(f->vc == vc);
    assert(f->head);

    // the priority here is equal
    _tx_vc_arbiter->AddRequest(vc, vc, 0);

    ++iter;
  }

  if (_tx_arbiter_delay > 0) {
    return;
  }

  // _tx_arbiter_delay = 0, we update here

  int selected_vc = _tx_vc_arbiter->Arbitrate();

  if (selected_vc >= 0) {

    Buffer* const cur_buf = _tx_fifo;

    cur_buf->SetState(selected_vc, VC::active);

    // trigger the packet packing, unpacking or directly send

    Flit* const f = cur_buf->FrontFlit(selected_vc);
    assert(f->head);

    // we still base on the fsize to decide, the psize here is not involved
    if (f->fsize > _ucie_flit_size) {
      // unpacking
      _tx_unpacking_vcs.push_back(make_pair(-1, make_pair(selected_vc, -1)));
    } else if (f->fsize < _ucie_flit_size) {
      // packing
      _tx_packing_vcs.push_back(
          make_pair(-1, make_pair(selected_vc, make_pair(-1, -1))));
    } else {
      // directly send
      _tx_nopacking_vcs.push_back(make_pair(-1, selected_vc));
    }

    if (cur_buf->GetPackingState(selected_vc) == VC::not_started) {
      cur_buf->SetPackingState(selected_vc, VC::waiting);
    }

    // pop the corrsponding selected vc from _tx_fifo_arbiter_vcs
    bool found = false;
    deque<pair<int, int>>::iterator iter_arbiter = _tx_fifo_arbiter_vcs.begin();
    while (iter_arbiter != _tx_fifo_arbiter_vcs.end()) {
      if (iter_arbiter->second == selected_vc) {
        _tx_fifo_arbiter_vcs.erase(iter_arbiter);
        found = true;
        break;
      }
      ++iter_arbiter;
    }
    assert(found);

    // the vc has been selected, update the arbiter state
    _tx_vc_arbiter->UpdateState();
  }
}

void UCIEInterface::_TX_FIFO_ArbiterUpdate() {
  while (!_tx_fifo_arbiter_vcs.empty()) {
    pair<int, int> const& item = _tx_fifo_arbiter_vcs.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const vc = item.second;
    assert((vc >= 0) && (vc < _vcs));

    assert(false);
  }
}

void UCIEInterface::_TX_FIFO_PackingEvaluate() {

  deque<pair<int, pair<int, pair<int, int>>>>::iterator iter =
      _tx_packing_vcs.begin();
  while (iter != _tx_packing_vcs.end()) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    int const vc = iter->second.first;
    assert((vc >= 0) && (vc < _vcs));

    int const consumed_flits = iter->second.second.first;
    assert(consumed_flits == -1);

    int ucie_flits_to_generate = iter->second.second.second;
    assert(ucie_flits_to_generate == -1);

    UCIEBuffer* const cur_buf = _tx_fifo;
    assert(cur_buf->GetState(vc) == VC::active);
    assert(cur_buf->GetPackingState(vc) == VC::waiting);

    int bytes_sent = cur_buf->GetSizeSent(vc);
    assert(bytes_sent % _ucie_flit_size == 0);

    int bytes_accumulated = 0;
    int flits_to_consume = 0;
    bool has_tail = false;
    int psize = -1;

    Flit* f = nullptr;

    for (int idx = 1; idx <= cur_buf->GetOccupancy(vc); ++idx) {
      f = cur_buf->FrontNumberFlit(vc, idx);
      assert(f);

      if (psize < 0)
        psize = f->psize;

      int actual_bytes = 0;
      if (f->tail) {
        actual_bytes = psize - f->fid * f->fsize;
        assert(actual_bytes > 0 && actual_bytes <= f->fsize);
        has_tail = true;
      } else {
        actual_bytes = f->fsize;
      }

      bytes_accumulated += actual_bytes;
      flits_to_consume = idx;

      int total_bytes = bytes_sent + bytes_accumulated;

      if (has_tail) {
        // tail flit has been reached, we can calculate the remaining ucie flits
        int total_ucie_flits = ceil((double)psize / (double)_ucie_flit_size);
        ucie_flits_to_generate =
            total_ucie_flits - bytes_sent / _ucie_flit_size;
        assert(ucie_flits_to_generate > 0);
        break;
      } else {
        ucie_flits_to_generate =
            total_bytes / _ucie_flit_size - bytes_sent / _ucie_flit_size;
        if (ucie_flits_to_generate > 0)
          break;
      }
    }

    if (ucie_flits_to_generate > 0) {

      assert(flits_to_consume > 0);

      // 1. downstream interface _rx_fifo is aviliable for ucie_flits_to_generate flits
      // 2. _tx_pipe_fifo is aviliable for ucie_flits_to_generate flits

      if (_rx_fifo_state->Available() >= ucie_flits_to_generate &&
          _tx_pipe_fifo_state->Available() >= ucie_flits_to_generate) {

        if (_tx_packing_delay == 0) {
          // we should update here
          assert(false);
        } else {

          iter->second.second.first = flits_to_consume;
          iter->second.second.second = ucie_flits_to_generate;
          iter->first = GetSimTime() + _tx_packing_delay - 1;

          cur_buf->SetPackingState(vc, VC::in_progress);
          ++iter;
        }

      } else {
        // the _rx_fifo_state or _tx_pipe_fifo_state is not aviliable for ucie_flits_to_generate flits
        break;
      }

    } else {
      // the flits in _tx_fifo is not enough to be packed
      break;
    }
  }
}

void UCIEInterface::_TX_FIFO_PackingUpdate() {
  assert(_tx_packing_delay);

  while (!_tx_packing_vcs.empty()) {
    pair<int, pair<int, pair<int, int>>> const& item = _tx_packing_vcs.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const vc = item.second.first;
    assert((vc >= 0) && (vc < _vcs));

    int const flits_to_consume = item.second.second.first;
    assert(flits_to_consume > 0);

    int const ucie_flits_to_generate = item.second.second.second;
    assert(ucie_flits_to_generate > 0);

    UCIEBuffer* const cur_buf = _tx_fifo;
    assert(cur_buf->GetState(vc) == VC::active);
    assert(cur_buf->GetPackingState(vc) == VC::in_progress);
    assert(cur_buf->GetOccupancy(vc) >= flits_to_consume);

    Flit* front_f = cur_buf->FrontNumberFlit(vc, 1);
    Flit* back_f = cur_buf->FrontNumberFlit(vc, flits_to_consume);
    assert(front_f);
    assert(back_f);

    // generate the new ucie flits
    Flit* new_f = nullptr;
    for (int i = 0; i < ucie_flits_to_generate; ++i) {
      bool is_head = (_tx_fifo->GetFidSend(vc) == 0);
      bool is_tail = (back_f->tail && (i == ucie_flits_to_generate - 1));
      new_f = front_f->De_Serialize(cur_buf->GetFidSend(vc), _ucie_flit_size,
                                    is_head, is_tail);

      if (is_tail) {
        _tx_fifo->ResetFidSend(vc);
      } else {
        _tx_fifo->UpdateFidSend(vc);
      }

      _tx_pipe_fifo->AddUCIeFlit(new_f);

      if (_tx_pipe_fifo->GetPackingState(0) == VC::not_started) {
        _tx_pipe_fifo_flits.push_back(make_pair(-1, _tx_input));
        _tx_pipe_fifo->SetPackingState(0, VC::waiting);
      }

      _rx_fifo_state->SendingUCIeFlit(new_f);
      _tx_pipe_fifo_state->SendingUCIeFlit(new_f);
    }

    const int pid = front_f->pid;
    bool has_tail = false;
    for (int i = 1; i <= flits_to_consume; ++i) {
      Flit* pop_f = cur_buf->RemoveFlit(vc);
      assert(pop_f);
      assert(pop_f->pid == pid);
      if (pop_f->tail)
        has_tail = true;

      pop_f->Free();

      Credit* const c = Credit::New();
      c->vc.insert(vc);
      _credit_buffer[_tx_input].push(c);
    }

    if (has_tail) {
      cur_buf->SetSizeSent(vc, 0);

      if (cur_buf->Empty(vc)) {
        // another packet has not arrived yet
        cur_buf->SetState(vc, VC::idle);
        cur_buf->SetPackingState(vc, VC::not_started);
      } else {
        // trigger the _tx_fifo_arbiter_vcs
        _tx_fifo_arbiter_vcs.push_back(make_pair(-1, vc));
        cur_buf->SetState(vc, VC::vc_alloc);
        cur_buf->SetPackingState(vc, VC::not_started);
      }
    } else {
      cur_buf->SetSizeSent(vc, cur_buf->GetSizeSent(vc) +
                                   ucie_flits_to_generate * _ucie_flit_size);

      if (cur_buf->Empty(vc)) {
        // the following flit has not reach yet
        cur_buf->SetPackingState(vc, VC::not_started);
      } else {
        // the following flit has already arrived
        _tx_packing_vcs.push_back(
            make_pair(-1, make_pair(vc, make_pair(-1, -1))));
        cur_buf->SetPackingState(vc, VC::waiting);
      }
    }

    _tx_packing_vcs.pop_front();
  }
}

void UCIEInterface::_TX_FIFO_UnpackingEvaluate() {
  deque<pair<int, pair<int, int>>>::iterator iter = _tx_unpacking_vcs.begin();
  while (iter != _tx_unpacking_vcs.end()) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    int const vc = iter->second.first;
    assert((vc >= 0) && (vc < _vcs));

    int flitPossible = iter->second.second;
    assert(flitPossible == -1);

    UCIEBuffer* const cur_buf = _tx_fifo;
    assert(cur_buf->GetState(vc) == VC::active);
    assert(cur_buf->GetPackingState(vc) == VC::waiting);

    Flit* const f = cur_buf->FrontFlit(vc);
    assert(f);

    assert(f->fsize > _ucie_flit_size);

    int bytes_sent = cur_buf->GetSizeSent(vc);
    int ucie_flits_sent = bytes_sent / _ucie_flit_size;

    assert(bytes_sent % _ucie_flit_size == 0);

    int flit_index = 1;
    bool has_tail = false;
    int psize = -1;
    int bytes_accumulated = 0;
    int ucie_flits_possible = 0;

    while (flit_index <= cur_buf->GetOccupancy(vc)) {
      Flit* f = cur_buf->FrontNumberFlit(vc, flit_index);
      if (psize < 0)
        ;
      psize = f->psize;

      // caculate the real flit size
      int actual_bytes = 0;
      if (f->tail) {
        actual_bytes = psize - f->fid * f->fsize;
        assert(actual_bytes > 0);
        has_tail = true;
      } else {
        actual_bytes = f->fsize;
      }

      bytes_accumulated += actual_bytes;

      int total_bytes = bytes_sent + bytes_accumulated;

      if (has_tail) {
        // tail flit has been reached, we can calculate the remaining ucie flits
        int total_ucie_flits = ceil((double)psize / (double)_ucie_flit_size);
        ucie_flits_possible = total_ucie_flits - ucie_flits_sent;
        break;
      } else {
        // only generate the complete ucie flits
        ucie_flits_possible = total_bytes / _ucie_flit_size - ucie_flits_sent;
        assert(ucie_flits_possible >= 0);

        if (ucie_flits_possible > 0)
          break;
      }

      flit_index++;
    }

    assert(flit_index < 2);

    if (ucie_flits_possible > 0) {

      // 1. downstream interface _rx_fifo is aviliable for ucie_flits_possible flits
      // 2. _tx_pipe_fifo is aviliable for ucie_flits_possible flits

      if (_rx_fifo_state->Available() >= ucie_flits_possible &&
          _tx_pipe_fifo_state->Available() >= ucie_flits_possible) {

        if (_tx_unpacking_delay == 0) {
          // we should update here
          assert(false);
        } else {
          iter->second.second = ucie_flits_possible;
          iter->first = GetSimTime() + _tx_unpacking_delay - 1;

          cur_buf->SetPackingState(vc, VC::in_progress);

          ++iter;
        }

      } else {
        // the _rx_fifo_state or _tx_pipe_fifo_state is not aviliable for ucie_flits_possible flits
        break;
      }

    } else {
      assert(false);
    }
  }
}

void UCIEInterface::_TX_FIFO_UnpackingUpdate() {
  assert(_tx_unpacking_delay);

  while (!_tx_unpacking_vcs.empty()) {
    pair<int, pair<int, int>> const& item = _tx_unpacking_vcs.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const vc = item.second.first;
    assert((vc >= 0) && (vc < _vcs));

    int ucie_flits_possible = item.second.second;
    assert(ucie_flits_possible > 0);

    UCIEBuffer* const cur_buf = _tx_fifo;
    assert(!cur_buf->Empty(vc));
    assert(cur_buf->GetState(vc) == VC::active);
    assert(cur_buf->GetPackingState(vc) == VC::in_progress);

    Flit* const f = cur_buf->RemoveFlit(vc);

    for (int i = 0; i < ucie_flits_possible; ++i) {
      Flit* new_flit = nullptr;

      bool is_head = (f->head && (i == 0));
      bool is_tail = (f->tail && (i == ucie_flits_possible - 1));

      new_flit = f->De_Serialize(cur_buf->GetFidSend(vc), _ucie_flit_size,
                                 is_head, is_tail);
      assert(new_flit);

      // _fidSent++
      cur_buf->UpdateFidSend(vc);

      _tx_pipe_fifo->AddUCIeFlit(new_flit);

      _rx_fifo_state->SendingUCIeFlit(new_flit);
      _tx_pipe_fifo_state->SendingUCIeFlit(new_flit);

      if (_tx_pipe_fifo->GetPackingState(0) == VC::not_started) {
        _tx_pipe_fifo_flits.push_back(make_pair(-1, _tx_input));
        _tx_pipe_fifo->SetPackingState(0, VC::waiting);
      }

      if (is_tail) {
        cur_buf->ResetFidSend(vc);
        cur_buf->SetSizeSent(vc, 0);
      } else {
        cur_buf->SetSizeSent(vc, cur_buf->GetSizeSent(vc) + _ucie_flit_size);
      }
    }

    // send credit to upstream router
    if (_out_queue_credits.count(_tx_input) == 0) {
      _out_queue_credits.insert(make_pair(_tx_input, Credit::New()));
    }
    _out_queue_credits.find(_tx_input)->second->vc.insert(vc);

    if (f->tail) {
      // we need to restart the _tx_fifo_arbiter_vcs

      if (cur_buf->Empty(vc)) {
        // another packet has not arrived yet
        cur_buf->SetState(vc, VC::idle);
        cur_buf->SetPackingState(vc, VC::not_started);
      } else {
        // trigger the _tx_fifo_arbiter_vcs
        _tx_fifo_arbiter_vcs.push_back(make_pair(-1, vc));
        cur_buf->SetState(vc, VC::vc_alloc);
        cur_buf->SetPackingState(vc, VC::not_started);
      }

    } else {
      // the packet has not been sent yet

      if (cur_buf->Empty(vc)) {
        // the following flit has not reach yet
        cur_buf->SetPackingState(vc, VC::not_started);
      } else {
        // the following flit has already arrived
        _tx_unpacking_vcs.push_back(make_pair(-1, make_pair(vc, -1)));
        cur_buf->SetPackingState(vc, VC::waiting);
      }
    }

    // free the original flit here, since we have create new flits
    f->Free();

    _tx_unpacking_vcs.pop_front();
  }
}

void UCIEInterface::_TX_FIFO_NoPackingEvaluate() {
  deque<pair<int, int>>::iterator iter = _tx_nopacking_vcs.begin();
  while (iter != _tx_nopacking_vcs.end()) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    int const vc = iter->second;
    assert((vc >= 0) && (vc < _vcs));

    UCIEBuffer* const cur_buf = _tx_fifo;
    assert(cur_buf->GetState(vc) == VC::active);
    assert(cur_buf->GetPackingState(vc) == VC::waiting);

    Flit* const f = cur_buf->FrontFlit(vc);
    assert(f);

    int flit_size = f->fsize;
    assert(flit_size == _ucie_flit_size);

    if (!_rx_fifo_state->IsFull() && !_tx_pipe_fifo_state->IsFull()) {
      if (_tx_nopacking_delay == 0) {
        // we should update here

        Flit* const f = cur_buf->RemoveFlit(vc);
        assert(f);

        _tx_pipe_fifo->AddUCIeFlit(f);

        if (_tx_pipe_fifo->GetPackingState(0) == VC::not_started) {
          _tx_pipe_fifo_flits.push_back(make_pair(-1, _tx_input));
          _tx_pipe_fifo->SetPackingState(0, VC::waiting);
        }

        // send credit to upstream router
        if (_out_queue_credits.count(_tx_input) == 0) {
          _out_queue_credits.insert(make_pair(_tx_input, Credit::New()));
        }
        _out_queue_credits.find(_tx_input)->second->vc.insert(vc);

        iter = _tx_nopacking_vcs.erase(iter);

        if (f->tail) {
          // we need to restart the _tx_fifo_arbiter_vcs
          if (cur_buf->Empty(vc)) {
            // another packet has not arrived yet
            cur_buf->SetState(vc, VC::idle);
            cur_buf->SetPackingState(vc, VC::not_started);
          } else {
            // trigger the _tx_fifo_arbiter_vcs
            _tx_fifo_arbiter_vcs.push_back(make_pair(-1, vc));
            cur_buf->SetState(vc, VC::vc_alloc);
            cur_buf->SetPackingState(vc, VC::not_started);
          }
        } else {
          // the packet has not been sent yet

          if (cur_buf->Empty(vc)) {
            // the following flit has not reach yet
            cur_buf->SetPackingState(vc, VC::not_started);
          } else {
            // the following flit has already arrived
            _tx_nopacking_vcs.push_back(make_pair(-1, vc));
            cur_buf->SetPackingState(vc, VC::waiting);
          }
        }

        _rx_fifo_state->SendingUCIeFlit(f);
        _tx_pipe_fifo_state->SendingUCIeFlit(f);

        break;

      } else {
        iter->first = GetSimTime() + _tx_nopacking_delay - 1;

        _rx_fifo_state->SendingUCIeFlit(f);
        _tx_pipe_fifo_state->SendingUCIeFlit(f);

        cur_buf->SetPackingState(vc, VC::in_progress);

        ++iter;
        assert(false);
      }

    } else {
      // the _rx_fifo_state or _tx_pipe_fifo_state is not aviliable for ucie_flits_possible flits
      break;
    }
  }
}

void UCIEInterface::_TX_FIFO_NoPackingUpdate() {

  while (!_tx_nopacking_vcs.empty()) {
    pair<int, int> const& item = _tx_nopacking_vcs.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    assert(false);
  }
}

void UCIEInterface::_TX_Pipe_FIFOEvaluate() {
  deque<pair<int, int>>::iterator iter = _tx_pipe_fifo_flits.begin();
  while (iter != _tx_pipe_fifo_flits.end()) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    assert(_tx_pipe_fifo->GetPackingState(0) == VC::waiting);

    if (!_tx_fdi_fifo->Full()) {
      if (_tx_pipe_fifo_delay == 0) {
        // we should update here

        Flit* const f = _tx_pipe_fifo->RemoveUCIeFlit();
        assert(f);
        _tx_fdi_fifo->AddUCIeFlit(f);

        Credit* const c = Credit::New();
        c->vc.insert(0);
        _tx_pipe_fifo_state->ProcessCredit(c);
        c->Free();

        if (_tx_fdi_fifo->GetPackingState(0) == VC::not_started) {
          _tx_fdi_fifo_flits.push_back(make_pair(-1, _tx_input));
          _tx_fdi_fifo->SetPackingState(0, VC::waiting);
        }

        iter = _tx_pipe_fifo_flits.erase(iter);

        if (!_tx_pipe_fifo->Empty(0)) {
          _tx_pipe_fifo_flits.push_back(make_pair(-1, _tx_input));
          _tx_pipe_fifo->SetPackingState(0, VC::waiting);
        } else {
          _tx_pipe_fifo->SetPackingState(0, VC::not_started);
        }

        break;

      } else {
        iter->first = GetSimTime() + _tx_pipe_fifo_delay - 1;
        ++iter;
        assert(false);
      }

    } else {
      // Not ready, need to wait the ready signal from mainband
      break;
    }
  }
}

void UCIEInterface::_TX_Pipe_FIFOUpdate() {
  while (!_tx_pipe_fifo_flits.empty()) {
    pair<int, int> const& item = _tx_pipe_fifo_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const input = item.second;
    assert(input == _tx_input);

    assert(false);
  }
}

void UCIEInterface::_TX_FDI_FIFOEvaluate() {
  assert(_tx_fdi_fifo_delay);
  for (deque<pair<int, int>>::iterator iter = _tx_fdi_fifo_flits.begin();
       iter != _tx_fdi_fifo_flits.end(); ++iter) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    int const input = iter->second;
    assert(input == _tx_input);

    assert(!_tx_fdi_fifo->Empty(0));
    assert(_tx_fdi_fifo->GetPackingState(0) == VC::waiting);

    // it will stall under following conditions:
    // 1. Retry Buffer is full
    // 2. in the replay mode
    // 3. Flit packer is reading data from retry buffer to replay
    // 4. receive nak flit from RX

    if (!_retry_buf_state->IsFull() &&
        _tx_retry_state == TX_retry_state::state_min) {
      iter->first = GetSimTime() + _tx_fdi_fifo_delay - 1;
    } else {
      break;
    }
  }
}

void UCIEInterface::_TX_FDI_FIFOUpdate() {
  assert(_tx_fdi_fifo_delay);
  while (!_tx_fdi_fifo_flits.empty()) {
    pair<int, int> const& item = _tx_fdi_fifo_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    assert(!_retry_buf_state->IsFull());

    Flit* const f = _tx_fdi_fifo->RemoveUCIeFlit();
    assert(f);

    _retry_buf_state->SendingUCIeFlit(f);

    if (!_tx_fdi_fifo->Empty(0)) {
      _tx_fdi_fifo_flits.push_back(make_pair(-1, _tx_input));
      _tx_fdi_fifo->SetPackingState(0, VC::waiting);
    } else {
      _tx_fdi_fifo->SetPackingState(0, VC::not_started);
    }

    // trigger _tx_adapter_packer_flits

    _tx_adapter_packer_flits.push_back(make_pair(-1, make_pair(_tx_input, f)));

    _tx_fdi_fifo_flits.pop_front();
  }
}

void UCIEInterface::_TX_Adapter_PackerEvaluate() {
  assert(_tx_adapter_packer_delay);
  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _tx_adapter_packer_flits.begin();
       iter != _tx_adapter_packer_flits.end(); ++iter) {
    int const time = iter->first;
    if (time > 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _tx_input);

    // this process is to add the necessary header for link layer retransmission
    // we only model the timing here, the actual content is not involved

    iter->first = GetSimTime() + _tx_adapter_packer_delay - 1;
  }
}

void UCIEInterface::_TX_Adapter_PackerUpdate() {
  assert(_tx_adapter_packer_delay);
  while (!_tx_adapter_packer_flits.empty()) {
    pair<int, pair<int, Flit*>> const& item = _tx_adapter_packer_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    Flit* const f = item.second.second;
    assert(f);

    // trigger the tx crc
    _tx_crc_flits.push_back(make_pair(-1, make_pair(_tx_input, f)));
    _tx_adapter_packer_flits.pop_front();
  }
}

void UCIEInterface::_TX_CRCEvaluate() {
  assert(_tx_crc_delay);
  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _tx_crc_flits.begin();
       iter != _tx_crc_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _tx_input);

    // this process is to adding crc checksum to the flit
    // we only model the timing here, the actual content is not involved

    iter->first = GetSimTime() + _tx_crc_delay - 1;
  }
}

void UCIEInterface::_TX_CRCUpdate() {
  assert(_tx_crc_delay);
  while (!_tx_crc_flits.empty()) {
    pair<int, pair<int, Flit*>> const& item = _tx_crc_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    Flit* const f = item.second.second;
    assert(f);

    // send to tx retry control

    _tx_retry_normal_flits.push_back(make_pair(-1, make_pair(_tx_input, f)));

    _tx_crc_flits.pop_front();
  }
}

void UCIEInterface::_TX_Retry_NormalEvaluate() {
  // no cycle consumption for normal sending

  while (!_tx_retry_normal_flits.empty()) {
    pair<int, pair<int, Flit*>> const& item = _tx_retry_normal_flits.front();
    int const time = item.first;
    if (time > 0) {
      break;
    }

    int const input = item.second.first;
    assert(input == _tx_input);

    Flit* const f = item.second.second;
    assert(f);

    // ack/nak flit is not needed to be set seq num and add to retry buffer
    if (!f->ack_nak_used) {
      if (!f->is_retry) {
        f->SetSeqNum(_next_tx_seq_num);
        _next_tx_seq_num++;

        // make a copy of flit in the retry buffer
        Flit* f_copy = Flit::New();
        *f_copy = *f;
        _retry_buffer->AddUCIeFlit(f_copy);

        assert(_tx_retry_state == TX_retry_state::main ||
               _tx_retry_state == TX_retry_state::replay_pending);
      }

    } else {
      // ack/nak flit
    }

    // send to tx rdi

    _tx_rdi_flits.push_back(make_pair(-1, make_pair(_tx_input, f)));
    _tx_retry_normal_flits.pop_front();
  }

  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _tx_retry_replay_flits.begin();
       iter != _tx_retry_replay_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _tx_input);
    Flit* f = iter->second.second;
    assert(f->ack_nak_used);

    iter->first = GetSimTime() + _tx_replay_delay - 1;
  }
}

void UCIEInterface::_TX_Retry_NormalUpdate() {

  while (!_tx_retry_replay_flits.empty()) {
    pair<int, pair<int, Flit*>>& item = _tx_retry_replay_flits.front();
    int& time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const input = item.second.first;
    assert(input == _tx_input);
    Flit* f = item.second.second;
    assert(f->ack_nak_used);

    // update the retry FSM

    if (f->is_nak) {
      // nak flit
      if (_tx_retry_state == TX_retry_state::main) {
        _tx_retry_state = TX_retry_state::replay_pending;
      }

      if (_tx_retry_state == TX_retry_state::replay_pending) {
        bool pipeline_empty = _tx_adapter_packer_flits.empty() &&
                              _tx_crc_flits.empty() &&
                              _tx_retry_normal_flits.empty();

        if (pipeline_empty) {
          // only the pipeline is empty, we can switch to replay mode
          _tx_retry_state = TX_retry_state::replay;

          // clear the correct flit in the retry buffer has been cleared
          assert(_retry_buffer->ClearFlitForNak(f->tx_acknak_flit_seq_num));

          _tx_retry_replay_flits.pop_front();

          // trigger the _tx_retry_buffer_flits
          _tx_retry_buffer_flits.push_back(make_pair(-1, _tx_input));
        } else {
          // do nothing, wait for the pipeline to be empty
          item.first++;
        }
      } else {
        assert(false);
      }

    } else {
      // ack flit

      assert(_tx_retry_state == TX_retry_state::main);

      // clear the retry buffer according to the seq num of the ack flit

      int retry_seq_num = -2;
      while (retry_seq_num < f->tx_acknak_flit_seq_num) {
        Flit* f_retry = _retry_buffer->RemoveUCIeFlit();
        assert(f_retry);
        retry_seq_num = f_retry->seq_num;
        assert(retry_seq_num <= f->tx_acknak_flit_seq_num);
        f_retry->Free();

        Credit* const c = Credit::New();
        c->vc.insert(0);
        _retry_buf_state->ProcessCredit(c);
        c->Free();
      }

      f->Free();

      _tx_retry_replay_flits.pop_front();
    }
  }
}

void UCIEInterface::_TX_RetryBufferEvaluate() {
  assert(_tx_retry_buffer_flits.size() == 1);
  for (deque<pair<int, int>>::iterator iter = _tx_retry_buffer_flits.begin();
       iter != _tx_retry_buffer_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second;
    assert(input == _tx_input);
    assert(_tx_retry_state == TX_retry_state::replay);

    iter->first = GetSimTime() + _tx_retry_buffer_delay - 1;
  }
}

void UCIEInterface::_TX_RetryBufferUpdate() {
  assert(_tx_retry_buffer_flits.size() == 1);
  while (!_tx_retry_buffer_flits.empty()) {
    pair<int, int>& item = _tx_retry_buffer_flits.front();
    int& time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const input = item.second;
    assert(input == _tx_input);
    assert(_tx_retry_state == TX_retry_state::replay);

    Flit* f_retry = _retry_buffer->ReplayFlitForNak();
    assert(f_retry);

    // make a copy of the flit
    Flit* f_copy = Flit::New();
    *f_copy = *f_retry;
    assert(f_copy->is_retry);

    if (_retry_buffer->GetReplayState() == UCIERetryBuffer::ReplayState::idle) {
      // all flits have been sent
      _tx_retry_buffer_flits.pop_front();
      assert(_tx_retry_buffer_flits.empty());
      _tx_retry_state = TX_retry_state::main;
    } else {
      // there are more flits to be resend
      assert(_retry_buffer->GetReplayState() ==
             UCIERetryBuffer::ReplayState::replay);

      // next cycle will keep resending flit
      item.first++;
    }

    // send to tx adapter packer
    _tx_adapter_packer_flits.push_back(make_pair(-1, make_pair(input, f_copy)));
  }
}

void UCIEInterface::_TX_RDI_Evaluate() {
  deque<pair<int, pair<int, Flit*>>>::iterator iter = _tx_rdi_flits.begin();
  while (iter != _tx_rdi_flits.end()) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _tx_input);
    Flit* f = iter->second.second;
    assert(f);

    // send to TX PHY layer
    _tx_serilazie_flits.push_back(make_pair(-1, make_pair(input, f)));
    iter = _tx_rdi_flits.erase(iter);
  }
}

void UCIEInterface::_TX_RDI_Update() {
  while (!_tx_rdi_flits.empty()) {
    pair<int, pair<int, Flit*>>& item = _tx_rdi_flits.front();
    int& time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }

    assert(false);
  }
}

void UCIEInterface::_TX_SerializeEvaluate() {
  assert(_tx_serdes_delay);
  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _tx_serilazie_flits.begin();
       iter != _tx_serilazie_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _tx_input);
    Flit* f = iter->second.second;
    assert(f);
    assert(f->seq_num != -1 || f->ack_nak_used);

    iter->first = GetSimTime() + _tx_serdes_delay - 1;
  }
}

void UCIEInterface::_TX_SerializeUpdate() {
  while (!_tx_serilazie_flits.empty()) {
    pair<int, pair<int, Flit*>>& item = _tx_serilazie_flits.front();
    int& time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    Flit* const f = item.second.second;
    assert(f);

    // serilazie the one flit to _serdes_ratio flits
    assert(f->fsize % _serdes_ratio == 0 &&
           "flit size must be a multiple of serdes ratio for serilazie");

    int new_fid = f->fid * _serdes_ratio;
    int new_fsize = f->fsize / _serdes_ratio;
    for (int i = 0; i < _serdes_ratio; ++i) {
      Flit* new_flit =
          f->De_Serialize(new_fid + i, new_fsize, (f->head && i == 0),
                          (f->tail && i == _serdes_ratio - 1));

      _output_buffer[_tx_input].push(new_flit);
    }

    f->Free();
    _tx_serilazie_flits.pop_front();
  }
}

void UCIEInterface::_RX_DeserializeEvaluate() {
  for (deque<pair<int, pair<int, queue<Flit*>>>>::iterator iter =
           _rx_deserialize_flits.begin();
       iter != _rx_deserialize_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _rx_input);
    queue<Flit*> flit_queue = iter->second.second;
    assert(flit_queue.size() == (size_t)_serdes_ratio);

    iter->first = GetSimTime() + _rx_serdes_delay - 1;
  }
}

void UCIEInterface::_RX_DeserializeUpdate() {
  while (!_rx_deserialize_flits.empty()) {
    pair<int, pair<int, queue<Flit*>>>& item = _rx_deserialize_flits.front();
    int& time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    queue<Flit*>& flit_queue = item.second.second;

    Flit* head_flit = nullptr;
    Flit* last_flit = nullptr;

    int pid = flit_queue.front()->pid;
    int seq_num = flit_queue.front()->seq_num;

    for (int i = 0; i < _serdes_ratio; i++) {
      last_flit = flit_queue.front();
      flit_queue.pop();

      assert(last_flit->pid == pid && last_flit->seq_num == seq_num);

      if (last_flit->head) {
        assert(head_flit == nullptr);
        assert(i == 0);
        head_flit = last_flit;
      }

      if (i != _serdes_ratio - 1 && (last_flit != head_flit)) {
        // only free the non-head flits but also the last flit
        last_flit->Free();
      }
    }

    // the current flit size should be _ucie_flit_size for both ack/nak flit or normal flit

    Flit* new_flit = nullptr;

    int new_fid = -1;

    if (head_flit != nullptr) {
      assert(head_flit->fid % _serdes_ratio == 0);
      // assert(!head_flit->tail);
      new_flit =
          head_flit->De_Serialize(0, _ucie_flit_size, true, last_flit->tail);
      assert(new_flit->head && new_flit->fsize == _ucie_flit_size);
      if (head_flit != last_flit) {
        last_flit->Free();
      }
      head_flit->Free();
    } else {
      assert((last_flit->fid + 1) % _serdes_ratio == 0);
      new_fid = ((last_flit->fid + 1) / _serdes_ratio) - 1;
      assert(new_fid >= 1);
      new_flit = last_flit->De_Serialize(new_fid, _ucie_flit_size, false,
                                         last_flit->tail);
      last_flit->Free();
    }

    if (new_flit) {
      // send to rx retry
      _rx_retry_flits.push_back(make_pair(-1, make_pair(_rx_input, new_flit)));
    } else {
      assert(false);
    }

    _rx_deserialize_flits.pop_front();
  }
}

void UCIEInterface::_RX_RetryEvaluate() {
  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _rx_retry_flits.begin();
       iter != _rx_retry_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _rx_input);
    Flit* f = iter->second.second;
    assert(f);

    iter->first = GetSimTime() + _rx_retry_delay - 1;
  }
}

void UCIEInterface::_RX_RetryUpdate() {
  while (!_rx_retry_flits.empty()) {
    pair<int, pair<int, Flit*>>& item = _rx_retry_flits.front();
    int& time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    Flit* f = item.second.second;
    assert(f);

    // 1. deal with the received ack/nak flit

    if (f->ack_nak_used) {
      // deadling with the ack/nak flit is in the same cycle with normal flit in TX retry control

      _tx_retry_replay_flits.push_back(make_pair(-1, make_pair(_tx_input, f)));
      _rx_retry_flits.pop_front();
      continue;
    }

    // 2. check the received normal flit

    if (RandomFloat() < _error_rate) {

      if (_rx_retry_state == RX_retry_state::idle) {
        // the flit is error

        assert(f->seq_num == _next_expected_rx_seq_num);

        _rx_retry_state = RX_retry_state::nak_scheduled;
        assert(_replay_num == 0);
        _replay_num++;

        // send nak flit

        Flit* nak_flit = Flit::New();
        nak_flit->SetIsNak(true, _ucie_flit_size,
                           _next_expected_rx_seq_num - 1);

        _tx_adapter_packer_flits.push_back(
            make_pair(-1, make_pair(_tx_input, nak_flit)));

        _rx_retry_flits.pop_front();
        f->Free();
        continue;

      } else if (_rx_retry_state == RX_retry_state::nak_scheduled) {
        // the nak flit has been sent, we need to wait the retry flit

        if (f->seq_num == _next_expected_rx_seq_num) {
          // the retry flit is error again
          // we do not limit the replay num here

          assert(_replay_num != 0);
          _replay_num++;

          // send nak flit
          Flit* nak_flit = Flit::New();
          nak_flit->SetIsNak(true, _ucie_flit_size,
                             _next_expected_rx_seq_num - 1);

          _tx_adapter_packer_flits.push_back(
              make_pair(-1, make_pair(_tx_input, nak_flit)));

          _rx_retry_flits.pop_front();
          f->Free();
          continue;
        } else {
          // latter flit should be dropped

          _rx_retry_flits.pop_front();
          f->Free();
          continue;
        }
      }
    }

    // 3. the received flit is correct

    if (f->seq_num == _next_expected_rx_seq_num) {
      _next_expected_rx_seq_num++;

      if (f->is_retry && _rx_retry_state == RX_retry_state::nak_scheduled) {
        // the retry flit is correct, can switch to idle state
        _rx_retry_state = RX_retry_state::idle;
        assert(_replay_num != 0);
        _replay_num = 0;
      }

      // send ack flit to tx packer

      Flit* ack_flit = Flit::New();
      ack_flit->SetIsNak(false, _ucie_flit_size, _next_expected_rx_seq_num - 1);

      _tx_adapter_packer_flits.push_back(
          make_pair(-1, make_pair(_tx_input, ack_flit)));

      // send original flit to rx unpacker
      f->ReSetSeqNum();
      f->is_retry = false;
      assert(_replay_num == 0);

      _rx_adapter_unpacker_flits.push_back(
          make_pair(-1, make_pair(_rx_input, f)));

      _rx_retry_flits.pop_front();

    } else {
      // even the received flit is correct, while the retry flit is not received yet
      // the retry flit will be ignored

      assert(_rx_retry_state == RX_retry_state::nak_scheduled);
      _rx_retry_flits.pop_front();
      f->Free();
    }
  }
}

void UCIEInterface::_RX_Adapter_UnpackerEvaluate() {
  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _rx_adapter_unpacker_flits.begin();
       iter != _rx_adapter_unpacker_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _rx_input);
    Flit* const f = iter->second.second;
    assert(f);

    iter->first = GetSimTime() + _rx_adapter_unpacker_delay - 1;
  }
}

void UCIEInterface::_RX_Adapter_UnpackerUpdate() {
  while (!_rx_adapter_unpacker_flits.empty()) {
    pair<int, pair<int, Flit*>> const& item =
        _rx_adapter_unpacker_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const input = item.second.first;
    assert(input == _rx_input);
    Flit* const f = item.second.second;
    assert(f);

    // send to rx fdi
    _rx_fdi_flits.push_back(make_pair(-1, make_pair(input, f)));

    _rx_adapter_unpacker_flits.pop_front();
  }
}

void UCIEInterface::_RX_FDI_Evaluate() {
  assert(_rx_fdi_delay);
  for (deque<pair<int, pair<int, Flit*>>>::iterator iter =
           _rx_fdi_flits.begin();
       iter != _rx_fdi_flits.end(); ++iter) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _rx_input);
    Flit* const f = iter->second.second;
    assert(f);

    assert(!_rx_fifo->Full());

    iter->first = GetSimTime() + _rx_fdi_delay - 1;
  }
}

void UCIEInterface::_RX_FDI_Update() {
  while (!_rx_fdi_flits.empty()) {
    pair<int, pair<int, Flit*>> const& item = _rx_fdi_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    Flit* const f = item.second.second;
    assert(f);

    assert(!_rx_fifo->Full());

    _rx_fifo->AddUCIeFlit(f);

    if (_rx_fifo->GetPackingState(0) == VC::not_started) {
      _rx_fifo_flits.push_back(make_pair(-1, make_pair(_rx_input, -1)));
      _rx_fifo->SetPackingState(0, VC::waiting);
    }

    _rx_fdi_flits.pop_front();
  }
}

void UCIEInterface::_RX_FIFOEvaluate() {
  deque<pair<int, pair<int, int>>>::iterator iter = _rx_fifo_flits.begin();
  while (iter != _rx_fifo_flits.end()) {
    int const time = iter->first;
    if (time >= 0) {
      break;
    }

    int const input = iter->second.first;
    assert(input == _rx_input);
    assert(_rx_fifo->GetPackingState(0) == VC::waiting);

    Flit* f = _rx_fifo->FrontFlit(0);
    assert(f);

    assert(f->fsize == _ucie_flit_size);

    // we should caculate the num of flits to be packed or unpacked
    // it will also stall if _downstream_router_state is full

    if (f->fsize > _next_chiplet_flit_size) {
      // unpacking

      int bytes_sent = _rx_fifo->GetSizeSent(0);
      int target_flits_sent = bytes_sent / _next_chiplet_flit_size;

      assert(bytes_sent % _next_chiplet_flit_size == 0);

      int flit_index = 1;
      bool has_tail = false;
      int psize = -1;
      int vc = -1;
      int bytes_accumulated = 0;
      int target_flits_possible = 0;

      while (flit_index <= _rx_fifo->GetOccupancy(0)) {
        Flit* f = _rx_fifo->FrontNumberFlit(0, flit_index);
        if (psize < 0) {
          psize = f->psize;
          vc = f->vc;
        }

        // caculate the real flit size
        int actual_bytes = 0;
        if (f->tail) {
          actual_bytes = psize - f->fid * f->fsize;
          assert(actual_bytes > 0);
          has_tail = true;
        } else {
          actual_bytes = f->fsize;
        }

        bytes_accumulated += actual_bytes;

        int total_bytes = bytes_sent + bytes_accumulated;

        if (has_tail) {
          int total_target_flits =
              ceil(double(psize) / double(_next_chiplet_flit_size));
          target_flits_possible = total_target_flits - target_flits_sent;
          break;
        } else {
          target_flits_possible =
              total_bytes / _next_chiplet_flit_size - target_flits_sent;
          assert(target_flits_possible >= 0);
          if (target_flits_possible > 0)
            break;
        }

        flit_index++;
      }

      assert(flit_index < 2);

      if (target_flits_possible > 0) {

        if (_downstream_router_state->AvailableFor(vc) >=
            target_flits_possible) {

          int time = GetSimTime() + _rx_unpacking_delay;
          _rx_unpacking_flits.push_back(make_pair(time, target_flits_possible));

          _rx_fifo->SetPackingState(0, VC::in_progress);
          iter = _rx_fifo_flits.erase(iter);

        } else {
          // the downstream router state is not enough, we need to stall
          break;
        }

      } else {
        assert(false);
      }

    } else if (f->fsize < _next_chiplet_flit_size) {
      // packing

      int bytes_sent = _rx_fifo->GetSizeSent(0);

      int bytes_accumulated = 0;
      int flits_to_consume = 0;
      bool has_tail = false;
      int psize = -1;
      int vc = -1;

      int target_flits_to_generate = 0;

      for (int idx = 1; idx <= _rx_fifo->GetOccupancy(0); idx++) {
        Flit* f = _rx_fifo->FrontNumberFlit(0, idx);
        assert(f);

        if (psize < 0) {
          psize = f->psize;
          vc = f->vc;
        }

        int actual_bytes;
        if (f->tail) {
          actual_bytes = psize - f->fid * f->fsize;
          assert(actual_bytes > 0 && actual_bytes <= f->fsize);
          has_tail = true;
        } else {
          actual_bytes = f->fsize;  // = ucie_flit_size
        }

        bytes_accumulated += actual_bytes;
        flits_to_consume = idx;

        int total_bytes = bytes_accumulated + bytes_sent;

        if (has_tail) {
          int total_target_flits =
              ceil(double(psize) / double(_next_chiplet_flit_size));
          target_flits_to_generate =
              total_target_flits - bytes_sent / _next_chiplet_flit_size;
          assert(target_flits_to_generate > 0);
          break;
        } else {
          target_flits_to_generate = total_bytes / _next_chiplet_flit_size -
                                     bytes_sent / _next_chiplet_flit_size;
          if (target_flits_to_generate > 0)
            break;
        }
      }

      if (target_flits_to_generate > 0) {

        assert(flits_to_consume > 0);

        if (_downstream_router_state->AvailableFor(vc) >=
            target_flits_to_generate) {

          int time = GetSimTime() + _rx_packing_delay;
          _rx_packing_flits.push_back(make_pair(
              time, make_pair(flits_to_consume, target_flits_to_generate)));

          _rx_fifo->SetPackingState(0, VC::in_progress);

          iter = _rx_fifo_flits.erase(iter);

        } else {
          // the downstream router state is not enough, we need to stall
          break;
        }

      } else {
        // the flits in _rx_fifo is not enough to be packed
        break;
      }

    } else {
      // directly send

      Flit* f = _rx_fifo->FrontFlit(0);
      int vc = f->vc;

      if (!_downstream_router_state->IsFullFor(vc)) {

        int time = GetSimTime() + _rx_nopacking_delay;
        _rx_nopacking_flits.push_back(make_pair(time, vc));

        _rx_fifo->SetPackingState(0, VC::in_progress);
        iter = _rx_fifo_flits.erase(iter);

      } else {
        // the downstream router state is not enough, we need to stall
        break;
      }
    }
  }
}

void UCIEInterface::_RX_PackingUpdate() {
  assert(_rx_packing_delay);

  while (!_rx_packing_flits.empty()) {
    pair<int, pair<int, int>> const& item = _rx_packing_flits.front();
    int const time = item.first;
    assert(time > 0);
    if (GetSimTime() < time) {
      break;
    }
    assert(GetSimTime() == time);

    assert(_rx_fifo->GetPackingState(0) == VC::in_progress);

    int const consumed_flits = item.second.first;
    assert(consumed_flits > 0);
    int const target_flits_to_generate = item.second.second;
    assert(target_flits_to_generate > 0);

    assert(_rx_fifo->GetPackingState(0) == VC::in_progress);
    assert(_rx_fifo->GetOccupancy(0) >= consumed_flits);

    Flit* front_f = _rx_fifo->FrontFlit(0);
    Flit* back_f = _rx_fifo->FrontNumberFlit(0, consumed_flits);
    assert(front_f);
    assert(back_f);

    Flit* new_f = nullptr;
    for (int i = 0; i < target_flits_to_generate; i++) {
      bool is_tail = (back_f->tail && i == target_flits_to_generate - 1);
      bool is_head = (_rx_fifo->GetFidSend(0) == 0);

      new_f = front_f->De_Serialize(_rx_fifo->GetFidSend(0),
                                    _next_chiplet_flit_size, is_head, is_tail);

      if (is_tail) {
        _rx_fifo->ResetFidSend(0);
        _rx_fifo->SetSizeSent(0, 0);
      } else {
        _rx_fifo->UpdateFidSend(0);
        _rx_fifo->SetSizeSent(
            0, _rx_fifo->GetSizeSent(0) + _next_chiplet_flit_size);
      }

      if (is_head) {
        _downstream_router_state->TakeBuffer(new_f->vc, 1);
      }

      _downstream_router_state->SendingFlit(new_f);

      _output_buffer[_rx_input].push(new_f);
    }

    const int pid = front_f->pid;
    for (int i = 0; i < consumed_flits; i++) {
      Flit* pop_flit = _rx_fifo->RemoveUCIeFlit();
      assert(pop_flit);
      assert(pop_flit->pid == pid);
      pop_flit->Free();

      Credit* const c = Credit::New();
      c->vc.insert(0);
      _credit_buffer[_rx_input].push(c);
    }

    if (_rx_fifo->Empty(0)) {
      _rx_fifo->SetPackingState(0, VC::not_started);
    } else {
      _rx_fifo_flits.push_back(make_pair(-1, make_pair(_rx_input, -1)));
      _rx_fifo->SetPackingState(0, VC::waiting);
    }

    _rx_packing_flits.pop_front();
  }
}

void UCIEInterface::_RX_UnpackingUpdate() {
  while (!_rx_unpacking_flits.empty()) {
    pair<int, int> const& item = _rx_unpacking_flits.front();
    int const time = item.first;
    assert(time > 0);
    if (GetSimTime() < time) {
      break;
    }
    assert(GetSimTime() == time);

    assert(_rx_fifo->GetPackingState(0) == VC::in_progress);

    int const target_flits_possible = item.second;
    assert(target_flits_possible > 0);

    Flit* const f = _rx_fifo->RemoveUCIeFlit();

    // send credit to upstream interface
    if (_out_queue_credits.count(_rx_input) == 0) {
      _out_queue_credits.insert(make_pair(_rx_input, Credit::New()));
    }
    _out_queue_credits.find(_rx_input)->second->vc.insert(0);

    for (int i = 0; i < target_flits_possible; i++) {
      Flit* new_flit = nullptr;
      bool is_head = (f->head && i == 0);
      bool is_tail = (f->tail && i == target_flits_possible - 1);

      new_flit = f->De_Serialize(_rx_fifo->GetFidSend(0),
                                 _next_chiplet_flit_size, is_head, is_tail);

      if (is_head) {
        _downstream_router_state->TakeBuffer(f->vc, 1);
      }

      _downstream_router_state->SendingFlit(new_flit);

      _rx_fifo->UpdateFidSend(0);

      _output_buffer[_rx_input].push(new_flit);

      if (is_tail) {
        _rx_fifo->ResetFidSend(0);
      }
    }

    if (f->tail) {
      _rx_fifo->SetSizeSent(0, 0);
    } else {
      _rx_fifo->SetSizeSent(
          0, _rx_fifo->GetSizeSent(0) +
                 _next_chiplet_flit_size * target_flits_possible);
    }

    if (_rx_fifo->Empty(0)) {
      _rx_fifo->SetPackingState(0, VC::not_started);
    } else {
      _rx_fifo_flits.push_back(make_pair(-1, make_pair(_rx_input, -1)));
      _rx_fifo->SetPackingState(0, VC::waiting);
    }

    f->Free();

    _rx_unpacking_flits.pop_front();
  }
}

void UCIEInterface::_RX_NoPackingUpdate() {
  while (!_rx_nopacking_flits.empty()) {
    pair<int, int> const& item = _rx_nopacking_flits.front();
    int const time = item.first;
    if ((time < 0) || (GetSimTime() < time)) {
      break;
    }
    assert(GetSimTime() == time);

    int const vc = item.second;
    assert(vc >= 0 && vc < _vcs);

    assert(_rx_fifo->GetPackingState(0) == VC::in_progress);

    Flit* const f = _rx_fifo->RemoveUCIeFlit();
    assert(f);

    if (f->head)
      _downstream_router_state->TakeBuffer(vc, 1);

    _downstream_router_state->SendingFlit(f);

    _output_buffer[_rx_input].push(f);

    if (_out_queue_credits.count(_rx_input) == 0) {
      _out_queue_credits.insert(make_pair(_rx_input, Credit::New()));
    }
    _out_queue_credits.find(_rx_input)->second->vc.insert(0);

    if (_rx_fifo->Empty(0)) {
      _rx_fifo->SetPackingState(0, VC::not_started);
    } else {
      _rx_fifo_flits.push_back(make_pair(-1, make_pair(_rx_input, -1)));
      _rx_fifo->SetPackingState(0, VC::waiting);
    }

    _rx_nopacking_flits.pop_front();
  }
}

void UCIEInterface::_OutputQueuing() {
  for (map<int, Credit*>::const_iterator iter = _out_queue_credits.begin();
       iter != _out_queue_credits.end(); ++iter) {
    int const input = iter->first;
    assert((input >= 0) && (input < _inputs));

    Credit* const c = iter->second;
    assert(c);

    _credit_buffer[input].push(c);
  }
  _out_queue_credits.clear();
}

void UCIEInterface::_SendFlits() {
  for (int output = 0; output < _outputs; ++output) {
    double send_rate = _output_send_rate[output];

    if (!_output_buffer[output].empty()) {

      if (_output_channels[output]->GetSinkPort() != -1) {
        // to downstream router
        assert(send_rate == 1);

        Flit* const f = _output_buffer[output].front();
        assert(f);
        _output_buffer[output].pop();
        _output_channels[output]->Send(f);

      } else {
        // to downstream interface

        _partial_lane_width += send_rate;
        while (_partial_lane_width >= 1.0 && !_output_buffer[output].empty()) {
          Flit* const f = _output_buffer[output].front();
          assert(f);
          _output_buffer[output].pop();
          _output_channels[output]->Send(f);
          _partial_lane_width -= 1.0;
        }
      }
    }
  }
}

void UCIEInterface::_SendCredits() {
  for (int input = 0; input < _inputs; ++input) {
    if (!_credit_buffer[input].empty()) {
      Credit* const c = _credit_buffer[input].front();
      assert(c);
      _credit_buffer[input].pop();
      _input_credits[input]->Send(c);

      //   if (_input_channels[input]->GetSourcePort() == -1) {
      //     // to upstream interface
      //   } else {
      //     // to upstream router
      //   }
    }
  }
}

void UCIEInterface::AddInputChannel(FlitChannel* channel,
                                    CreditChannel* backchannel) {
  Interface::AddInputChannel(channel, backchannel);
  if (channel->GetSourcePort() == -1) {
    // from upstream interface
    _input_receive_rate[_input_channels.size() - 1] = _serdes_ratio;
    _rx_input = _input_channels.size() - 1;
  } else {
    // from upstream router
    _input_receive_rate[_input_channels.size() - 1] = 1;
    _tx_input = _input_channels.size() - 1;
  }
}

void UCIEInterface::AddOutputChannel(FlitChannel* channel,
                                     CreditChannel* backchannel) {
  Interface::AddOutputChannel(channel, backchannel);
  if (channel->GetInterfaceSinkPort() != -1) {
    // to downstream interface
    _output_send_rate[_output_channels.size() - 1] =
        _serdes_ratio * _lane_width;
  } else {
    // to downstream router
    _output_send_rate[_output_channels.size() - 1] = 1;
  }
}
