// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _FLIT_HPP_
#define _FLIT_HPP_

#include <iostream>
#include <stack>
#include <list>
#include <mutex>

#include "booksim.hpp"
#include "outputset.hpp"
#include "thread_local_pool.hpp"

class Flit {

 public:
  const static int NUM_FLIT_TYPES = 5;
  enum FlitType {
    READ_REQUEST = 0,
    READ_REPLY = 1,
    WRITE_REQUEST = 2,
    WRITE_REPLY = 3,
    ANY_TYPE = 4
  };
  FlitType type;

  int vc;

  int cl;

  bool head;
  bool tail;

  int ctime;
  int itime;
  int atime;

  int id;
  int pid;

  bool record;

  int src;
  int dest;

  int pri;

  int hops;
  bool watch;
  int subnetwork;

  // intermediate destination (if any)
  mutable int intm;

  // phase in multi-phase algorithms
  mutable int ph;

  // Fields for arbitrary data
  void* data;

  // Lookahead route info
  OutputSet la_route_set;

  // used for UniCNet

  enum PhaseType {
    Intra = 0,
    Inbound = 1,
    Outbound = 2,
  };

  mutable PhaseType ph_chiplet;

  int src_router;
  int dest_router;

  int src_chiplet;
  int dest_chiplet;

  // intermediate router for modular routing (UniCNet)
  // <src_boundary, intermediate_boundary> --> ... --> <intermediate_boundary, dest_boundary>
  mutable list<pair<int, int>> intm_chiplet;

  // used for interface
  mutable int fid;    // flit id in packet
  mutable int psize;  // packet size (bytes)
  mutable int fsize;  // flit size (bytes)

  // used for ucie retry
  bool ack_nak_used;
  bool is_retry;
  int seq_num;
  bool is_nak;
  int tx_acknak_flit_seq_num;

  void Reset();

  Flit* De_Serialize(int new_fid, int cur_flit_size, bool is_head,
                     bool is_tail);

  inline void SetSeqNum(int seq_num) {
    assert(this->seq_num == -1);
    this->seq_num = seq_num;
  }

  inline void ReSetSeqNum() {
    assert(seq_num != -1);
    seq_num = -1;
  }
  inline void SetIsNak(bool is_nak, int ack_nak_flit_size, int seq_num) {
    SetAckNakUsed(true, ack_nak_flit_size);
    tx_acknak_flit_seq_num = seq_num;
    this->is_nak = is_nak;
  }

  inline void SetAckNakUsed(bool used, int ack_nak_flit_size) {
    head = true;
    tail = true;
    vc = 0;
    cl = 0;

    fid = 0;
    fsize = ack_nak_flit_size;

    ack_nak_used = used;
  }

  static Flit* New();
  void Free();
  static void FreeAll();

 private:
  friend class ThreadLocalPool<Flit>;

  Flit();
  ~Flit() {}

  static stack<Flit*> _all;
  static stack<Flit*> _free;
  static std::mutex _pool_mutex;

  using FlitPool = ThreadLocalPool<Flit>;
};

ostream& operator<<(ostream& os, const Flit& f);

#endif
