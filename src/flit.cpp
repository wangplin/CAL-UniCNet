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

/*flit.cpp
 *
 *flit struct is a flit, carries all the control signals that a flit needs
 *Add additional signals as necessary. Flits has no concept of length
 *it is a singluar object.
 *
 *When adding objects make sure to set a default value in this constructor
 */

#include "flit.hpp"
#include "thread_local_pool.hpp"

#include "booksim.hpp"

#include <cmath>
#include <mutex>
#include <iostream>

template class ThreadLocalPool<Flit>;

stack<Flit*> Flit::_all;
stack<Flit*> Flit::_free;
std::mutex Flit::_pool_mutex;

ostream& operator<<(ostream& os, const Flit& f) {
  os << "  Flit ID: " << f.id << " (" << &f << ")" << " Packet ID: " << f.pid
     << " Type: " << f.type << " Head: " << f.head << " Tail: " << f.tail
     << endl;
  os << "  Source: " << f.src << "  Dest: " << f.dest << " Intm: " << f.intm
     << endl;
  os << "  Creation time: " << f.ctime << " Injection time: " << f.itime
     << " Arrival time: " << f.atime << " Phase: " << f.ph << endl;
  os << "  VC: " << f.vc << endl;
  return os;
}

Flit::Flit() {
  Reset();
}

void Flit::Reset() {
  type = ANY_TYPE;
  vc = -1;
  cl = -1;
  head = false;
  tail = false;
  ctime = -1;
  itime = -1;
  atime = -1;
  id = -1;
  pid = -1;
  hops = 0;
  watch = false;
  record = false;
  intm = 0;
  src = -1;
  dest = -1;
  pri = 0;
  intm = -1;
  ph = -1;
  data = 0;

  // used for UniCNet
  ph_chiplet = Intra;
  src_router = -1;
  dest_router = -1;
  src_chiplet = -1;
  dest_chiplet = -1;
  intm_chiplet.clear();

  // used for interface
  fid = -1;
  psize = -1;
  fsize = -1;

  // used for ucie retry
  ack_nak_used = false;
  is_retry = false;
  seq_num = -1;
  is_nak = false;
  tx_acknak_flit_seq_num = -1;
}

Flit* Flit::De_Serialize(int new_fid, int cur_flit_size, bool is_head,
                         bool is_tail) {
  Flit temp_copy = *this;
  Flit* f = Flit::New();
  *f = temp_copy;

  f->src = this->src;
  f->src_router = this->src_router;
  f->src_chiplet = this->src_chiplet;

  f->fid = new_fid;
  f->head = is_head;
  f->tail = is_tail;
  f->fsize = cur_flit_size;

  if (f->head) {
    // head flit (1-flit packet or m-flits packet head)
    assert(this->head);
  } else {
    // m-flits packet: body or tail flit
    f->dest = -1;
    f->dest_router = -1;
  }

  return f;
}

Flit* Flit::New() {
  Flit* f;
  if (_free.empty()) {
    f = new Flit;
    _all.push(f);
  } else {
    f = _free.top();
    f->Reset();
    _free.pop();
  }
  return f;
}

void Flit::Free() {
  _free.push(this);
}

void Flit::FreeAll() {
  FlitPool::FreeAll();
  std::lock_guard<std::mutex> lock(_pool_mutex);
  while (!_all.empty()) {
    delete _all.top();
    _all.pop();
  }
}
