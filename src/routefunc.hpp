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

#ifndef _ROUTEFUNC_HPP_
#define _ROUTEFUNC_HPP_

#include "flit.hpp"
#include "router.hpp"
#include "outputset.hpp"
#include "config_utils.hpp"

typedef void (*tRoutingFunction)(const Router*, const Flit*, int in_channel,
                                 OutputSet*, bool);

void InitializeRoutingMap(const Configuration& config);

extern map<string, tRoutingFunction> gRoutingFunctionMap;

extern int gNumVCs;
extern int gReadReqBeginVC, gReadReqEndVC;
extern int gWriteReqBeginVC, gWriteReqEndVC;
extern int gReadReplyBeginVC, gReadReplyEndVC;
extern int gWriteReplyBeginVC, gWriteReplyEndVC;

// ============================================================
// UniCNet Chiplet VC Configuration Structure
// ============================================================
struct ChipletVC {
  int chiplet_id;
  int NumVCs;
  int ReadReqBeginVC;
  int ReadReqEndVC;
  int WriteReqBeginVC;
  int WriteReqEndVC;
  int ReadReplyBeginVC;
  int ReadReplyEndVC;
  int WriteReplyBeginVC;
  int WriteReplyEndVC;

  // Constructor with default values (inline)
  ChipletVC()
      : chiplet_id(-1),
        NumVCs(0),
        ReadReqBeginVC(0),
        ReadReqEndVC(0),
        WriteReqBeginVC(0),
        WriteReqEndVC(0),
        ReadReplyBeginVC(0),
        ReadReplyEndVC(0),
        WriteReplyBeginVC(0),
        WriteReplyEndVC(0) {}
};

// Global vector storing VC configuration for each chiplet and interposer
// Index: 0 to chiplet_num-1 for chiplets, chiplet_num for interposer
extern vector<ChipletVC> gChipletVCConfigs;

// Function declarations
void InitializeRoutingMap(const Configuration& config);
void InitChipletVCConfig(const Configuration& config, int chiplet_id);

// Helper functions to access chiplet VC configurations
int GetChipletNumVCs(int chiplet_id);

#endif
