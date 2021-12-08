/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/pred/prophet_criticsold.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

prophet_criticsoldBP::prophet_criticsoldBP(const prophet_criticsoldBPParams *params)
    : BPredUnit(params),
      localPredictorSize(params->localPredictorSize),
      localHistoryTableSize(params->localHistoryTableSize),
      localCtrBits(params->localCtrBits),
      localPredictorSets(localPredictorSize / localCtrBits),
      localCtrs(localPredictorSets, SatCounter(localCtrBits)),
      indexMask(localPredictorSets - 1)
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    if (!isPowerOf2(localPredictorSets)) {
        fatal("Invalid number of local predictor sets! Check localCtrBits.\n");
    }

    DPRINTF(Fetch, "index mask: %#x\n", indexMask);

    DPRINTF(Fetch, "local predictor size: %i\n",
            localPredictorSize);

    DPRINTF(Fetch, "local counter bits: %i\n", localCtrBits);

    DPRINTF(Fetch, "instruction shift amount: %i\n",
            instShiftAmt);
    localHistoryTable.resize(localHistoryTableSize);
    for (int i = 0; i < localHistoryTableSize; ++i)
        localHistoryTable[i] = 0;
}

void
prophet_criticsoldBP::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    unsigned local_history_idx = getLocalIndex(branch_addr);
    //Update Global History to Not Taken (clear LSB)
    //globalHistory[tid] &= (historyRegisterMask & ~ULL(1));
    //Update Local History to Not Taken
    localHistoryTable[local_history_idx*2] =
       0;
    localHistoryTable[local_history_idx*2+1] =
       0;
}


bool
prophet_criticsoldBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    bool taken;
    unbranch = 0;
    //uint8_t takens; 
    unsigned local_predictor_idx = getLocalIndex(branch_addr);
    //bool futureBits[4];
    
    //futureBits[3] = localHistoryTable[local_predictor_idx*2+localHistoryTable[local_predictor_idx*2]];
    //futureBits[2] = localHistoryTable[local_predictor_idx*2+futureBits[3]];
    //futureBits[1] = localHistoryTable[local_predictor_idx*2+futureBits[2]];
    //futureBits[0] = localHistoryTable[local_predictor_idx*2+futureBits[2]];
    //futurebit = futureBits[3] * 8 + futureBits[2] * 4 + futureBits[1] * 2 + futureBits[0];
    //int *foo = std::find(std::begin(address_4outcome), std::end(address_4outcome), (local_predictor_idx<<4)+futurebit);

    /*
    if (foo != std::end(address_4outcome)) {
        critic_valid[std::distance(address_4outcome, foo)] = 1;
        taken = critic_outcome[std::distance(address_4outcome, foo)];
        return taken;
    }
    */
    DPRINTF(Fetch, "Looking up index %#x\n",
            local_predictor_idx);

    uint8_t counter_val = localCtrs[local_predictor_idx*2+localHistoryTable[local_predictor_idx*2]];
    //printf("prediction is %i.\n",(int)counter_val);
    DPRINTF(Fetch, "prediction is %i.\n",
            (int)counter_val);

    taken = getPrediction(counter_val);
    
    return taken;
}

void
prophet_criticsoldBP::update(ThreadID tid, Addr branch_addr, bool taken, void *bp_history,
                bool squashed, const StaticInstPtr & inst, Addr corrTarget)
{
    assert(bp_history == NULL);
    //unsigned local_predictor_idx;

    // No state to restore, and we do not update on the wrong
    // path.
    unsigned local_predictor_idx = getLocalIndex(branch_addr);
    
    if (squashed) {
        /*
        if (!unbranch) localHistoryTable[local_predictor_idx*2+localHistoryTable[local_predictor_idx*2]] = taken;
        
        int *foo = std::find(std::begin(address_4outcome), std::end(address_4outcome), (local_predictor_idx<<4)+futurebit);
        if (foo != std::end(address_4outcome)) {
            //critic_valid[std::distance(array, foo)] = 1;
            critic_outcome[std::distance(address_4outcome, foo)] = taken;
        } else {
            critic_valid[leastPointer] = 1;
            critic_outcome[leastPointer] = taken;
            address_4outcome[leastPointer] = (local_predictor_idx<<4)+futurebit; 
            leastPointer = (leastPointer+1)%256;
        }
        */
        return;
    }

    // Update the local predictor.
    local_predictor_idx = getLocalIndex(branch_addr);

    DPRINTF(Fetch, "Looking up index %#x\n", local_predictor_idx);
    //localHistoryTable[local_predictor_idx] = taken;
    if (taken) {
        DPRINTF(Fetch, "Branch updated as taken.\n");
        localCtrs[local_predictor_idx]++;
    } else {
        DPRINTF(Fetch, "Branch updated as not taken.\n");
        localCtrs[local_predictor_idx]--;
    }
}

inline
bool
prophet_criticsoldBP::getPrediction(uint8_t &count)
{
    // Get the MSB of the count
    return (count >> (localCtrBits - 1));
}

inline
unsigned
prophet_criticsoldBP::getLocalIndex(Addr &branch_addr)
{
    //printf("index %#x indexMask %lx\n",branch_addr, (branch_addr >> instShiftAmt) & indexMask, indexMask);
    return (branch_addr >> (instShiftAmt+1)) & indexMask;
}

void
prophet_criticsoldBP::uncondBranch(ThreadID tid, Addr pc, void *&bp_history)
{
    unbranch = 1;
}

prophet_criticsoldBP*
prophet_criticsoldBPParams::create()
{
    return new prophet_criticsoldBP(this);
}
