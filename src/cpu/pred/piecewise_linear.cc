/*
 * Copyright (c) 2011, 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#include "cpu/pred/piecewise_linear.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"

Piecewise_linear::Piecewise_linear(const Piecewise_linearParams *params)
    : BPredUnit(params),
      n_row_length_of_W(params->n_row_length_of_W),
      m_column_length_of_W(params->m_column_length_of_W),
      h(params->h)

{
    W_array_of_weights.resize(n_row_length_of_W);
    for(int i = 0; i < n_row_length_of_W; i++){
        W_array_of_weights[i].resize(m_column_length_of_W);
        for(int j = 0; j < m_column_length_of_W; j++){
            W_array_of_weights[i][j].resize(h+1);
            for(int k = 0; k < h+1; k++){
                W_array_of_weights[i][j][k] = 0;
            }
        }
    }

    GHR.resize(h);
    for(int i = 0; i<h; i++)
        GHR[i] = false;

    GA.resize(h);
    for(int i = 0; i<h; i++)
        GA[i] = 0;

    theta = (2.14*(h + 1)) + 20.58;

}

inline
unsigned
Piecewise_linear::getAddress(Addr &branch_addr)
{
    return (branch_addr >> instShiftAmt) & (n_row_length_of_W - 1);
}


inline
unsigned
Piecewise_linear::calcLocHistIdx(Addr &branch_addr)
{
    // Get low order bits after removing instruction offset.
    return (branch_addr >> instShiftAmt) & (localHistoryTableSize - 1);
}

inline
void
Piecewise_linear::updateGlobalHistTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1) | 1;
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
Piecewise_linear::updateGlobalHistNotTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1);
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
Piecewise_linear::updateLocalHistTaken(unsigned local_history_idx)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1) | 1;
}

inline
void
Piecewise_linear::updateLocalHistNotTaken(unsigned local_history_idx)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1);
}


void
Piecewise_linear::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
	GHR[0] = false; 	
}

bool
Piecewise_linear::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
	bool prediction;
	int output;
	unsigned calculated_address;
	
	calculated_address = getAddress(branch_addr);
	
	output = W_array_of_weights[calculated_address][0][0];

	for(int i = 0; i < h; i++){
		int j = GA[i] & (m_column_length_of_W - 1);
		if(GHR[i] == true)
			output = output + W_array_of_weights[calculated_address][j][i+1];
		else
			output = output - W_array_of_weights[calculated_address][j][i+1];
	}

	prediction = output >= 0;

        bphistory_piecewiselinear *history = new bphistory_piecewiselinear;
        history->output = output;
        history->GA.resize(h);
        history->GHR.resize(h);
        for(int i = 0; i < h ; i++){
                history->GA[i] = GA[i];
                history->GHR[i] = GHR[i];
        }
        bp_history = (void *)history;



	for(int i = 0; i < h-1 ; i++){
		GA[i+1] = GA[i];
		GHR[i+1] = GHR[i]; 
	}
	GA[0] = calculated_address;
	GHR[0] = prediction;
	
	return prediction;
}

void
Piecewise_linear::uncondBranch(ThreadID tid, Addr pc, void * &bp_history)
{

        unsigned calculated_address;
        calculated_address = getAddress(pc);

        bphistory_piecewiselinear *history = new bphistory_piecewiselinear;
        history->output = theta;
        history->GA.resize(h);
        history->GHR.resize(h);
        for(int i = 0; i < h ; i++){
                history->GA[i] = GA[i];
                history->GHR[i] = GHR[i];
        }
        bp_history = (void *)history;

        for(int i = 0; i < h-1 ; i++){
                GA[i+1] = GA[i];
                GHR[i+1] = GHR[i];
        }
        GA[0] = calculated_address;
        GHR[0] = true;
	

}

void
Piecewise_linear::update(ThreadID tid, Addr branch_addr, bool taken,
                     void *bp_history, bool squashed,
                     const StaticInstPtr & inst, Addr corrTarget)
{
	unsigned calculated_address;
        calculated_address = getAddress(branch_addr);

	bphistory_piecewiselinear *history = (bphistory_piecewiselinear *)bp_history;

	if(history == NULL)
		fatal("Null Error\n\n");

	if(squashed){
	        for(int i = 0; i < h-1; i++){
        	        GA[i+1] = history-> GA[i];
                	GHR[i+1] = history->GHR[i];
        	}
	        GA[0] = calculated_address;
        	GHR[0] = taken;
	}

	if(abs(history->output)<theta || squashed){
		if(taken){
			if(W_array_of_weights[calculated_address][0][0] < 127)
			W_array_of_weights[calculated_address][0][0] += 1;
		}
		else{
			if(W_array_of_weights[calculated_address][0][0] > -128)
			W_array_of_weights[calculated_address][0][0] -= 1;
		}
		for(int i = 0; i < h; i++){
			int j = GA[i] & (m_column_length_of_W - 1);

			if(GHR[i] == taken){
                        if(W_array_of_weights[calculated_address][j][i+1] < 127)
                        W_array_of_weights[calculated_address][j][i+1] += 1;
			}
			else{
                        if(W_array_of_weights[calculated_address][j][i+1] > -128)
                        W_array_of_weights[calculated_address][j][i+1] -= 1;
			}
		}
	}

}

void
Piecewise_linear::squash(ThreadID tid, void *bp_history)
{

}

Piecewise_linear*
Piecewise_linearParams::create()
{
    return new Piecewise_linear(this);
}

#ifdef DEBUG
int
Piecewise_linear::BPHistory::newCount = 0;
#endif
