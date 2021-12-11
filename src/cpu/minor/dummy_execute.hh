/*
 * Copyright (c) 2013-2014 ARM Limited
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

/**
 * @file
 *
 *  All the fun of executing instructions from Decode and sending branch/new
 *  instruction stream info. to Fetch1.
 */

#ifndef __CPU_MINOR_D_EXECUTE_HH__
#define __CPU_MINOR_D_EXECUTE_HH__

#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/func_unit.hh"
#include "cpu/minor/lsq.hh"
#include "cpu/minor/pipe_data.hh"
#include "cpu/minor/scoreboard.hh"
#include "cpu/minor/dyn_inst.hh"

namespace Minor
{

/** Execute stage.  Everything apart from fetching and decoding instructions.
 *  The LSQ lives here too. */
class dummy_Execute : public Named
{
  protected:
    /** Pointer back to the containing CPU */
    MinorCPU &cpu;

    /** Input port carrying instructions from Decode */
    Latch<ForwardInstData>::Output inp;

    /** Input port carrying stream changes to execute */
    Latch<ForwardInstData>::Input out;

    std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;
    unsigned int outputWidth;
    
    bool processMoreThanOneInput;

  public: /* Public for Pipeline to be able to pass it to Decode */
    std::vector<InputBuffer<ForwardInstData>> inputBuffer;

  protected:
    const ForwardInstData *getInput(ThreadID tid);
    void popInput(ThreadID tid);
 
    ThreadID getScheduledThread();

    enum DrainState
    {
        NotDraining, /* Not draining, possibly running */
        DrainCurrentInst, /* Draining to end of inst/macroop */
        DrainHaltFetch, /* Halting Fetch after completing current inst */
        DrainAllInsts /* Discarding all remaining insts */
    };


  protected:
    struct DecodeThreadInfo {
        DecodeThreadInfo() :
            inputIndex(0),
            inMacroop(false),
            execSeqNum(InstId::firstExecSeqNum),
            blocked(false)
        { }

        DecodeThreadInfo(const DecodeThreadInfo& other) :
            inputIndex(other.inputIndex),
            inMacroop(other.inMacroop),
            execSeqNum(other.execSeqNum),
            blocked(other.blocked)
        { }

        unsigned int inputIndex;

        bool inMacroop;
        TheISA::PCState microopPC;

        InstSeqNum execSeqNum;

        bool blocked;
    };

    std::vector<DecodeThreadInfo> E1Info;
    ThreadID threadPriority;



  public:
    dummy_Execute(const std::string &name_,
        MinorCPU &cpu_,
        MinorCPUParams &params,
        Latch<ForwardInstData>::Output inp_,
        Latch<ForwardInstData>::Input out_,
        std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer);


  public:
    void evaluate();
    bool isDrained();

};

}

#endif /* __CPU_MINOR_D_EXECUTE_HH__ */
