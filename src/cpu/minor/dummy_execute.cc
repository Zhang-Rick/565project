/*
 * Copyright (c) 2013-2014,2018-2019 ARM Limited
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

#include "cpu/minor/dummy_execute.hh"

#include "cpu/minor/cpu.hh"
#include "cpu/minor/decode.hh"
#include "cpu/minor/pipeline.hh"
#include "debug/Decode.hh"

namespace Minor
{

dummy_Execute::dummy_Execute(const std::string &name_,
    MinorCPU &cpu_,
    MinorCPUParams &params,
    Latch<ForwardInstData>::Output inp_,
    Latch<ForwardInstData>::Input out_,
    std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer):
    Named(name_),
    cpu(cpu_),
    inp(inp_),
    out(out_),
    nextStageReserve(next_stage_input_buffer),
    outputWidth(params.executeInputWidth),
    processMoreThanOneInput(params.decodeCycleInput),
    E1Info(params.numThreads),
    threadPriority(0)
 
{
    if (outputWidth < 1){}
     // fatal("%s: executeInputWidth must be >= 1 (%d)\n", name, outputWidth);
}


inline ThreadID
dummy_Execute::getScheduledThread()
{
    /* Select thread via policy. */
    std::vector<ThreadID> priority_list;

    switch (cpu.threadPolicy) {
      case Enums::SingleThreaded:
        priority_list.push_back(0);
        break;
      case Enums::RoundRobin:
        priority_list = cpu.roundRobinPriority(threadPriority);
        break;
      case Enums::Random:
        priority_list = cpu.randomPriority();
        break;
      default:
        panic("Unknown fetch policy");
    }

    for (auto tid : priority_list) {
        if (getInput(tid) && !E1Info[tid].blocked) {
            threadPriority = tid;
            return tid;
        }
    }

   return InvalidThreadID;
}


const ForwardInstData *
dummy_Execute::getInput(ThreadID tid)
{
    /* Get a line from the inputBuffer to work with */
    if (!inputBuffer[tid].empty()) {
        const ForwardInstData &head = inputBuffer[tid].front();

        return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
    } else {
        return NULL;
    }
}


void
dummy_Execute::popInput(ThreadID tid)
{
    if (!inputBuffer[tid].empty())
        inputBuffer[tid].pop();
}

void
dummy_Execute::evaluate()
{

    /* Push input onto appropriate input buffer */
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);

    ForwardInstData &insts_out = *out.inputWire;

    assert(insts_out.isBubble());

    for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
        E1Info[tid].blocked = !nextStageReserve[tid].canReserve();

    ThreadID tid = getScheduledThread();

    if (tid != InvalidThreadID) {
        DecodeThreadInfo &decode_info = E1Info[tid];
        const ForwardInstData *insts_in = getInput(tid);

        unsigned int output_index = 0;

        /* Pack instructions into the output while we can.  This may involve
 *          * using more than one input line */
        while (insts_in &&
           decode_info.inputIndex < insts_in->width() && /* Still more input */
           output_index < outputWidth /* Still more output to fill */)
        {
            MinorDynInstPtr inst = insts_in->insts[decode_info.inputIndex];

            if (inst->isBubble()) {
                /* Skip */
                decode_info.inputIndex++;
                decode_info.inMacroop = false;
            } else {
                StaticInstPtr static_inst = inst->staticInst;
                /* Static inst of a macro-op above the output_inst */
                StaticInstPtr parent_static_inst = NULL;
                MinorDynInstPtr output_inst = inst;

                if (inst->isFault()) {
                    DPRINTF(Decode, "Fault being passed: %d\n",
                        inst->fault->name());

                    decode_info.inputIndex++;
                    decode_info.inMacroop = false;
                } else {
                    /* Doesn't need decomposing, pass on instruction */
                    DPRINTF(Decode, "Passing on inst: %s inputIndex:"
                        " %d output_index: %d\n",
                        *output_inst, decode_info.inputIndex, output_index);

                    parent_static_inst = static_inst;

                    /* Step input */
                    decode_info.inputIndex++;
                    decode_info.inMacroop = false;
                }

                /* Set execSeqNum of output_inst */
                output_inst->id.execSeqNum = decode_info.execSeqNum;

                /* Step to next sequence number */
                decode_info.execSeqNum++;

                /* Correctly size the output before writing */
                if (output_index == 0) insts_out.resize(outputWidth);
                /* Push into output */
                insts_out.insts[output_index] = output_inst;
                output_index++;
            }

            /* Have we finished with the input? */
            if (decode_info.inputIndex == insts_in->width()) {
                /* If we have just been producing micro-ops, we *must* have
 *                  * got to the end of that for inputIndex to be pushed past
 *                                   * insts_in->width() */
                assert(!decode_info.inMacroop);
                popInput(tid);
                insts_in = NULL;

                if (processMoreThanOneInput) {
                    DPRINTF(Decode, "Wrapping\n");
                    insts_in = getInput(tid);
                }
            }
        }

        /* The rest of the output (if any) should already have been packed
 *          *  with bubble instructions by insts_out's initialisation
 *                   *
 *                            *  for (; output_index < outputWidth; output_index++)
 *                                     *      assert(insts_out.insts[output_index]->isBubble());
 *                                              */
    }

    /* If we generated output, reserve space for the result in the next stage
 *      *  and mark the stage as being active this cycle */
    if (!insts_out.isBubble()) {
        /* Note activity of following buffer */
        cpu.activityRecorder->activity();
        insts_out.threadId = tid;
        nextStageReserve[tid].reserve();
    }

    /* If we still have input to process and somewhere to put it,
 *      *  mark stage as active */
    for (ThreadID i = 0; i < cpu.numThreads; i++)
    {
        if (getInput(i) && nextStageReserve[i].canReserve()) {
            cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);
            break;
        }
    }

    /* Make sure the input (if any left) is pushed */
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].pushTail();




/*
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);

    ForwardInstData &insts_out = *out.inputWire;
    MinorDynInstPtr inst;

    ThreadID tid = getScheduledThread();

    if (tid != InvalidThreadID) {
         DecodeThreadInfo &decode_info = E1Info[tid];
         const ForwardInstData *insts_in = getInput(tid);


         MinorDynInstPtr inst = insts_in->insts[decode_info.inputIndex];
         

  	 inst->id.execSeqNum = decode_info.execSeqNum;        
         insts_out.resize(outputWidth);
         insts_out.insts[0] = inst;
	 popInput(tid);

}


         unsigned int output_index = 0;
        while (insts_in &&
           decode_info.inputIndex < insts_in->width() && 
           output_index < outputWidth)

        {
             MinorDynInstPtr inst = insts_in->insts[decode_info.inputIndex];

            if(inst->isBubble()){
            decode_info.inputIndex++;
            decode_info.inMacroop = false;
            }

            else{
            decode_info.inputIndex++;
            decode_info.inMacroop = false;
             
            inst->id.execSeqNum = decode_info.execSeqNum;
            
            decode_info.execSeqNum++;
            if (output_index == 0) insts_out.resize(outputWidth);
            
            insts_out.insts[output_index] = inst;
            output_index++;
            }

            if (decode_info.inputIndex == insts_in->width()) {
                assert(!decode_info.inMacroop);
                popInput(tid);
                insts_in = NULL;

                if (processMoreThanOneInput) {
                    DPRINTF(Decode, "Wrapping\n");
                    insts_in = getInput(tid);
                }
          }

    }
}

    if (!insts_out.isBubble()) {
        cpu.activityRecorder->activity();
        insts_out.threadId = tid;
        nextStageReserve[tid].reserve();
    }

    // ForwardInstData &insts_out = *out.inputWire;
    
    //assert(insts_out.isBubble());
    //DecodeThreadInfo &decode_info = E1Info[tid];
    //const ForwardInstData *insts_in = getInput(tid);
    //unsigned int output_index = 0;
    //insts_out.resize(outputWidth);
    //insts_out.insts[output_index] = insts_in;

    //cpu.activityRecorder->activity();
    //insts_out.threadId = tid;
    //nextStageReserve[tid].reserve();

    for (ThreadID i = 0; i < cpu.numThreads; i++)
    {
        if (getInput(i) && nextStageReserve[i].canReserve()) {
            cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);
            break;
        }
    }

    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].pushTail();

  // out.inputWire.threadId = tid;
  // nextStageReserve[tid].reserve();
*/
}

bool
dummy_Execute::isDrained()
{
    for (const auto &buffer : inputBuffer) {
        if (!buffer.empty())
            return false;
    }

    return (*inp.outputWire).isBubble();
}


}
