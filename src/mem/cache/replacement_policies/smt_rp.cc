/**
 * Copyright (c) 2018-2020 Inria
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

#include "mem/cache/replacement_policies/smt_rp.hh"

#include <cassert>
#include <memory>

#include "params/SMTRP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace replacement_policy
{

SMT::SMT(const Params &p)
  : Base(p)
{
}

void
SMT::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    std::static_pointer_cast<SMTReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
SMT::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<SMTReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
SMT::touchBit(const std::shared_ptr<ReplacementData>& replacement_data,
              const PacketPtr pkt) const
{
    if (pkt->isWrite()) {
        Addr touched_addr = pkt->getAddr();
        int address = int(touched_addr);
        int blk_addr = ((address >> 6) << 6);
        int offset = address & 0x3f;
        int subblock_num = (offset) / 0x10;
        // std :: cout << "blk_addr => " << blk_addr;
        std :: cout << "write_addr => " << touched_addr;
        // std :: cout << " , " << pkt->print();
        // std :: cout << std :: endl;
        // std :: cout << "touched_addr => " << touched_addr;
        // std :: cout << ", touched_block_area=> ";
        // std :: cout << std:: hex;
        // std :: cout << (int)std::static_pointer_cast<SMTReplData>
        //                 (replacement_data)->type;
        std :: cout << std :: endl;

        if (subblock_num >= 0 && subblock_num < 1)
        {

            std::static_pointer_cast<SMTReplData>
                (replacement_data)->type |= 0x1000;


            std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub0 = curTick();
            // std :: cout << "prevtime_sub0: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->prevtime_sub0;
            // std :: cout << " , curtime_sub0: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->curtime_sub0;

            uint64_t subtraction = ((std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->curtime_sub0) -
                                    (std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->prevtime_sub0));

            if (subtraction <= 0x2710){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat0 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat0 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat0 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat0 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat0 = 0x55;
            }
            std::static_pointer_cast<SMTReplData>
                (replacement_data)->prevtime_sub0 =
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub0;

            // std :: cout << " , heat0: ";
            // std :: cout << std::dec;
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->heat0;
            // std :: cout << std :: endl;


        }
        if (subblock_num >= 1 && subblock_num < 2)
        {
            std::static_pointer_cast<SMTReplData>
                (replacement_data)->type |= 0x100;


            std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub1 = curTick();
            // std :: cout << "prevtime_sub1: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->prevtime_sub1;
            // std :: cout << " , curtime_sub1: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->curtime_sub1;

            uint64_t subtraction = ((std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->curtime_sub1) -
                                    (std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->prevtime_sub1));

            if (subtraction <= 0x2710){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat1 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat1 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat1 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat1 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat1 = 0x55;
            }
            std::static_pointer_cast<SMTReplData>
                (replacement_data)->prevtime_sub1 =
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub1;

            // std :: cout << " , heat1: ";
            // std :: cout << std::dec;
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->heat1;
            // std :: cout << std :: endl;
        }
        if (subblock_num >= 2 && subblock_num < 3)
        {
            std::static_pointer_cast<SMTReplData>
                (replacement_data)->type |= 0x10;

            std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub2 = curTick();
            // std :: cout << "prevtime_sub2: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->prevtime_sub2;
            // std :: cout << " , curtime_sub2: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->curtime_sub2;

            uint64_t subtraction = ((std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->curtime_sub2) -
                                    (std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->prevtime_sub2));

            if (subtraction <= 0x2710){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat2 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat2 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat2 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat2 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat2 = 0x55;
            }
            std::static_pointer_cast<SMTReplData>
                (replacement_data)->prevtime_sub2 =
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub2;

            // std :: cout << " , heat2: ";
            // std :: cout << std::dec;
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->heat2;
            // std :: cout << std :: endl;
        }
        if (subblock_num >= 3)
        {
            std::static_pointer_cast<SMTReplData>
                (replacement_data)->type |= 0x1;

            std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub3 = curTick();
            // std :: cout << "prevtime_sub3: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->prevtime_sub3;
            // std :: cout << " , curtime_sub3: ";
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //                 (replacement_data)->curtime_sub3;

            uint64_t subtraction = ((std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->curtime_sub3) -
                                    (std::static_pointer_cast<SMTReplData>
                                    (replacement_data)->prevtime_sub3));
            if (subtraction <= 0x2710){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat3 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat3 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat3 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat3 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->heat3 = 0x55;
            }

            std::static_pointer_cast<SMTReplData>
                (replacement_data)->prevtime_sub3 =
                std::static_pointer_cast<SMTReplData>
                (replacement_data)->curtime_sub3;

            // std :: cout << " , heat3: ";
            // std :: cout << std::dec;
            // std :: cout << (int)std::static_pointer_cast<SMTReplData>
            //             (replacement_data)->heat3;
            // std :: cout << std :: endl;
        }
        // std :: cout << "touched_block_area_updated=> ";
        // std :: cout << std:: hex;
        // std :: cout << (int)std::static_pointer_cast<SMTReplData>
        //                 (replacement_data)->type;
        // std :: cout << std :: endl;
    }
}

uint64_t
SMT::getheat(const std::shared_ptr<ReplacementData>& replacement_data,
            const int i) const
{
    if (i==0){
        int heat0=std::static_pointer_cast<SMTReplData>(
            replacement_data)->heat0;
        return heat0;
    }
    if (i==1){
        int heat1=std::static_pointer_cast<SMTReplData>(
            replacement_data)->heat1;
        return heat1;
    }
    if (i==2){
        int heat2=std::static_pointer_cast<SMTReplData>(
            replacement_data)->heat2;
        return heat2;
    }
    else{
        int heat3=std::static_pointer_cast<SMTReplData>(
            replacement_data)->heat3;
        return heat3;
    }
}

void
SMT::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<SMTReplData>(
        replacement_data)->lastTouchTick = curTick();
}


ReplaceableEntry*
SMT::getVictim(const ReplacementCandidates& candidates,
                const uint64_t type) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    std::string pName = name();
    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    for (const auto& candidate : candidates) {
        if ((((type) & (std::static_pointer_cast<SMTReplData>
            (candidate->replacementData)->type)) == 0x0000))
        {
            victim = candidate;
        }
        // if (pName.compare("system.cpu.dcache.replacement_policy") == 0){
        //     std :: cout << "candidates: type: " ;
        //     std :: cout << std:: hex;
        //     std :: cout <<(int)std::static_pointer_cast<SMTReplData>
        //                 (candidate->replacementData)->type;
        //     std :: cout << std::endl;
        // }
    }
    // std :: cout << "accpkt: type: " << std:: hex << type << std::endl;
    // if (pName.compare("system.cpu.dcache.replacement_policy") == 0){
    //     std :: cout << "victim => type: ";
    //     std :: cout << std:: hex;
    //     std :: cout <<(int)std::static_pointer_cast<SMTReplData>
    //                 (victim->replacementData)->type;
    //     std :: cout << std :: endl;}
    // *type = std::static_pointer_cast<SMTReplData>
    // (victim->replacementData)->type;
    return victim;
}


std::shared_ptr<ReplacementData>
SMT::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new SMTReplData());
}

} // namespace replacement_policy
} // namespace gem5
