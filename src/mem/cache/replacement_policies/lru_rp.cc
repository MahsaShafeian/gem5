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

#include "mem/cache/replacement_policies/lru_rp.hh"

#include <cassert>
#include <memory>

#include "params/LRURP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace replacement_policy
{

LRU::LRU(const Params &p)
  : Base(p)
{
}

void
LRU::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}

void
LRU::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
LRU::touchBit(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt) const
{
    std::string pName = name();
    if ((pkt->isWrite())&&
    (pName.compare("system.cpu.dcache.replacement_policy") == 0))
    {
        Addr touched_addr = pkt->getAddr();
        int address = int(touched_addr);
        // int blk_addr = ((address > 6) << 6);
        int offset = address & 0x3f;
        int subblock_num = (offset) / 0x10;

        if (subblock_num >= 0 && subblock_num < 1)
        {
            std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub0 = curTick();

            uint64_t subtraction = ((std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->curtime_sub0) -
                                    (std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->prevtime_sub0));

            if (subtraction <= 0x2710){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat0 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat0 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat0 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat0 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat0 = 0x55;
            }
            std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub0 =
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub0;
        }
        if (subblock_num >= 1 && subblock_num < 2)
        {

            std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub1 = curTick();

            uint64_t subtraction = ((std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->curtime_sub1) -
                                    (std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->prevtime_sub1));

            if (subtraction <= 0x2710){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat1 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat1 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat1 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat1 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat1 = 0x55;
            }
            std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub1 =
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub1;

        }
        if (subblock_num >= 2 && subblock_num < 3)
        {

            std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub2 = curTick();

            uint64_t subtraction = ((std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->curtime_sub2) -
                                    (std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->prevtime_sub2));

            if (subtraction <= 0x2710){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat2 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat2 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat2 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat2 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat2 = 0x55;
            }
            std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub2 =
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub2;
        }
        if (subblock_num >= 3)
        {
            std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub3 = curTick();

            uint64_t subtraction = ((std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->curtime_sub3) -
                                    (std::static_pointer_cast<LRUReplData>
                                    (replacement_data)->prevtime_sub3));
            if (subtraction <= 0x2710){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat3 = 0x87;
            }
            if ((subtraction > 0x2710)&& (subtraction <= 0x4e20)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat3 = 0x69;
            }
            if ((subtraction > 0x4e20)&& (subtraction <= 0x7530)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat3 = 0x5f;
            }
            if ((subtraction > 0x7530)&& (subtraction <= 0x9c40)){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat3 = 0x5a;
            }
            if (subtraction > 0x9c40){
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->heat3 = 0x55;
            }

            std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub3 =
                std::static_pointer_cast<LRUReplData>
                (replacement_data)->curtime_sub3;

        }
    }
}

uint64_t
LRU::getheat(const std::shared_ptr<ReplacementData>& replacement_data,
            const int i) const
{
    if (i==0){
        int heat0=std::static_pointer_cast<LRUReplData>(
            replacement_data)->heat0;
        return heat0;
    }
    if (i==1){
        int heat1=std::static_pointer_cast<LRUReplData>(
            replacement_data)->heat1;
        return heat1;
    }
    if (i==2){
        int heat2=std::static_pointer_cast<LRUReplData>(
            replacement_data)->heat2;
        return heat2;
    }
    else{
        int heat3=std::static_pointer_cast<LRUReplData>(
            replacement_data)->heat3;
        return heat3;
    }
}
uint64_t
LRU::gettime(const std::shared_ptr<ReplacementData>& replacement_data,
            const int i) const
{
    if (i==0){
        int time0=std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub0;
        return time0;
    }
    if (i==1){
        int time1=std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub1;
        return time1;
    }
    if (i==2){
        int time2=std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub2;
        return time2;
    }
    else{
        int time3=std::static_pointer_cast<LRUReplData>
                (replacement_data)->prevtime_sub3;
        return time3;
    }
}
void
LRU::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<LRUReplData>(
        replacement_data)->lastTouchTick = curTick();
}

ReplaceableEntry*
LRU::getVictim(const ReplacementCandidates& candidates,
                const uint64_t type) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    for (const auto& candidate : candidates) {
        // Update victim entry if necessary
        if (std::static_pointer_cast<LRUReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<LRUReplData>(
                    victim->replacementData)->lastTouchTick) {
            victim = candidate;
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
LRU::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new LRUReplData());
}

} // namespace replacement_policy
} // namespace gem5
