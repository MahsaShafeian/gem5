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

/**
 * @file
 * Declaration of a Least Recently Used replacement policy.
 * The victim is chosen using the last touch timestamp.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SMT_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SMT_RP_HH__

#include "mem/cache/replacement_policies/base.hh"

namespace gem5
{

struct SMTRPParams;

namespace replacement_policy
{

class SMT : public Base
{
  protected:
    /** SMT-specific implementation of replacement data. */
    struct SMTReplData : ReplacementData
    {
        /** Tick on which the entry was last touched. */
        Tick lastTouchTick;
        uint64_t type = 0x0000;
        Tick curtime_sub0, curtime_sub1, curtime_sub2, curtime_sub3;
        Tick prevtime_sub0, prevtime_sub1, prevtime_sub2, prevtime_sub3;
        uint64_t heat0 = 85, heat1 = 85, heat2 = 85, heat3 = 85;
        /**
         * Default constructor. Invalidate data.
         */
        SMTReplData() : lastTouchTick(0) {}
    };

  public:
    typedef SMTRPParams Params;
    SMT(const Params &p);
    ~SMT() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;
    void touchBit(const std::shared_ptr<ReplacementData>& replacement_data,
                  const PacketPtr pkt = nullptr) const override;
    uint64_t getheat(const std::shared_ptr<ReplacementData>& replacement_data,
                    const int i) const override;
    uint64_t gettime(const std::shared_ptr<ReplacementData>& replacement_data,
                    const int i) const override;
    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using SMT timestamps.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates,
                                const uint64_t type) const override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SMT_RP_HH__
