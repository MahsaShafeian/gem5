/*
 * Copyright (c) 2012, 2014, 2018 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2011 Regents of the University of California
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

#ifndef __SYSTEM_HH__
#define __SYSTEM_HH__

#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "base/loader/memory_image.hh"
#include "base/loader/symtab.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/pc_event.hh"
#include "cpu/thread_context.hh"
#include "enums/MemoryMode.hh"
#include "mem/mem_requestor.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "params/System.hh"
#include "sim/eventq.hh"
#include "sim/futex_map.hh"
#include "sim/redirect_path.hh"
#include "sim/se_signal.hh"
#include "sim/sim_object.hh"
#include "sim/workload.hh"

namespace gem5
{

class BaseRemoteGDB;
class KvmVM;
class ThreadContext;

class System : public SimObject, public PCEventScope
{


  private:
    bool doPrint = false;

  public:
    Addr appVaddr = 0;
    // Addr bVaddr = 0;
    // Addr cVaddr = 0;
    // Addr dVaddr = 0;
    // Addr eVaddr = 0;
    // Addr fVaddr = 0;
    // Addr gVaddr = 0;
    // Addr hVaddr = 0;

  private:
    /**
     * Private class for the system port which is only used as a
     * requestor for debug access and for non-structural entities that do
     * not have a port of their own.
     */
    class SystemPort : public RequestPort
    {
      public:

        /**
         * Create a system port with a name and an owner.
         */
        SystemPort(const std::string &_name)
            : RequestPort(_name)
        { }

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            panic("SystemPort does not receive timing!");
        }

        void
        recvReqRetry() override
        {
            panic("SystemPort does not expect retry!");
        }
    };

    std::list<PCEvent *> liveEvents;
    SystemPort _systemPort;

    // Map of memory address ranges for devices with their own backing stores
    std::unordered_map<RequestorID, std::vector<memory::AbstractMemory *>>
        deviceMemMap;

  public:
    void addWrite(Addr paddr, Addr vaddr, uint8_t size, int coreId);
    void printWrites(uint64_t Program);
    void resetWrites(uint64_t Program);
    void printAddress(uint64_t Program);
    void logAction(ThreadContext *tc, uint64_t Program, uint64_t action,
                   uint64_t functionNumber,uint64_t rbpVA,uint64_t stackSize);
    void printFunctionCallCounts() const;
    void printEnd() const;
    void printStart_Reset() const;

    enum LogAction
    {
      LOG_RESET_WRITES        = 0,
      LOG_PRINT_WRITES       = 1,
    //   LOG_PRINT_ADDRESS      = 2,
      LOG_FUNCTION_INFO      = 3,
    };
    struct FunctionInfo
    {
        uint64_t callCount = 0;
        uint64_t rbpPA = 0;
    };

    // system.hh — alternative if you need both paddr and vaddr
    struct AddrKey
    {
        Addr paddr;
        Addr vaddr;
        bool operator==(const AddrKey& o) const {
            return paddr == o.paddr && vaddr == o.vaddr;
        }
    };
    struct AddrKeyHash
    {
        std::size_t operator()(const AddrKey& k) const {
            return std::hash<Addr>{}(k.paddr) ^
                  (std::hash<Addr>{}(k.vaddr) << 32);
        }
    };



    struct FunctionKey
    {
        uint64_t Program;
        uint64_t functionNumber;
        uint64_t rbpVA;
        uint64_t stackSize;

        bool operator==(const FunctionKey &other) const
        {
            return Program == other.Program &&
                  functionNumber == other.functionNumber &&
                  rbpVA == other.rbpVA &&
                  stackSize == other.stackSize;
        }
    };

    struct FunctionKeyHash
    {
        std::size_t operator()(const FunctionKey &k) const
        {
            std::size_t h = std::hash<uint64_t>{}(k.Program);

            h ^= std::hash<uint64_t>{}(k.functionNumber)
                + 0x9e3779b9 + (h << 6) + (h >> 2);

            h ^= std::hash<uint64_t>{}(k.rbpVA)
                + 0x9e3779b9 + (h << 6) + (h >> 2);

            h ^= std::hash<uint64_t>{}(k.stackSize)
                + 0x9e3779b9 + (h << 6) + (h >> 2);

            return h;
        }
    };
  private:
    uint64_t incrementFunctionCall(uint64_t Program,
                               uint64_t functionNumber,
                               uint64_t rbpVA,
                               uint64_t rbpPA,
                               uint64_t stackSize);

    mutable std::mutex logMutex;
    std::unordered_map<FunctionKey, FunctionInfo,
                       FunctionKeyHash> functionCallCounts;
    static constexpr unsigned STORM_BLK       = 64;
    static constexpr unsigned STORM_NUM_SETS  = 128;
    static constexpr unsigned STORM_REGION_SZ = 4;
    static constexpr unsigned STORM_NUM_REGION = STORM_BLK / STORM_REGION_SZ;
    static constexpr unsigned STORM_MAX_CORES = 8;
    static constexpr unsigned STORM_SLOTS     = STORM_MAX_CORES + 1;
    uint64_t stormGrid[STORM_SLOTS][STORM_NUM_SETS][STORM_NUM_REGION] = {};
    uint64_t stormSetWrites[STORM_SLOTS][STORM_NUM_SETS] = {};
    Tick     stormLastTick[STORM_SLOTS][STORM_NUM_SETS]  = {};
    uint64_t stormDelayAccum[STORM_SLOTS][STORM_NUM_SETS] = {};
    uint64_t stormDelaySamples[STORM_SLOTS][STORM_NUM_SETS] = {};
    bool     stormCoreSeen[STORM_SLOTS] = {};

    std::unordered_map<AddrKey, uint64_t, AddrKeyHash>
        writeAccessMap[STORM_SLOTS];
    int writecall = 0;
    bool reset = true;

  public:
    // ---- StORM set x region aggregation + inter-write delay ----

    void stormDumpGrid();   // -> $STORM_GRID_FILE (default storm_grid.txt)
    void stormDumpSets();   // -> $STORM_SETS_FILE (default storm_sets.csv)

//     void addWrite(ContextID ctxId, Addr paddr, Addr vaddr, uint8_t size);
//     void printWrites(uint64_t program);
//     void resetWrites(uint64_t program);
//     void logAction(ThreadContext *tc, uint64_t program, uint64_t action,
//                    uint64_t functionNumber, uint64_t rbpVA,
//                    uint64_t stackSize);
//     void printFunctionCallCounts(uint64_t program) const;

//     enum LogAction
//     {
//         LOG_RESET_WRITES  = 0,
//         LOG_PRINT_WRITES  = 1,
//         LOG_FUNCTION_INFO = 3,
//     };

//     struct AddrKey
//     {
//         Addr paddr;
//         Addr vaddr;
//         bool operator==(const AddrKey &o) const
//         { return paddr == o.paddr && vaddr == o.vaddr; }
//     };

//     struct AddrKeyHash
//     {
//         std::size_t operator()(const AddrKey &k) const
//         {
//             std::size_t h = std::hash<Addr>{}(k.paddr);
//             h ^= std::hash<Addr>{}(k.vaddr)
//                  + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
//             return h;
//         }
//     };

//     struct ProgramSlot
//     {
//         uint64_t program = 0;
//         bool     active  = false;
//     };

//   private:
//     /* which program is currently being measured on each hardware thread */
//     std::unordered_map<ContextID, ProgramSlot> ctxToProgram;

//     /* program id -> that program's own write map */
//     std::unordered_map<uint64_t,
//         std::unordered_map<AddrKey, uint64_t, AddrKeyHash>> writeAccessMap;

//     std::unordered_map<FunctionKey, FunctionInfo, FunctionKeyHash>
//         functionCallCounts;

//     mutable std::mutex logMutex;

  public:
    class Threads
    {
      private:
        struct Thread
        {
            ThreadContext *context = nullptr;
            bool active = false;
            Event *resumeEvent = nullptr;

            void resume();
            std::string name() const;
            void quiesce() const;
        };

        std::vector<Thread> threads;

        Thread &
        thread(ContextID id)
        {
            assert(id < size());
            return threads[id];
        }

        const Thread &
        thread(ContextID id) const
        {
            assert(id < size());
            return threads[id];
        }

        void insert(ThreadContext *tc);
        void replace(ThreadContext *tc, ContextID id);

        friend class System;

      public:
        class const_iterator
        {
          private:
            Threads const* threads;
            int pos;

            friend class Threads;

            const_iterator(const Threads &_threads, int _pos) :
                threads(&_threads), pos(_pos)
            {}

          public:
            using iterator_category = std::forward_iterator_tag;
            using value_type = ThreadContext *;
            using difference_type = int;
            using pointer = const value_type *;
            using reference = const value_type &;

            const_iterator &
            operator ++ ()
            {
                pos++;
                return *this;
            }

            const_iterator
            operator ++ (int)
            {
                return const_iterator(*threads, pos++);
            }

            reference operator * () { return threads->thread(pos).context; }
            pointer operator -> () { return &threads->thread(pos).context; }

            bool
            operator == (const const_iterator &other) const
            {
                return threads == other.threads && pos == other.pos;
            }

            bool
            operator != (const const_iterator &other) const
            {
                return !(*this == other);
            }
        };

        ThreadContext *findFree();

        ThreadContext *
        operator [](ContextID id) const
        {
            return thread(id).context;
        }

        void markActive(ContextID id) { thread(id).active = true; }

        int size() const { return threads.size(); }
        bool empty() const { return threads.empty(); }
        int numRunning() const;
        int
        numActive() const
        {
            int count = 0;
            for (auto &thread: threads) {
                if (thread.active)
                    count++;
            }
            return count;
        }

        void quiesce(ContextID id);
        void quiesceTick(ContextID id, Tick when);

        const_iterator begin() const { return const_iterator(*this, 0); }
        const_iterator end() const { return const_iterator(*this, size()); }
    };

    /**
     * Get a reference to the system port that can be used by
     * non-structural simulation objects like processes or threads, or
     * external entities like loaders and debuggers, etc, to access
     * the memory system.
     *
     * @return a reference to the system port we own
     */
    RequestPort& getSystemPort() { return _systemPort; }

    /**
     * Additional function to return the Port of a memory object.
     */
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    /** @{ */
    /**
     * Is the system in atomic mode?
     *
     * There are currently two different atomic memory modes:
     * 'atomic', which supports caches; and 'atomic_noncaching', which
     * bypasses caches. The latter is used by hardware virtualized
     * CPUs. SimObjects are expected to use Port::sendAtomic() and
     * Port::recvAtomic() when accessing memory in this mode.
     */
    bool
    isAtomicMode() const
    {
        return memoryMode == enums::atomic ||
            memoryMode == enums::atomic_noncaching;
    }

    /**
     * Is the system in timing mode?
     *
     * SimObjects are expected to use Port::sendTiming() and
     * Port::recvTiming() when accessing memory in this mode.
     */
    bool isTimingMode() const { return memoryMode == enums::timing; }

    /**
     * Should caches be bypassed?
     *
     * Some CPUs need to bypass caches to allow direct memory
     * accesses, which is required for hardware virtualization.
     */
    bool
    bypassCaches() const
    {
        return memoryMode == enums::atomic_noncaching;
    }
    /** @} */

    /** @{ */
    /**
     * Get the memory mode of the system.
     *
     * \warn This should only be used by the Python world. The C++
     * world should use one of the query functions above
     * (isAtomicMode(), isTimingMode(), bypassCaches()).
     */
    enums::MemoryMode getMemoryMode() const { return memoryMode; }

    /**
     * Change the memory mode of the system.
     *
     * \warn This should only be called by the Python!
     *
     * @param mode Mode to change to (atomic/timing/...)
     */
    void setMemoryMode(enums::MemoryMode mode);
    /** @} */

    /**
     * Get the cache line size of the system.
     */
    Addr cacheLineSize() const { return _cacheLineSize; }

    Threads threads;

    const bool multiThread;

    using SimObject::schedule;

    bool schedule(PCEvent *event) override;
    bool remove(PCEvent *event) override;

    uint64_t init_param;

    /** Port to physical memory used for writing object files into ram at
     * boot.*/
    PortProxy physProxy;

    /** OS kernel */
    Workload *workload = nullptr;

  public:
    /**
     * Get a pointer to the Kernel Virtual Machine (KVM) SimObject,
     * if present.
     */
    KvmVM *getKvmVM() const { return kvmVM; }

    /**
     * Set the pointer to the Kernel Virtual Machine (KVM) SimObject. For use
     * by that object to declare itself to the system.
     */
    void setKvmVM(KvmVM *const vm) { kvmVM = vm; }

    /** Get a pointer to access the physical memory of the system */
    memory::PhysicalMemory& getPhysMem() { return physmem; }
    const memory::PhysicalMemory& getPhysMem() const { return physmem; }

    /** Amount of physical memory that exists */
    Addr memSize() const;

    /**
     * Check if a physical address is within a range of a memory that
     * is part of the global address map.
     *
     * @param addr A physical address
     * @return Whether the address corresponds to a memory
     */
    bool isMemAddr(Addr addr) const;

    /**
     * Add a physical memory range for a device. The ranges added here will
     * be considered a non-PIO memory address if the requestorId of the packet
     * and range match something in the device memory map.
     */
    void addDeviceMemory(RequestorID requestorId,
        memory::AbstractMemory *deviceMemory);

    /**
     * Similar to isMemAddr but for devices. Checks if a physical address
     * of the packet match an address range of a device corresponding to the
     * RequestorId of the request.
     */
    bool isDeviceMemAddr(const PacketPtr& pkt) const;

    /**
     * Return a pointer to the device memory.
     */
    memory::AbstractMemory *getDeviceMemory(const PacketPtr& pkt) const;

    /*
     * Return the list of address ranges backed by a shadowed ROM.
     *
     * @return List of address ranges backed by a shadowed ROM
     */
    AddrRangeList getShadowRomRanges() const { return ShadowRomRanges; }

    /**
     * Get the guest byte order.
     */
    ByteOrder
    getGuestByteOrder() const
    {
        return workload->byteOrder();
    }

    /**
     * The thermal model used for this system (if any).
     */
    ThermalModel * getThermalModel() const { return thermalModel; }

  protected:

    KvmVM *kvmVM = nullptr;

    memory::PhysicalMemory physmem;

    AddrRangeList ShadowRomRanges;

    enums::MemoryMode memoryMode;

    const Addr _cacheLineSize;

    uint64_t workItemsBegin = 0;
    uint64_t workItemsEnd = 0;
    uint32_t numWorkIds;

    /** This array is a per-system list of all devices capable of issuing a
     * memory system request and an associated string for each requestor id.
     * It's used to uniquely id any requestor in the system by name for things
     * like cache statistics.
     */
    std::vector<RequestorInfo> requestors;

    ThermalModel * thermalModel;

  protected:
    /**
     * Strips off the system name from a requestor name
     */
    std::string stripSystemName(const std::string& requestor_name) const;

  public:

    /**
     * Request an id used to create a request object in the system. All objects
     * that intend to issues requests into the memory system must request an id
     * in the init() phase of startup. All requestor ids must be fixed by the
     * regStats() phase that immediately precedes it. This allows objects in
     * the memory system to understand how many requestors may exist and
     * appropriately name the bins of their per-requestor stats before the
     * stats are finalized.
     *
     * Registers a RequestorID:
     * This method takes two parameters, one of which is optional.
     * The first one is the requestor object, and it is compulsory; in case
     * a object has multiple (sub)requestors, a second parameter must be
     * provided and it contains the name of the subrequestor. The method will
     * create a requestor's name by concatenating the SimObject name with the
     * eventual subrequestor string, separated by a dot.
     *
     * As an example:
     * For a cpu having two requestors: a data requestor and an
     * instruction requestor,
     * the method must be called twice:
     *
     * instRequestorId = getRequestorId(cpu, "inst");
     * dataRequestorId = getRequestorId(cpu, "data");
     *
     * and the requestors' names will be:
     * - "cpu.inst"
     * - "cpu.data"
     *
     * @param requestor SimObject related to the requestor
     * @param subrequestor String containing the subrequestor's name
     * @return the requestor's ID.
     */
    RequestorID getRequestorId(const SimObject* requestor,
                         std::string subrequestor={});

    /**
     * Registers a GLOBAL RequestorID, which is a RequestorID not related
     * to any particular SimObject; since no SimObject is passed,
     * the requestor gets registered by providing the full requestor name.
     *
     * @param requestorName full name of the requestor
     * @return the requestor's ID.
     */
    RequestorID getGlobalRequestorId(const std::string& requestor_name);

    /**
     * Get the name of an object for a given request id.
     */
    std::string getRequestorName(RequestorID requestor_id);

    /**
     * Looks up the RequestorID for a given SimObject
     * returns an invalid RequestorID (invldRequestorId) if not found.
     */
    RequestorID lookupRequestorId(const SimObject* obj) const;

    /**
     * Looks up the RequestorID for a given object name string
     * returns an invalid RequestorID (invldRequestorId) if not found.
     */
    RequestorID lookupRequestorId(const std::string& name) const;

    /** Get the number of requestors registered in the system */
    RequestorID maxRequestors() { return requestors.size(); }

  protected:
    /** helper function for getRequestorId */
    RequestorID _getRequestorId(const SimObject* requestor,
                          const std::string& requestor_name);

    /**
     * Helper function for constructing the full (sub)requestor name
     * by providing the root requestor and the relative subrequestor name.
     */
    std::string leafRequestorName(const SimObject* requestor,
                               const std::string& subrequestor);

  public:

    void regStats() override;
    /**
     * Called by pseudo_inst to track the number of work items started by this
     * system.
     */
    uint64_t
    incWorkItemsBegin()
    {
        return ++workItemsBegin;
    }

    /**
     * Called by pseudo_inst to track the number of work items completed by
     * this system.
     */
    uint64_t
    incWorkItemsEnd()
    {
        return ++workItemsEnd;
    }

    /**
     * Called by pseudo_inst to mark the cpus actively executing work items.
     * Returns the total number of cpus that have executed work item begin or
     * ends.
     */
    int
    markWorkItem(int index)
    {
        threads.markActive(index);
        return threads.numActive();
    }

    void
    workItemBegin(uint32_t tid, uint32_t workid)
    {
        std::pair<uint32_t, uint32_t> p(tid, workid);
        lastWorkItemStarted[p] = curTick();
    }

    void workItemEnd(uint32_t tid, uint32_t workid);

    /* Returns whether we successfully trapped into GDB. */
    bool trapToGdb(GDBSignal signal, ContextID ctx_id) const;

  protected:
    /**
     * Range for memory-mapped m5 pseudo ops. The range will be
     * invalid/empty if disabled.
     */
    const AddrRange _m5opRange;

  public:
    PARAMS(System);

    System(const Params &p);
    ~System();

    /**
     * Range used by memory-mapped m5 pseudo-ops if enabled. Returns
     * an invalid/empty range if disabled.
     */
    const AddrRange &m5opRange() const { return _m5opRange; }

  public:

    void registerThreadContext(ThreadContext *tc);
    void replaceThreadContext(ThreadContext *tc, ContextID context_id);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public:
    std::map<std::pair<uint32_t, uint32_t>, Tick>  lastWorkItemStarted;
    std::map<uint32_t, statistics::Histogram*> workItemStats;

    ////////////////////////////////////////////
    //
    // STATIC GLOBAL SYSTEM LIST
    //
    ////////////////////////////////////////////

    static std::vector<System *> systemList;
    static int numSystemsRunning;

    static void printSystems();

    FutexMap futexMap;

    static const int maxPID = 32768;

    /** Process set to track which PIDs have already been allocated */
    std::set<int> PIDs;

    // By convention, all signals are owned by the receiving process. The
    // receiver will delete the signal upon reception.
    std::list<BasicSignal> signalList;

    // Used by syscall-emulation mode. This member contains paths which need
    // to be redirected to the faux-filesystem (a duplicate filesystem
    // intended to replace certain files on the host filesystem).
    std::vector<RedirectPath*> redirectPaths;
};

void printSystems();

} // namespace gem5

#endif // __SYSTEM_HH__
