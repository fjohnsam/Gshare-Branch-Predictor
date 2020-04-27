#include "cpu/pred/gshare.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"

GshareBP::GshareBP(const GshareBPParams *params)
    : BPredUnit(params),
      globalPredictorSize(params->globalPredictorSize),
      globalCtrBits(params->globalCtrBits),
      globalCtrs(globalPredictorSize, SatCounter(globalCtrBits)),
      globalHistory(params->numThreads, 0),
      globalHistoryBits(params->globalHistoryBits)
{

    globalHistoryMask = globalPredictorSize - 1;

    historyRegisterMask = mask(globalHistoryBits);

    if (globalHistoryMask > historyRegisterMask) {
        fatal("Global predictor too large for global history bits!\n");
    }
    
    if (globalHistoryMask < historyRegisterMask ) {
        inform("More global history bits than required by predictors\n");
    }
    globalThreshold = (ULL(1) << (globalCtrBits - 1)) - 1;
}

inline
void
GshareBP::updateGlobalHistTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1) | 1;
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
GshareBP::updateGlobalHistNotTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1);
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}


void
GshareBP::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    //Update Global History to Not Taken (clear LSB)
    globalHistory[tid] &= (historyRegisterMask & ~ULL(1));
}

bool
GshareBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    bool global_prediction;

    //Lookup in the global predictor to get its branch prediction
    global_prediction = globalThreshold <
      globalCtrs[globalHistory[tid] & globalHistoryMask];

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->globalPredTaken = global_prediction;
    bp_history = (void *)history;

    // Speculative update of the global history 
    if (global_prediction) {
        updateGlobalHistTaken(tid);
        return true;
    } else {
        updateGlobalHistNotTaken(tid);
        return false;
    }

}

void
GshareBP::uncondBranch(ThreadID tid, Addr pc, void * &bp_history)
{
    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->globalPredTaken = true;
    bp_history = static_cast<void *>(history);

    updateGlobalHistTaken(tid);
}

void
GshareBP::update(ThreadID tid, Addr branch_addr, bool taken,
                     void *bp_history, bool squashed,
                     const StaticInstPtr & inst, Addr corrTarget)
{
    assert(bp_history);

    BPHistory *history = static_cast<BPHistory *>(bp_history);
  
    // If this is a misprediction, restore the speculatively
    // updated state (global history register)
    // and update again.
    if (squashed) {
        // Global history restore and update
        globalHistory[tid] = (history->globalHistory << 1) | taken;
        globalHistory[tid] &= historyRegisterMask;

        return;
    }


    // Update the counters with the proper
    // resolution of the branch. Histories are updated
    // speculatively, restored upon squash() calls, and
    // recomputed upon update(squash = true) calls,
    // so they do not need to be updated.
    unsigned global_predictor_idx =
            history->globalHistory & globalHistoryMask;
    if (taken) {
        globalCtrs[global_predictor_idx]++;
    } else {
        globalCtrs[global_predictor_idx]--;
    }

    // We're done with this history, now delete it.
    delete history;
}

void
GshareBP::squash(ThreadID tid, void *bp_history)
{
    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // Restore global history to state prior to this branch.
    globalHistory[tid] = history->globalHistory;

    // Delete this BPHistory now that we're done with it.
    delete history;
}

GshareBP*
GshareBPParams::create()
{
    return new GshareBP(this);
}

#ifdef DEBUG
int
GshareBP::BPHistory::newCount = 0;
#endif
