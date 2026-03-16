/**
 * \file  TlmChanImplCfg.hpp
 * \brief Project override – reduced for embedded STM32 deployment
 *
 * Default F' uses 500 buckets (~594 KB BSS with double buffering).
 * This autopilot has ~30 telemetry channels, so 50 buckets is plenty.
 */

#ifndef TLMCHANIMPLCFG_HPP_
#define TLMCHANIMPLCFG_HPP_

namespace {

enum {
    TLMCHAN_NUM_TLM_HASH_SLOTS = 25,
    TLMCHAN_HASH_MOD_VALUE = 97,
    TLMCHAN_HASH_BUCKETS = 100
};

}

#endif /* TLMCHANIMPLCFG_HPP_ */
