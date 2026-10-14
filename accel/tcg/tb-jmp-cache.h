/*
 * The per-CPU TranslationBlock jump cache.
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ACCEL_TCG_TB_JMP_CACHE_H
#define ACCEL_TCG_TB_JMP_CACHE_H

#include "qemu/rcu.h"
#include "exec/cpu-common.h"

#define TB_JMP_CACHE_BITS 16
#define TB_JMP_CACHE_SIZE (1 << TB_JMP_CACHE_BITS)
#define TB_JMP_CACHE_GROUPS (2 * CPU_TLB_INVALIDATION_GROUPS)

/* Each permission group has context-dependent and independent code classes. */
static inline uint8_t tb_jmp_cache_group(unsigned group,
                                        bool context_independent)
{
    assert(group < CPU_TLB_INVALIDATION_GROUPS);
    return group + (context_independent ? CPU_TLB_INVALIDATION_GROUPS : 0);
}

/*
 * Invalidated in parallel; accesses to 'tb' and the cache generation must
 * be atomic. Group generations are accessed only by the owning CPU.
 * Generation zero disables hits and publication during wrap.
 * A valid entry is read/written by a single CPU, therefore there is
 * no need for qatomic_rcu_read(); pc and the entry generations are consistent
 * with a non-NULL value of 'tb'.  Matching generations must be checked before
 * dereferencing 'tb', since flushes leave stale pointers in the array.
 * Strictly speaking pc is only needed for CF_PCREL, but it's used always for
 * simplicity.
 */
typedef struct CPUJumpCache {
    struct rcu_head rcu;
    uint64_t generation;
    uint64_t group_flush_generation;
    uint64_t group_generation[TB_JMP_CACHE_GROUPS];
    struct {
        TranslationBlock *tb;
        vaddr pc;
        uint64_t generation;
        uint64_t group_generation;
        uint8_t group;
    } array[TB_JMP_CACHE_SIZE];
} CPUJumpCache;

#endif /* ACCEL_TCG_TB_JMP_CACHE_H */
