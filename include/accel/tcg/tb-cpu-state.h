/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Definition of TCGTBCPUState.
 */

#ifndef EXEC_TB_CPU_STATE_H
#define EXEC_TB_CPU_STATE_H

#include "qemu/typedefs.h"
#include "exec/vaddr.h"

typedef struct TCGTBCPUState {
    vaddr pc;
    uint32_t flags;
    uint32_t cflags;
    uint64_t cs_base;
} TCGTBCPUState;

/* Lookup after the caller allows I/O and extracts the current target state. */
const void *tcg_lookup_tb_ptr(CPUState *cpu, TCGTBCPUState state);

#endif
