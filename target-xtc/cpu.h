/*
 *  XThunderCore virtual CPU header
 *
 *  Copyright (c) 2015 Alvaro Lopes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CPU_XTC_H
#define CPU_XTC_H

#include "config.h"
#include "qemu-common.h"

#define TARGET_LONG_BITS 32

#define CPUArchState struct CPUXTCState

#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"
struct CPUXTCState;
typedef struct CPUXTCState CPUXTCState;
#if !defined(CONFIG_USER_ONLY)
#include "mmu.h"
#endif

#define ELF_MACHINE	EM_XTC

#define EXCP_NMI        1
#define EXCP_MMU        2
#define EXCP_IRQ        3
#define EXCP_BREAK      4
#define EXCP_HW_BREAK   5
#define EXCP_HW_EXCP    6

/* CPU flags.  */

/* Condition codes.  */

#define CC_NONE 0
#define CC_NE  1
#define CC_EQ  2
#define CC_GT  3
#define CC_GE  4
#define CC_LT  5
#define CC_LE  6
#define CC_UGT 7
#define CC_UGE 8
#define CC_ULT 9
#define CC_ULE 10
#define CC_SI  11
#define CC_NS  12

#define STREAM_EXCEPTION (1 << 0)
#define STREAM_ATOMIC    (1 << 1)
#define STREAM_TEST      (1 << 2)
#define STREAM_CONTROL   (1 << 3)
#define STREAM_NONBLOCK  (1 << 4)

struct xtc_cop_regs
{
    uint32_t regs[8];
};

#define SR_Y 0

#define SR_PSR 1
#define SR_SPSR 2
#define SR_PC 3


#define PSR_SUPERVISOR (1<<0)
#define PSR_IEN (1<<1)
#define PSR_DBGEN (1<<2)

#define NB_MMU_MODES 3

struct CPUXTCState {
    uint32_t debug;
    uint32_t btaken;
    uint32_t btarget;
    uint32_t bimm;

    uint32_t imm;
    uint32_t regs[32];
    uint32_t sregs[4];
    uint32_t cc_src;
    uint32_t cc_src2;
    uint32_t cc_dst;

    //    float_status fp_status;

    // Coprocessor regs.
    struct xtc_cop_regs copregs[4];

    /* Internal flags.  */
//#define IMM_FLAG        1
    uint32_t iflags;

    struct {
        uint32_t regs[16];
    } pvr;

#if !defined(CONFIG_USER_ONLY)
    /* Unified MMU.  */
    struct xtc_mmu mmu;
#endif

    CPU_COMMON
};

#include "cpu-qom.h"

void xtc_tcg_init(void);
XTCCPU *cpu_xtc_init(const char *cpu_model);
int cpu_xtc_exec(CPUXTCState *s);
/* you can call this signal handler from your SIGBUS and SIGSEGV
   signal handlers to inform the virtual CPU of exceptions. non zero
   is returned if the signal was handled by the virtual CPU.  */
int cpu_xtc_signal_handler(int host_signum, void *pinfo,
                          void *puc);

enum {
    CC_OP_DYNAMIC, /* Use env->cc_op  */
    CC_OP_FLAGS,
    CC_OP_CMP,
};

/* FIXME: MB uses variable pages down to 1K but linux only uses 4k.  */
#define TARGET_PAGE_BITS 12
#define MMAP_SHIFT TARGET_PAGE_BITS

#define TARGET_PHYS_ADDR_SPACE_BITS 32
#define TARGET_VIRT_ADDR_SPACE_BITS 32

#define cpu_init(cpu_model) CPU(cpu_xtc_init(cpu_model))

#define cpu_exec cpu_xtc_exec
#define cpu_gen_code cpu_xtc_gen_code
#define cpu_signal_handler cpu_xtc_signal_handler

/* MMU modes definitions */
#define MMU_MODE0_SUFFIX _nommu
#define MMU_MODE1_SUFFIX _kernel
#define MMU_MODE2_SUFFIX _user
#define MMU_NOMMU_IDX   0
#define MMU_KERNEL_IDX  1
#define MMU_USER_IDX    2
/* See NB_MMU_MODES further up the file.  */

static inline int cpu_mmu_index (CPUXTCState *env)
{
    if (env->sregs[SR_PSR] & PSR_SUPERVISOR)
        return MMU_KERNEL_IDX;
    return MMU_USER_IDX;
}

int xtc_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int rw,
                            int mmu_idx);

static inline int cpu_interrupts_enabled(CPUXTCState *env)
{
    return env->sregs[SR_PSR] & PSR_IEN;
}

#include "exec/cpu-all.h"

static inline target_ulong cpu_get_pc(CPUXTCState *env)
{
    return env->sregs[SR_PC];
}

static inline void cpu_get_tb_cpu_state(CPUXTCState *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *pc = env->sregs[SR_PC];
//    printf("PC: 0x%08x\n",*pc);
    *cs_base = 0;
    *flags = 0;
}

#if !defined(CONFIG_USER_ONLY)
void xtc_cpu_unassigned_access(CPUState *cpu, hwaddr addr,
                              bool is_write, bool is_exec, int is_asi,
                              unsigned size);
#endif

#include "exec/exec-all.h"

#endif
