/*
 *  XTC helper routines.
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
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include <assert.h>
#include "cpu.h"
#include "exec/helper-proto.h"
#include "qemu/host-utils.h"
#include "exec/cpu_ldst.h"

#define D(x)

#if !defined(CONFIG_USER_ONLY)

/* Try to fill the TLB and return an exception if error. If retaddr is
 * NULL, it means that the function was called in C code (i.e. not
 * from generated code or from helper.c)
 */
void tlb_fill(CPUState *cs, target_ulong addr, int is_write, int mmu_idx,
              uintptr_t retaddr)
{
    target_ulong physical = addr;
    int prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;

    tlb_set_page(cs, addr & TARGET_PAGE_MASK,
                 physical & TARGET_PAGE_MASK, prot,
                 mmu_idx, TARGET_PAGE_SIZE);
}
#endif

void helper_put(uint32_t id, uint32_t ctrl, uint32_t data)
{
    int test = ctrl & STREAM_TEST;
    int atomic = ctrl & STREAM_ATOMIC;
    int control = ctrl & STREAM_CONTROL;
    int nonblock = ctrl & STREAM_NONBLOCK;
    int exception = ctrl & STREAM_EXCEPTION;

    qemu_log("Unhandled stream put to stream-id=%d data=%x %s%s%s%s%s\n",
             id, data,
             test ? "t" : "",
             nonblock ? "n" : "",
             exception ? "e" : "",
             control ? "c" : "",
             atomic ? "a" : "");
}

uint32_t helper_get(uint32_t id, uint32_t ctrl)
{
    int test = ctrl & STREAM_TEST;
    int atomic = ctrl & STREAM_ATOMIC;
    int control = ctrl & STREAM_CONTROL;
    int nonblock = ctrl & STREAM_NONBLOCK;
    int exception = ctrl & STREAM_EXCEPTION;

    qemu_log("Unhandled stream get from stream-id=%d %s%s%s%s%s\n",
             id,
             test ? "t" : "",
             nonblock ? "n" : "",
             exception ? "e" : "",
             control ? "c" : "",
             atomic ? "a" : "");
    return 0xdead0000 | id;
}

void helper_raise_exception(CPUXTCState *env, uint32_t index)
{
    abort();
}

void helper_debug(CPUXTCState *env)
{
    abort();
}

uint32_t helper_cmp(uint32_t a, uint32_t b)
{
    uint32_t t;

    t = b + ~a + 1;
    if ((b & 0x80000000) ^ (a & 0x80000000))
        t = (t & 0x7fffffff) | (b & 0x80000000);
    return t;
}

uint32_t helper_cmpu(uint32_t a, uint32_t b)
{
    uint32_t t;

    t = b + ~a + 1;
    if ((b & 0x80000000) ^ (a & 0x80000000))
        t = (t & 0x7fffffff) | (a & 0x80000000);
    return t;
}

uint32_t helper_clz(uint32_t t0)
{
    abort();
    //return clz32(t0);
}

uint32_t helper_carry(uint32_t a, uint32_t b, uint32_t cf)
{
    abort();
    //uint32_t ncf;
    //ncf = compute_carry(a, b, cf);
    //return ncf;
}

uint32_t helper_divs(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_divu(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}


uint32_t helper_fadd(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_frsub(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fmul(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fdiv(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_un(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_lt(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_eq(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_le(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_gt(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_ne(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_fcmp_ge(CPUXTCState *env, uint32_t a, uint32_t b)
{
    abort();
}

uint32_t helper_flt(CPUXTCState *env, uint32_t a)
{
    abort();
}

uint32_t helper_fint(CPUXTCState *env, uint32_t a)
{
    abort();
}

uint32_t helper_fsqrt(CPUXTCState *env, uint32_t a)
{
    abort();
}

uint32_t helper_pcmpbf(uint32_t a, uint32_t b)
{
    abort();
}

void helper_memalign(CPUXTCState *env, uint32_t addr, uint32_t dr, uint32_t wr,
                     uint32_t mask)
{
    abort();
    /*
     if (addr & mask) {
            qemu_log_mask(CPU_LOG_INT,
                          "unaligned access addr=%x mask=%x, wr=%d dr=r%d\n",
                          addr, mask, wr, dr);
            env->sregs[SR_EAR] = addr;
            env->sregs[SR_ESR] = ESR_EC_UNALIGNED_DATA | (wr << 10) \
                                 | (dr & 31) << 5;
            if (mask == 3) {
                env->sregs[SR_ESR] |= 1 << 11;
            }
            if (!(env->sregs[SR_MSR] & MSR_EE)) {
                return;
            }
            helper_raise_exception(env, EXCP_HW_EXCP);
    }       */
}

void helper_stackprot(CPUXTCState *env, uint32_t addr)
{
#if 0
    if (addr < env->slr || addr > env->shr) {
            qemu_log("Stack protector violation at %x %x %x\n",
                     addr, env->slr, env->shr);
            env->sregs[SR_EAR] = addr;
            env->sregs[SR_ESR] = ESR_EC_STACKPROT;
            helper_raise_exception(env, EXCP_HW_EXCP);
    }
#endif
}

#if !defined(CONFIG_USER_ONLY)
/* Writes/reads to the MMU's special regs end up here.  */
uint32_t helper_mmu_read(CPUXTCState *env, uint32_t rn)
{
    return mmu_read(env, rn);
}

void helper_mmu_write(CPUXTCState *env, uint32_t rn, uint32_t v)
{
    mmu_write(env, rn, v);
}
#if 1
void xtc_cpu_unassigned_access(CPUState *cs, hwaddr addr,
                              bool is_write, bool is_exec, int is_asi,
                              unsigned size)
{

    qemu_log("Unassigned " TARGET_FMT_plx " wr=%d exe=%d\n",
             addr, is_write ? 1 : 0, is_exec ? 1 : 0);
    if (cs == NULL) {
        return;
    }
}
#endif
#endif

void helper_traceinsn(CPUXTCState *env, uint32_t pc, uint32_t opc, uint32_t a, uint32_t b)
{
    static FILE *trace = NULL;
    if (trace==NULL) {
        trace=fopen("trace.txt","w+");
    }
    if (opc&0x8000) {
        fprintf(trace,"E 0x%08x 0x%08x 0x%08x 0x%08x\n", pc, opc,a,b);
        qemu_log("Trace: PC 0x%08x, opc 0x%08x, LHS 0x%08x, RHS 0x%08x\n", pc, opc,a,b);
    } else {
        fprintf(trace,"E 0x%08x 0x%04x     0x%08x 0x%08x\n", pc, opc,a,b);
        qemu_log("Trace: PC 0x%08x, opc 0x%04x, LHS 0x%08x, RHS 0x%08x\n", pc, opc,a,b);
    }
    fflush(trace);
}

void helper_tracecompare(CPUXTCState *env, uint32_t a, uint32_t b, uint32_t r)
{
    qemu_log("Compare 0x%08x and 0x%08x: %d\n",a,b,r);
}


void helper_branchtaken(CPUXTCState *env, uint32_t target)
{
    qemu_log("Branch taken -> 0x%08x\n",target);
}

void helper_memoryread(CPUXTCState *env, uint32_t a, uint32_t d)
{
    qemu_log("Memory Read 0x%08x -> 0x%08x\n",a,d);
}

void helper_memorywrite(CPUXTCState *env, uint32_t a, uint32_t d)
{
    qemu_log("Memory Write 0x%08x <- 0x%08x\n",a,d);
}

uint32_t helper_readpsr(CPUXTCState *env)
{
    return env->regs[SR_PSR];
}

void helper_writepsr(CPUXTCState *env, uint32_t v)
{
    env->regs[SR_PSR] = v;
}

uint32_t helper_readcop(CPUXTCState *env, uint32_t index, uint32_t reg)
{
    switch (index) {
    case 0:
        break;
    default:
        cpu_abort(CPU(env),"Non-implemented read access to Coprocessor %u, register %u",index,reg);
        break;
    }
    return 0;
}

void helper_writecop(CPUXTCState *env, uint32_t index, uint32_t reg, uint32_t value)
{
    switch(index) {
    case 0:
        break;
    default:
        cpu_abort(CPU(env),"Non-implemented write access to Coprocessor %u, register %u",index,reg);
        break;
    }
}
