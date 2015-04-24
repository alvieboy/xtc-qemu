/*
 * XTC gdb server stub
 *
 * Copyright (c) 2015 Alvaro Lopes
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
#include "config.h"
#include "qemu-common.h"
#include "exec/gdbstub.h"

int xtc_cpu_gdb_read_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    XTCCPU *cpu = XTC_CPU(cs);
    CPUXTCState *env = &cpu->env;

    if (n < 32) {
        return gdb_get_reg32(mem_buf, env->regs[n]);
    } else {
        return gdb_get_reg32(mem_buf, env->sregs[n - 32]);
    }
    return 0;
}

int xtc_cpu_gdb_write_register(CPUState *cs, uint8_t *mem_buf, int n)
{
    XTCCPU *cpu = XTC_CPU(cs);
    CPUClass *cc = CPU_GET_CLASS(cs);
    CPUXTCState *env = &cpu->env;
    uint32_t tmp;

    if (n > cc->gdb_num_core_regs) {
        return 0;
    }

    tmp = ldl_p(mem_buf);

    if (n < 32) {
        env->regs[n] = tmp;
    } else {
        env->sregs[n - 32] = tmp;
    }
    return 4;
}
