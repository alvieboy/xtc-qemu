/*
 * QEMU XThunderCore CPU
 *
 * Copyright (c) 2015 Alvaro Lopes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */
#ifndef QEMU_XTC_CPU_QOM_H
#define QEMU_XTC_CPU_QOM_H

#include "qom/cpu.h"

#define TYPE_XTC_CPU "xtc-cpu"

#define XTC_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(XTCCPUClass, (klass), TYPE_XTC_CPU)
#define XTC_CPU(obj) \
    OBJECT_CHECK(XTCCPU, (obj), TYPE_XTC_CPU)
#define XTC_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(XTCCPUClass, (obj), TYPE_XTC_CPU)

/**
 * XTCCPUClass:
 * @parent_realize: The parent class' realize handler.
 * @parent_reset: The parent class' reset handler.
 *
 * A XTC CPU model.
 */
typedef struct XTCCPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);
} XTCCPUClass;

/**
 * XTCCPU:
 * @env: #CPUXTCState
 *
 * A XTC CPU.
 */
typedef struct XTCCPU {
    /*< private >*/
    CPUState parent_obj;
    uint32_t base_vectors;
    uint32_t reset_address;
    /*< public >*/

    CPUXTCState env;
} XTCCPU;

static inline XTCCPU *xtc_env_get_cpu(CPUXTCState *env)
{
    return container_of(env, XTCCPU, env);
}

#define ENV_GET_CPU(e) CPU(xtc_env_get_cpu(e))

#define ENV_OFFSET offsetof(XTCCPU, env)

void xtc_cpu_do_interrupt(CPUState *cs);
bool xtc_cpu_exec_interrupt(CPUState *cs, int int_req);
void xtc_cpu_dump_state(CPUState *cpu, FILE *f, fprintf_function cpu_fprintf,
                       int flags);
hwaddr xtc_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
int xtc_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int xtc_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);

#endif
