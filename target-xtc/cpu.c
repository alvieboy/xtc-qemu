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

#include "cpu.h"
#include "qemu-common.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "monitor/monitor.h"
#include "sysemu/sysemu.h"
#include "trace.h"

static void xtc_cpu_set_pc(CPUState *cs, vaddr value)
{
    XTCCPU *cpu = XTC_CPU(cs);

    cpu->env.sregs[SR_PC] = value;
}

static bool xtc_cpu_has_work(CPUState *cs)
{
    abort();
}

#ifndef CONFIG_USER_ONLY
#if 0
static void xtc_cpu_set_irq(void *opaque, int irq, int level)
{
//    XTCCPU *cpu = opaque;
//    CPUState *cs = CPU(cpu);
    abort();
}
#endif
#endif

/* CPUClass::reset() */
static void xtc_cpu_reset(CPUState *s)
{
    XTCCPU *cpu = XTC_CPU(s);
    XTCCPUClass *mcc = XTC_CPU_GET_CLASS(cpu);
    CPUXTCState *env = &cpu->env;
    mcc->parent_reset(s);

    memset(env, 0, sizeof(CPUXTCState));
    tlb_flush(s, 1);

#if 0
    env->pvr.regs[0] = PVR0_PVR_FULL_MASK \
        | PVR0_USE_BARREL_MASK \
        | PVR0_USE_DIV_MASK \
        | PVR0_USE_HW_MUL_MASK \
        | PVR0_USE_EXC_MASK \
        | PVR0_USE_ICACHE_MASK \
        | PVR0_USE_DCACHE_MASK \
        | PVR0_USE_MMU \
        | (0xb << 8);
    env->pvr.regs[2] = PVR2_D_OPB_MASK \
        | PVR2_D_LMB_MASK \
        | PVR2_I_OPB_MASK \
        | PVR2_I_LMB_MASK \
        | PVR2_USE_MSR_INSTR \
        | PVR2_USE_PCMP_INSTR \
        | PVR2_USE_BARREL_MASK \
        | PVR2_USE_DIV_MASK \
        | PVR2_USE_HW_MUL_MASK \
        | PVR2_USE_MUL64_MASK \
        | PVR2_USE_FPU_MASK \
        | PVR2_USE_FPU2_MASK \
        | PVR2_FPU_EXC_MASK \
        | 0;
    env->pvr.regs[10] = 0x0c000000; /* Default to spartan 3a dsp family.  */
    env->pvr.regs[11] = PVR11_USE_MMU | (16 << 17);
#endif
    env->sregs[SR_PC] = cpu->reset_address;

#if defined(CONFIG_USER_ONLY)
    /* start in user mode with interrupts enabled.  */
    env->sregs[SR_PSR] = 0;
#else
    env->sregs[SR_PSR] = PSR_SUPERVISOR;
//    mmu_init(&env->mmu);
//    env->mmu.c_mmu = 3;
//    env->mmu.c_mmu_tlb_access = 3;
 //   env->mmu.c_mmu_zones = 16;
#endif
}

static Notifier xtc_exit_notifier;

static void xtc_handle_exit(struct Notifier *n, void *data)
{
    dumptrace();
}

static void xtc_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    XTCCPUClass *mcc = XTC_CPU_GET_CLASS(dev);

    cpu_reset(cs);
    qemu_init_vcpu(cs);

    mcc->parent_realize(dev, errp);

    xtc_exit_notifier.notify = xtc_handle_exit;
    qemu_add_exit_notifier(&xtc_exit_notifier);
}   

static void xtc_cpu_initfn(Object *obj)
{
    CPUState *cs = CPU(obj);
    XTCCPU *cpu = XTC_CPU(obj);
    CPUXTCState *env = &cpu->env;
    static bool tcg_initialized;

    cs->env_ptr = env;
    cpu_exec_init(env);

    if (tcg_enabled() && !tcg_initialized) {
        tcg_initialized = true;
        xtc_tcg_init();
    }

    
}


#if 0
static void xtc_cpu_exit(CPUState *cpu)
{
    dumptrace();
}
#endif

static const VMStateDescription vmstate_xtc_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

static Property xtc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void xtc_cpu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    XTCCPUClass *mcc = XTC_CPU_CLASS(oc);

    mcc->parent_realize = dc->realize;
    dc->realize = xtc_cpu_realizefn;

    mcc->parent_reset = cc->reset;
    cc->reset = xtc_cpu_reset;

    cc->has_work = xtc_cpu_has_work;
    //cc->do_interrupt = xtc_cpu_do_interrupt;
    //cc->cpu_exec_interrupt = xtc_cpu_exec_interrupt;
    cc->dump_state = xtc_cpu_dump_state;
    cc->set_pc = xtc_cpu_set_pc;
    cc->gdb_read_register = xtc_cpu_gdb_read_register;
    cc->gdb_write_register = xtc_cpu_gdb_write_register;
#ifdef CONFIG_USER_ONLY
    cc->handle_mmu_fault = xtc_cpu_handle_mmu_fault;
#endif
#if 1
    cc->do_unassigned_access = xtc_cpu_unassigned_access;
#endif
#if 0
    cc->get_phys_page_debug = xtc_cpu_get_phys_page_debug;
#endif
    //cc->cpu_exec_exit = xtc_cpu_exit;
    dc->vmsd = &vmstate_xtc_cpu;
    dc->props = xtc_properties;
    cc->gdb_num_core_regs = 32 + 5;
}

static const TypeInfo xtc_cpu_type_info = {
    .name = TYPE_XTC_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(XTCCPU),
    .instance_init = xtc_cpu_initfn,
    .class_size = sizeof(XTCCPUClass),
    .class_init = xtc_cpu_class_init
};

static void xtc_cpu_register_types(void)
{
    type_register_static(&xtc_cpu_type_info);
}

type_init(xtc_cpu_register_types)
