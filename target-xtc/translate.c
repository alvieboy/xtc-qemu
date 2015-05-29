/*
 *  XThunderCore emulation for qemu: main translation routines.
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

#include "cpu.h"
#include "disas/disas.h"
#include "tcg-op.h"
#include "exec/helper-proto.h"
#include "xtc-decode.h"
#include "exec/cpu_ldst.h"
#include "exec/helper-gen.h"

#include "trace-tcg.h"


#define SIM_COMPAT 0
#define DISAS_GNU 1
#define DISAS_XTC 1
#if DISAS_XTC && !SIM_COMPAT
#  define LOG_DIS(...) qemu_log_mask(CPU_LOG_TB_IN_ASM, ## __VA_ARGS__)
#else
#  define LOG_DIS(...) do { } while (0)
#endif

#define D(x)

#define EXTRACT_FIELD(src, start, end) \
            (((src) >> start) & ((1 << (end - start + 1)) - 1))

#define TRACE_INSN
#define TRACE_MEMORY

static TCGv env_debug;
static TCGv_ptr cpu_env;
static TCGv cpu_R[32];
static TCGv cpu_SR[4];
static TCGv env_imm;
static TCGv env_btaken;
static TCGv env_btarget;
static TCGv env_iflags;
static TCGv cpu_cc_src, cpu_cc_src2;
//static TCGv cpu_cc_dst;

#include "exec/gen-icount.h"

#define D_FLAG 1
#define IMM_FLAG 2

/* This is the state at translation time.  */
typedef struct DisasContext {
    XTCCPU *cpu;
    target_ulong pc;

    /* Decoder.  */
    uint32_t ir;
    uint8_t rd, ra, rb;
    uint32_t imm24;
    uint32_t imm;
    uint8_t imm8;
    int cc, savecc;

    int rhs_immediate;
    int is_extended;
    int isextdreg;
    int isextimm;

    uint32_t opcode;

    unsigned int cpustate_changed;
    unsigned int delayed_branch;
    unsigned int conditional_branch;
    unsigned int tb_flags, synced_flags; /* tb dependent flags.  */
    unsigned int clear_imm;
    int is_jmp;

#define JMP_REL     0
#define JMP_ABS     1
    unsigned int jmp;
    uint32_t jmp_pc;

    int abort_at_next_insn;
    int nr_nops;
    struct TranslationBlock *tb;
    int singlestep_enabled;
} DisasContext;

static const char *regnames[] =
{
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"
};

static const char *special_regnames[] =
{
    "y", "psr", "spsr", "pc"
};

static int xtc_get_condition(DisasContext*dc, int cc);

static inline void t_sync_flags(DisasContext *dc)
{
    /* Synch the tb dependent flags between translator and runtime.  */
    tcg_gen_movi_tl(env_iflags, dc->tb_flags);
    tcg_gen_movi_tl(env_imm, dc->imm);
    // tcg_gen_movi_tl(env_imm, dc->imm);
}

static inline void t_gen_raise_exception(DisasContext *dc, uint32_t index)
{
    TCGv_i32 tmp = tcg_const_i32(index);

    t_sync_flags(dc);
    tcg_gen_movi_tl(cpu_SR[SR_PC], dc->pc);
    gen_helper_raise_exception(cpu_env, tmp);
    tcg_temp_free_i32(tmp);
    dc->is_jmp = DISAS_UPDATE;
}
#if 0
static void gen_goto_tb(DisasContext *dc, int n, target_ulong dest)
{
    TranslationBlock *tb;
    tb = dc->tb;
    if ((tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK)) {
        tcg_gen_goto_tb(n);
        tcg_gen_movi_tl(cpu_SR[SR_PC], dest);
        tcg_gen_exit_tb((uintptr_t)tb + n);
    } else {
        tcg_gen_movi_tl(cpu_SR[SR_PC], dest);
        tcg_gen_exit_tb(0);
    }
}
#endif

#if 0
static void read_carry(DisasContext *dc, TCGv d)
{
    tcg_gen_shri_tl(d, cpu_SR[SR_PSR], 30);
    tcg_gen_andi_tl(d, cpu_SR[SR_PSR], 1);
}

/*
 * write_carry sets the carry bits in MSR based on bit 0 of v.
 * v[31:1] are ignored.
 */
static void write_carry(DisasContext *dc, TCGv v)
{
    TCGv t0 = tcg_temp_new();
    tcg_gen_shli_tl(t0, v, 30);
    tcg_gen_andi_tl(t0, v, 0x40000000);
    tcg_gen_andi_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], ~(0x40000000));
    tcg_gen_or_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], t0);
    tcg_temp_free(t0);
}
static void write_carryi(DisasContext *dc, bool carry)
{
    TCGv t0 = tcg_temp_new();
    tcg_gen_movi_tl(t0, carry);
    write_carry(dc, t0);
    tcg_temp_free(t0);
}

#endif

#define RETURN_IF_TARGET_ZERO(dc) \
    if ((dc->rd%16)==0) return;

static void dec_add(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_addi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else {
        tcg_gen_add_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
}


/* Sub, and store carry */
static void dec_csub(DisasContext *dc)
{
    TCGv t0 = tcg_temp_new();

    if (dc->rhs_immediate) {

        TCGv t1 = tcg_temp_new();
        TCGv t2 = tcg_temp_new();

        tcg_gen_movi_tl( t1, dc->imm );
        tcg_gen_not_tl( t2, cpu_R[dc->rb] );
        gen_helper_computecarry(t0, t2, t1, 0);
        tcg_temp_free(t1);
        tcg_temp_free(t2);
    } else {
        TCGv t2 = tcg_temp_new();
        tcg_gen_not_tl( t2, cpu_R[dc->ra] );
        gen_helper_computecarry(t0, t2, cpu_R[dc->rb], 0);
        tcg_temp_free(t2);
    }
    /* Update PSR */
    tcg_gen_shli_tl(t0, t0, 30);
    tcg_gen_andi_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], ~(1<<30));
    tcg_gen_or_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], t0);

    tcg_temp_free(t0);

    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_subi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else {
        tcg_gen_sub_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
}

/* Add, and store carry */
static void dec_cadd(DisasContext *dc)
{
    TCGv t0 = tcg_temp_new();

    if (dc->rhs_immediate) {
        TCGv t1 = tcg_temp_new();
        tcg_gen_movi_tl( t1, dc->imm );
        gen_helper_computecarry(t0, cpu_R[dc->rb], t1, 0);
        tcg_temp_free(t1);
    } else {
        gen_helper_computecarry(t0, cpu_R[dc->ra], cpu_R[dc->rb],0);
    }
    /* Update PSR */
    tcg_gen_shli_tl(t0, t0, 30);
    tcg_gen_andi_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], ~(1<<30));
    tcg_gen_or_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], t0);

    tcg_temp_free(t0);

    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_addi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else {
        tcg_gen_add_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
}

/* Add with carry */
static void dec_addc(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_addi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else {
        tcg_gen_add_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
    TCGv t0 = tcg_temp_new();
    tcg_gen_shri_tl(t0, cpu_SR[SR_PSR], 30);
    tcg_gen_andi_tl(t0, t0, 1);
    tcg_gen_add_tl(cpu_R[dc->rd], cpu_R[dc->rd], t0);
    tcg_temp_free(t0);
}

/* Sub with borrow with carry */
static void dec_subc(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_subi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else {
        tcg_gen_sub_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
    TCGv t0 = tcg_temp_new();
    tcg_gen_shri_tl(t0, cpu_SR[SR_PSR], 30);
    tcg_gen_andi_tl(t0, t0, 1);
    tcg_gen_sub_tl(cpu_R[dc->rd], cpu_R[dc->rd], t0);
    tcg_temp_free(t0);
}

static inline void xtc_loadimm8(DisasContext*dc)
{   /*
    tcg_gen_shli_tl(env_imm,env_imm,8);
    tcg_gen_ori_tl(env_imm,env_imm,dc->imm8);
    */

    // err.. need to do sign extent.
    if (dc->rhs_immediate || (dc->tb_flags&IMM_FLAG))
    {
        qemu_log("Not sign extending, imm %d, flag %d\n", dc->rhs_immediate,(dc->tb_flags&IMM_FLAG));
    } else {
        if (dc->imm8&0x80) {
            dc->imm=0xffffffff;
        }
    }
    dc->imm<<=8;
    dc->imm |= dc->imm8;
    qemu_log("@PC 0x%08x: imm8 %08x %02x\n",dc->pc,dc->imm,dc->imm8);
}


static void dec_limr(DisasContext *dc)
{
    xtc_loadimm8(dc);
    tcg_gen_movi_i32(cpu_R[dc->rd], dc->imm);//cpu_R[dc->rd], env_imm);
}

static void dec_sub(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_subi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else {
        tcg_gen_sub_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
}

static void dec_and(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_andi_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else
        tcg_gen_and_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
}

static void dec_or(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_ori_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else
        tcg_gen_or_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
}

static void dec_xor(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_xori_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else
        tcg_gen_xor_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);
}

static inline void psr_read(DisasContext *dc, TCGv d)
{
    tcg_gen_mov_tl(d, cpu_SR[SR_PSR]);
}

static inline void psr_write(DisasContext *dc, TCGv v)
{
    TCGv t;

    t = tcg_temp_new();
    dc->cpustate_changed = 1;
    /* SUPERVISOR bit is not writable.  */
    tcg_gen_andi_tl(t, v, ~PSR_SUPERVISOR);
    tcg_gen_or_tl(cpu_SR[SR_PSR], cpu_SR[SR_PSR], v);
    tcg_temp_free(t);
}


#if 0
/* 64-bit signed mul, lower result in d and upper in d2.  */
static void t_gen_muls(TCGv d, TCGv d2, TCGv a, TCGv b)
{
    TCGv_i64 t0, t1;

    t0 = tcg_temp_new_i64();
    t1 = tcg_temp_new_i64();

    tcg_gen_ext_i32_i64(t0, a);
    tcg_gen_ext_i32_i64(t1, b);
    tcg_gen_mul_i64(t0, t0, t1);

    tcg_gen_trunc_i64_i32(d, t0);
    tcg_gen_shri_i64(t0, t0, 32);
    tcg_gen_trunc_i64_i32(d2, t0);

    tcg_temp_free_i64(t0);
    tcg_temp_free_i64(t1);
}

/* 64-bit unsigned muls, lower result in d and upper in d2.  */
static void t_gen_mulu(TCGv d, TCGv d2, TCGv a, TCGv b)
{
    TCGv_i64 t0, t1;

    t0 = tcg_temp_new_i64();
    t1 = tcg_temp_new_i64();

    tcg_gen_extu_i32_i64(t0, a);
    tcg_gen_extu_i32_i64(t1, b);
    tcg_gen_mul_i64(t0, t0, t1);

    tcg_gen_trunc_i64_i32(d, t0);
    tcg_gen_shri_i64(t0, t0, 32);
    tcg_gen_trunc_i64_i32(d2, t0);

    tcg_temp_free_i64(t0);
    tcg_temp_free_i64(t1);
}
#endif

/* Multiplier unit.  */
static void dec_mul(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        TCGv t0 = tcg_const_i32(dc->imm);
        tcg_gen_muls2_tl(cpu_R[dc->rd], cpu_SR[SR_Y], cpu_R[dc->rb], t0);
        //t_gen_muls(cpu_R[dc->rd], cpu_SR[SR_Y], cpu_R[dc->rb], t0);
        tcg_temp_free_i32(t0);
    } else {
        //t_gen_muls(cpu_R[dc->rd], cpu_SR[SR_Y], cpu_R[dc->ra], cpu_R[dc->rb]);
        tcg_gen_muls2_tl(cpu_R[dc->rd], cpu_SR[SR_Y], cpu_R[dc->ra], cpu_R[dc->rb]);
    }
}

static void dec_sra(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_sari_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else
        tcg_gen_sar_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);

}
static void dec_srl(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_shri_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else
        tcg_gen_shr_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);

}
static void dec_shl(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    if (dc->rhs_immediate) {
        tcg_gen_shli_tl(cpu_R[dc->rd], cpu_R[dc->rb], dc->imm);
    } else
        tcg_gen_shl_tl(cpu_R[dc->rd], cpu_R[dc->ra], cpu_R[dc->rb]);

}

static void dec_imm(DisasContext *dc)
{
    LOG_DIS("imm %x\n", dc->imm24);
    /*
    tcg_gen_shli_tl(env_imm, env_imm, 24);
    tcg_gen_ori_tl(env_imm, env_imm, dc->imm24);
    */
    dc->imm = (dc->imm24 & 0x800000) ? 0xff800000:0x0;
    dc->imm |= dc->imm24;

    dc->tb_flags |= IMM_FLAG;
    dc->clear_imm = 0;
    qemu_log("@PC 0x%08x: immed, 0x%08x\n", dc->pc, dc->imm);
}

static void xtc_load(DisasContext *dc, TCGMemOp mop)
{
    //RETURN_IF_TARGET_ZERO(dc);
#if 1
    if ((dc->rb%16)==0)
        return;
#endif
    // Loads target rB, source is rD

    if (dc->imm) {
        TCGv t0 = tcg_temp_new();
        tcg_gen_addi_tl( t0, cpu_R[dc->rd], dc->imm );
        tcg_gen_qemu_ld_tl(cpu_R[dc->rb], t0, 0, mop);
#ifdef TRACE_MEMORY
        TCGv tpc = tcg_temp_new();
        TCGv t1 = tcg_temp_new();
        tcg_gen_movi_tl (tpc, dc->pc);
        tcg_gen_movi_tl (t1, mop);
        gen_helper_memoryread( t0, cpu_R[dc->rb], t1, tpc);
        tcg_temp_free(t1);
        tcg_temp_free(tpc);
#endif
        tcg_temp_free(t0);
    } else {
        tcg_gen_qemu_ld_tl(cpu_R[dc->rb], cpu_R[dc->rd], 0, mop);
#ifdef TRACE_MEMORY
        TCGv tpc = tcg_temp_new();
        TCGv t1 = tcg_temp_new();
        tcg_gen_movi_tl (tpc, dc->pc);
        tcg_gen_movi_tl (t1, mop);
        gen_helper_memoryread( cpu_R[dc->rd], cpu_R[dc->rb], t1, tpc);
        tcg_temp_free(t1);
        tcg_temp_free(tpc);

#endif
    }
}

static void dec_load(DisasContext *dc)
{
    xtc_load(dc,MO_TEUL);
}

static void dec_loads(DisasContext *dc)
{
    xtc_load(dc,MO_TEUW);
}

static void dec_loadb(DisasContext *dc)
{
    xtc_load(dc,MO_UB);
}

static void xtc_store(DisasContext *dc, TCGMemOp mop)
{
    // Value is Rb.

    TCGv t0 = tcg_temp_new();
    if (dc->imm) {
        tcg_gen_addi_tl( t0, cpu_R[dc->rd], dc->imm );
        tcg_gen_qemu_st_tl(cpu_R[dc->rb], t0, 0, mop);
#ifdef TRACE_MEMORY
        TCGv tpc = tcg_temp_new();
        TCGv t1 = tcg_temp_new();
        tcg_gen_movi_tl (tpc, dc->pc);
        tcg_gen_movi_tl (t1, mop);
        gen_helper_memorywrite(t0, cpu_R[dc->rb],t1,tpc);
        tcg_temp_free(t1);
        tcg_temp_free(tpc);
#endif
    } else {
#ifdef TRACE_MEMORY
        TCGv tpc = tcg_temp_new();
        TCGv t1 = tcg_temp_new();
        tcg_gen_movi_tl (tpc, dc->pc);
        tcg_gen_movi_tl (t1, mop);
        gen_helper_memorywrite(cpu_R[dc->rd], cpu_R[dc->rb],t1,tpc);
        tcg_temp_free(t1);
        tcg_temp_free(tpc);
#endif
        tcg_gen_qemu_st_tl(cpu_R[dc->rb], cpu_R[dc->rd], 0, mop);
    }
    tcg_temp_free(t0);
}

static void dec_store(DisasContext *dc)
{
    xtc_store(dc,MO_TEUL);
}

static void dec_stores(DisasContext *dc)
{
    xtc_store(dc,MO_TEUW);
}

static void dec_storeb(DisasContext *dc)
{
    xtc_store(dc,MO_UB);
}



static inline void eval_cc(DisasContext *dc, unsigned int cc,
                           TCGv d, TCGv a, TCGv b)
{
    switch (cc) {
    case CC_NONE:
        tcg_gen_setcond_tl(TCG_COND_ALWAYS, d, a, b);
        break;
    case CC_EQ:
        tcg_gen_setcond_tl(TCG_COND_EQ, d, a, b);
        break;
    case CC_NE:
        tcg_gen_setcond_tl(TCG_COND_NE, d, a, b);
        break;
    case CC_LT:
        tcg_gen_setcond_tl(TCG_COND_LT, d, a, b);
        break;
    case CC_LE:
        tcg_gen_setcond_tl(TCG_COND_LE, d, a, b);
        break;
    case CC_GE:
        tcg_gen_setcond_tl(TCG_COND_GE, d, a, b);
        break;
    case CC_GT:
        tcg_gen_setcond_tl(TCG_COND_GT, d, a, b);
        break;
    case CC_ULT:
        tcg_gen_setcond_tl(TCG_COND_LTU, d, a, b);
        break;
    case CC_ULE:
        tcg_gen_setcond_tl(TCG_COND_LEU, d, a, b);
        break;
    case CC_UGE:
        tcg_gen_setcond_tl(TCG_COND_GEU, d, a, b);
        break;
    case CC_UGT:
        tcg_gen_setcond_tl(TCG_COND_GTU, d, a, b);
        break;
    default:
        cpu_abort(CPU(dc->cpu), "Unknown condition code %x.\n", cc);
        break;
    }
}
#if 0
static void eval_cond_jmp(DisasContext *dc, TCGv pc_true, TCGv pc_false)
{
    TCGLabel *l1 = gen_new_label();
    /* Conditional jmp.  */
    tcg_gen_mov_tl(cpu_SR[SR_PC], pc_false);
    tcg_gen_brcondi_tl(TCG_COND_EQ, env_btaken, 0, l1);
    tcg_gen_mov_tl(cpu_SR[SR_PC], pc_true);
    gen_set_label(l1);
}
#endif

static void dec_cmp(DisasContext *dc)
{
    xtc_loadimm8(dc);

    tcg_gen_mov_tl(cpu_cc_src, cpu_R[dc->ra]);
    tcg_gen_mov_tl(cpu_cc_src2, cpu_R[dc->rb]);
}

static void dec_sexts(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    tcg_gen_ext16s_tl( cpu_R[dc->rd], cpu_R[dc->rb]);
}

static void dec_sextb(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    tcg_gen_ext8s_tl( cpu_R[dc->rd], cpu_R[dc->rb]);
}

static void dec_cmpi(DisasContext *dc)
{
    //TCGv tmp0 = tcg_temp_new();

    xtc_loadimm8(dc);

    tcg_gen_mov_tl(cpu_cc_src, cpu_R[dc->ra]);
    tcg_gen_movi_tl(cpu_cc_src2, dc->imm);
    //tcg_gen_subi_tl(cpu_cc_dst, cpu_R[dc->ra], dc->imm);
}

static void dec_addi(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    xtc_loadimm8(dc);

    tcg_gen_addi_tl(cpu_R[dc->rd], cpu_R[dc->ra], dc->imm);
}

static void dec_br(DisasContext *dc)
{
    dc->delayed_branch = 2;
    dc->tb_flags |= D_FLAG;
    dc->conditional_branch = 0;
    int cc = CC_NONE;

    xtc_loadimm8(dc);

    int32_t offset = (int32_t)dc->imm; /* sign-extend.  */

    //tcg_gen_movi_tl(env_btarget, dc->pc + offset);
    dc->jmp = JMP_REL;
    dc->jmp_pc = dc->pc + offset + 2;

    cc = dc->cc;

    if (dc->is_extended) {
        dc->jmp_pc+=2;
    } else {
        if (dc->tb_flags & IMM_FLAG) {
            /* Check CC from previous imm */
            qemu_log("@PC 0x%08x: Using previous insn condition code of %d\n", dc->pc, dc->savecc);
            cc = dc->savecc;
        }
    }


    qemu_log("@PC 0x%08x: Target PC will be 0x%08x, offset 0x%08x, cc %d\n", dc->pc, dc->jmp_pc,
             offset,cc);

    if (cc) {
        /* Need to eval CC right away. */
        dc->conditional_branch = 1;
        tcg_gen_setcond_tl( xtc_get_condition(dc,cc), env_btaken, cpu_cc_src, cpu_cc_src2);
        gen_helper_tracecompare( cpu_env, cpu_cc_src, cpu_cc_src2, env_btaken );
    }

    tcg_gen_movi_tl( env_btarget, dc->jmp_pc);

    if ((dc->rd%16)!=0)
        tcg_gen_movi_tl( cpu_R[dc->rd], dc->pc + (dc->is_extended ? 6 : 4));

}

static void dec_jmp(DisasContext *dc)
{
    dc->delayed_branch = 2;
    dc->tb_flags |= D_FLAG;
    dc->conditional_branch = 0;

    int32_t offset = (int32_t)dc->imm; /* sign-extend.  */

    dc->jmp = JMP_ABS;

    //TCGv tmp0 = tcg_temp_new();
    tcg_gen_addi_tl( env_btarget, cpu_R[dc->rb], offset);
   // tcg_gen_mov_tl( env_btarget, tmp0 );
    //tcg_temp_free(tmp0);

    if (dc->cc) {
        /* Need to eval CC right away. */
        dc->conditional_branch = 1;
        tcg_gen_setcond_tl( xtc_get_condition(dc,dc->cc), env_btaken, cpu_cc_src, cpu_cc_src2);
        gen_helper_tracecompare( cpu_env, cpu_cc_src, cpu_cc_src2, env_btaken );

    } else {
        // Enforce jump.

    }
    if ((dc->rd%16)!=0)
        tcg_gen_movi_tl( cpu_R[dc->rd], dc->pc + (dc->is_extended ? 6 : 4));

}

static void dec_null(DisasContext *dc)
{
    cpu_abort(CPU(dc->cpu),"unknown insn pc=0x%08x opc=0x%08x\n", dc->pc, dc->opcode);
    dc->abort_at_next_insn = 1;
}

static void dec_addr(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);

    //if (dc->rhs_immediate) {
    tcg_gen_addi_tl( cpu_R[dc->rd], cpu_R[dc->rb], dc->imm );
    //} else {
   //     tcg_gen_mov_tl( cpu_R[dc->rd], cpu_R[dc->rb] );
   // }
}

static void dec_rspr(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    switch (dc->rb) {
    case SR_PSR:
        gen_helper_readpsr( cpu_R[dc->rd], cpu_env );
        break;
    case SR_Y:
        tcg_gen_mov_tl( cpu_R[dc->rd], cpu_SR[SR_Y] );
        break;
    default:
        cpu_abort(CPU(dc->cpu),"unknown SPR read access pc=0x%08x opc=0x%08x spr=%d\n", dc->pc, dc->opcode,
                 dc->rb);
        break;
    }
}

static void dec_wspr(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    switch (dc->rd) {
    case SR_PSR:
        gen_helper_writepsr(  cpu_env, cpu_R[dc->rb]);
        break;
    default:
        cpu_abort(CPU(dc->cpu),"unknown SPR write access pc=0x%08x opc=0x%08x spr=%d\n", dc->pc, dc->opcode,
                  dc->rb);
        break;
    }
}

static void dec_copr(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    uint32_t copindex = EXTRACT_FIELD(dc->ir, 8, 9);
    TCGv tmp0 = tcg_temp_new();
    tcg_gen_movi_i32( tmp0, copindex );
    gen_helper_readcop( cpu_R[dc->rd], cpu_env, tmp0, cpu_R[dc->rb] );
    tcg_temp_free(tmp0);
}

static void dec_copw(DisasContext *dc)
{
    RETURN_IF_TARGET_ZERO(dc);
    uint32_t copindex = EXTRACT_FIELD(dc->ir, 8, 9);
    TCGv tmp0 = tcg_temp_new();
    tcg_gen_movi_i32( tmp0, copindex );
    gen_helper_writecop( cpu_env, tmp0, cpu_R[dc->rb], cpu_R[dc->rd] );
    tcg_temp_free(tmp0);
}


static struct decoder_info {
    struct {
        uint32_t bits;
        uint32_t mask;
        uint32_t flags;
    };
    void (*dec)(DisasContext *dc);
} decinfo[] = {
    { DEC_IMM, dec_imm },
    { DEC_ADD, dec_add },
    { DEC_ADDC, dec_addc },
    { DEC_SUB, dec_sub },
    { DEC_SUBB, dec_subc },
    { DEC_AND, dec_and },
    { DEC_OR, dec_or },
    { DEC_XOR, dec_xor },
    { DEC_CMP, dec_cmp },
    { DEC_SHL, dec_shl },
    { DEC_SRL, dec_srl      },
    { DEC_SRA , dec_sra     },
    { DEC_MUL , dec_mul     },
    { DEC_ADDR, dec_addr     },
    { DEC_NOT , dec_null     },
    { DEC_CADD, dec_cadd     },
    { DEC_CSUB, dec_csub     },
    { DEC_STW , dec_store     },
    { DEC_STS , dec_stores     },
    { DEC_STB , dec_storeb     },
    { DEC_STWP, dec_null     },
    { DEC_STSP, dec_null     },
    { DEC_STBP, dec_null     },
    { DEC_LDW , dec_load     },
    { DEC_LDS , dec_loads     },
    { DEC_LDB , dec_loadb     },
    { DEC_LDWP, dec_null     },
    { DEC_LDSP, dec_null     },
    { DEC_LDBP, dec_null     },
    { DEC_COPR, dec_copr     },
    { DEC_COPW, dec_copw     },
    { DEC_BR  , dec_br     },
    { DEC_ADDI, dec_addi     },
    { DEC_CMPI, dec_cmpi      },
    { DEC_LIMR, dec_limr     },
    { DEC_JMP , dec_jmp     },
    { DEC_JMPE, dec_null     },
    { DEC_SEXTB, dec_sextb    },
    { DEC_SEXTS, dec_sexts    },
    { DEC_RSPR , dec_rspr    },
    { DEC_WSPR , dec_wspr   },
    { DEC_RUSR , dec_null   },
    { DEC_WUSR , dec_null   },
    {{0, 0}, dec_null}
};

static int xtc_get_condition(DisasContext*dc, int cc)
{
    int r;
    switch (cc) {
    case CC_NONE:
        r = TCG_COND_ALWAYS;
        break;
    case CC_EQ:
        r = TCG_COND_EQ;
        break;
    case CC_NE:
        r = TCG_COND_NE;
        break;
    case CC_LT:
        r = TCG_COND_LT;
        break;
    case CC_LE:
        r = TCG_COND_LE;
        break;
    case CC_GE:
        r = TCG_COND_GE;
        break;
    case CC_GT:
        r = TCG_COND_GT;
        break;
    case CC_ULT:
        r = TCG_COND_LTU;
        break;
    case CC_ULE:
        r = TCG_COND_LEU;
        break;
    case CC_UGE:
        r = TCG_COND_GEU;
        break;
    case CC_UGT:
        r = TCG_COND_GTU;
        break;
    default:
        cpu_abort(CPU(dc->cpu), "Unknown condition code %x.\n", cc);
        break;
    }
    qemu_log("Cond code: %d -> %d\n", cc, r);
    return r;
}


static inline void decode(DisasContext *dc, uint32_t ir)
{
    int i;
    int handled = 0;

    if (unlikely(qemu_loglevel_mask(CPU_LOG_TB_OP | CPU_LOG_TB_OP_OPT))) {
        tcg_gen_debug_insn_start(dc->pc);
    }

    dc->opcode = ir;//EXTRACT_FIELD(ir, 26, 31);
    dc->rd = EXTRACT_FIELD(ir, 0, 3);
    dc->ra = EXTRACT_FIELD(ir, 0, 3);
    dc->rb = EXTRACT_FIELD(ir, 4, 7);
    dc->imm8 = EXTRACT_FIELD(ir, 4, 11);
    dc->cc = CC_NONE;
    dc->savecc = CC_NONE;
    dc->ir = ir;
    dc->rhs_immediate=0;

    if (dc->is_extended) {
        // d.imm24 := opcode_low(12) & opcode_low(7 downto 0) & opcode_high(14 downto 0);
        // Note that opcodes are inverted.
        dc->imm24 =  EXTRACT_FIELD(ir, 0, 14);
        dc->imm24 += EXTRACT_FIELD(ir, 16, 23)<<15;
        dc->imm24 += EXTRACT_FIELD(ir, 28, 28)<<23;
        dc->cc     = EXTRACT_FIELD(ir, 24, 27);
        dc->savecc = EXTRACT_FIELD(ir, 24, 27); // Saved for later.
        if (dc->isextdreg) {
            // Special D-REG
            dc->rd = EXTRACT_FIELD(ir, 16, 19);
            qemu_log("%08x: target dreg, reg %d, opc %08x\n",dc->pc, dc->rd, ir);
        };
        if (dc->isextimm) {
            // Extended immediate.
            dc->imm<<=8;
            dc->imm += EXTRACT_FIELD(ir, 16, 23);
            //qemu_log("Extended imm: %x\n",dc->imm);
            dc->rhs_immediate=1;
            if (!(dc->tb_flags&IMM_FLAG)) {
                // ok. we need to extend sign.
                if (dc->imm&0x80) {
                    dc->imm |= 0xffffff00;
                }
            }
            dc->tb_flags|=IMM_FLAG;
        }
    }

    // if we have a CC, we need to emit

    TCGLabel *l1 = NULL;
    if (dc->cc != CC_NONE) {
//        l1 =  gen_new_label();
        /* Conditional insn.  */
//        tcg_gen_brcond_tl( xtc_get_condition(dc), cpu_cc_src, cpu_cc_src2, l1);
    }

    /* Large switch for all insns.  */

    for (i = 0; i < ARRAY_SIZE(decinfo); i++) {
        if ((ir & decinfo[i].mask) == decinfo[i].bits) {
            bool needLabel=false;
            //TCGv executed = tcg_temp_new();

            //tcg_gen_movi_tl( executed, 0);

            if ((dc->cc != CC_NONE) && !(decinfo[i].flags&OPCODE_FLAGS_NEEDEVAL)) {
                l1 =  gen_new_label();
                /* Conditional insn.  */
                tcg_gen_brcond_tl( tcg_invert_cond(xtc_get_condition(dc,dc->cc)), cpu_cc_src, cpu_cc_src2, l1);
                needLabel=true;
            }
#ifdef TRACE_INSN
            //tcg_gen_movi_tl( executed, 1);
            TCGv tpc = tcg_const_i32(dc->pc);
            TCGv iir = tcg_const_i32(ir);

            gen_helper_traceinsn( tpc, iir, cpu_R[dc->ra], cpu_R[dc->rb] );

            tcg_temp_free_i32(tpc);
            tcg_temp_free_i32(iir);
#endif
            decinfo[i].dec(dc);
            handled=1;

            if (needLabel) {
                gen_set_label(l1);
            }

            //tcg_temp_free(executed);

            break;
        }
    }

    if (dc->cc != CC_NONE) {
     //   gen_set_label(l1);
    }
    if (!handled) {
        dec_null(dc);
    }
}

static void check_breakpoint(CPUXTCState *env, DisasContext *dc)
{
    CPUState *cs = CPU(xtc_env_get_cpu(env));
    CPUBreakpoint *bp;

    if (unlikely(!QTAILQ_EMPTY(&cs->breakpoints))) {
        QTAILQ_FOREACH(bp, &cs->breakpoints, entry) {
            if (bp->pc == dc->pc) {
                t_gen_raise_exception(dc, EXCP_DEBUG);
                dc->is_jmp = DISAS_UPDATE;
             }
        }
    }
}

/* generate intermediate code for basic block 'tb'.  */
static inline void
gen_intermediate_code_internal(XTCCPU *cpu, TranslationBlock *tb,
                               bool search_pc)
{
    CPUState *cs = CPU(cpu);
    CPUXTCState *env = &cpu->env;
    uint32_t pc_start;
    int j, lj;
    struct DisasContext ctx;
    struct DisasContext *dc = &ctx;
   // uint32_t next_page_start;//, org_flags;
    target_ulong npc;
    int num_insns;
    int max_insns;
    uint32_t insn;
    unsigned pcoffset;

    pc_start = tb->pc;

    dc->cpu = cpu;
    dc->tb = tb;
    dc->tb_flags = env->iflags;
    assert(dc->tb_flags==0);

    dc->is_jmp = DISAS_NEXT;
    dc->jmp = env->btarget;
    dc->delayed_branch = !!(dc->tb_flags & D_FLAG);
    dc->pc = pc_start;
    dc->singlestep_enabled = cs->singlestep_enabled;
    dc->cpustate_changed = 0;
    dc->abort_at_next_insn = 0;
    dc->nr_nops = 0;
    dc->imm = 0;//env->imm; // ???

    qemu_log("TB start: flags %08x, pc 0x%08x, imm 0x%08x\n", dc->tb_flags, dc->pc, dc->imm);

    //dc->delayed_branch = 0;

    if (pc_start & 1) {
        cpu_abort(cs, "XTC: unaligned PC=%x\n", pc_start);
    }

    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
#if !SIM_COMPAT
        qemu_log("--------------\n");
        log_cpu_state(CPU(cpu), 0);
#endif
    }

    //next_page_start = (pc_start & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE;
    lj = -1;
    num_insns = 0;
    max_insns = tb->cflags & CF_COUNT_MASK;
    if (max_insns == 0)
        max_insns = CF_COUNT_MASK;

    gen_tb_start(tb);
    do
    {
#if SIM_COMPAT
        if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
            tcg_gen_movi_tl(cpu_SR[SR_PC], dc->pc);
            gen_helper_debug();
        }
#endif
        check_breakpoint(env, dc);

        if (search_pc) {
            j = tcg_op_buf_count();
            if (lj < j) {
                lj++;
                while (lj < j)
                    tcg_ctx.gen_opc_instr_start[lj++] = 0;
            }
            tcg_ctx.gen_opc_pc[lj] = dc->pc;
            tcg_ctx.gen_opc_instr_start[lj] = 1;
                        tcg_ctx.gen_opc_icount[lj] = num_insns;
        }

        /* Pretty disas.  */
        LOG_DIS("%8.8x:\t", dc->pc);

        qemu_log("@PC 0x%08x: start imm 0x%08x, imflag %d\n", dc->pc,dc->imm,
                 dc->tb_flags&IMM_FLAG);

        if (num_insns + 1 == max_insns && (tb->cflags & CF_LAST_IO))
            gen_io_start();

        dc->clear_imm = 1;
        dc->is_extended = 0;

        /* Load first word */
        insn = cpu_lduw_code(env, dc->pc);
        pcoffset = 2;
        if (insn&0x8000) {
            /* Extended opcode */
            insn |= cpu_lduw_code(env, dc->pc+pcoffset) << 16;
            LOG_DIS("%8.8x\n", insn);
            pcoffset+=2;
            dc->is_extended=1;
            dc->isextdreg=0;
            dc->isextimm=0;
            unsigned extc = EXTRACT_FIELD(insn,28,30);
            switch(extc) {
            case 0:
                break;
            case 1:
            case 3:
                cpu_abort(CPU(cpu), "Invalid extended insn, PC: 0x%08x\n", dc->pc);
            case 2:
                dc->isextdreg=1;
                break;
            case 4:
                dc->isextimm=1;
            default:
                break;
            }
        } else {
            LOG_DIS("%4.4x\n", insn);
        }
#if 0
        if (dc->is_extended && (dc->tb_flags&IMM_FLAG)) {
            cpu_abort(CPU(cpu),"Extended insn found after imm. "
                      "This is invalid. PC: 0x%08x", dc->pc);
        }

#endif
        decode(dc, insn);
        qemu_log("@PC 0x%08x: clearimm %d\n", dc->pc,dc->clear_imm);
        /* IMMediate processing */
        if (dc->clear_imm) {
            dc->tb_flags &= ~IMM_FLAG;
            //tcg_gen_movi_i32( env_imm, 0);
            dc->imm = 0;
        }
        /* Clear all zero regs */

        //tcg_gen_movi_i32( cpu_R[0], 0);
        //tcg_gen_movi_i32( cpu_R[16], 0);



        num_insns++;

        if (dc->delayed_branch) {
            qemu_log("@PC 0x%08x: delayed branch %d\n", dc->pc,dc->delayed_branch);
            dc->delayed_branch--;
            if (dc->delayed_branch==0) {
                dc->is_jmp=DISAS_UPDATE;
                /* Clear the delay slot flag.  */
                dc->tb_flags &= ~D_FLAG;

                if (dc->is_extended) {
                    cpu_abort(CPU(cpu),"Extended insn found in delay slot. "
                              "This is invalid. PC: 0x%08x", dc->pc);
                }

                if (dc->conditional_branch) {
                    TCGLabel *l1 = gen_new_label();
                    tcg_gen_brcondi_tl( TCG_COND_EQ, env_btaken, 0, l1);
                    tcg_gen_mov_tl(cpu_SR[SR_PC], env_btarget);
                    //gen_helper_branchtaken(cpu_env,env_btarget);
                    t_sync_flags(dc);
                    tcg_gen_exit_tb(0);
                    gen_set_label(l1);
                    tcg_gen_movi_tl(cpu_SR[SR_PC], dc->pc + pcoffset);
                    //dc->is_jmp = DISAS_NEXT;
                } else {
                    if (dc->jmp == JMP_ABS) {
                        //gen_helper_branchtaken(cpu_env,env_btarget);
                        tcg_gen_mov_tl(cpu_SR[SR_PC], env_btarget);
                    } else {
                        //dc->is_jmp=DISAS_UPDATE;//TB_JUMP;
                        tcg_gen_mov_tl(cpu_SR[SR_PC], env_btarget);
                        //tcg_gen_movi_tl(cpu_SR[SR_PC], dc->jmp_pc);
                        //tcg_gen_movi_tl(cpu_SR[SR_PC], dest);
                        //tcg_gen_exit_tb(0);

                        //gen_goto_tb(dc, 0, dc->jmp_pc);
                    }
                }
            }
        }

        dc->pc += pcoffset;

        if (cs->singlestep_enabled) {
            break;
        }

        if (dc->abort_at_next_insn)
            break;

    } while (!dc->is_jmp && !dc->cpustate_changed
            /* && !tcg_op_buf_full()*/
             && !singlestep
             /* && (dc->pc < next_page_start)*/
             && num_insns < max_insns);

    qemu_log("@PC 0x%08x: Exiting tb, jmp %d \n",dc->pc, dc->is_jmp);
    // Save pc ?...
    //tcg_gen_movi_tl(cpu_SR[SR_PC], dc->pc);

    npc = dc->pc;

    if (tb->cflags & CF_LAST_IO)
        gen_io_end();

    t_sync_flags(dc);

    if (unlikely(cs->singlestep_enabled)) {
        TCGv_i32 tmp = tcg_const_i32(EXCP_DEBUG);

        if (dc->is_jmp != DISAS_JUMP) {
            tcg_gen_movi_tl(cpu_SR[SR_PC], npc);
        }
        gen_helper_raise_exception(cpu_env, tmp);
        tcg_temp_free_i32(tmp);
    } else {
        switch(dc->is_jmp) {
            case DISAS_NEXT:
                //gen_goto_tb(dc, 1, npc);
                tcg_gen_movi_tl(cpu_SR[SR_PC], npc);
                tcg_gen_exit_tb(0);

                break;
            default:
            case DISAS_JUMP:
            case DISAS_UPDATE:
                /* indicate that the hash table must be used
                   to find the next TB */
                tcg_gen_exit_tb(0);
                break;
            case DISAS_TB_JUMP:
                /* nothing more to generate */
                break;
        }
    }
    gen_tb_end(tb, num_insns);

    if (search_pc) {
        j = tcg_op_buf_count();
        lj++;
        while (lj <= j)
            tcg_ctx.gen_opc_instr_start[lj++] = 0;
    } else {
        tb->size = dc->pc - pc_start;
                tb->icount = num_insns;
    }

#ifdef DEBUG_DISAS
#if !SIM_COMPAT
    if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)) {
        qemu_log("\n");
#if DISAS_GNU
        log_target_disas(env, pc_start, dc->pc - pc_start, 0);
#endif
        qemu_log("\nisize=%d osize=%d\n",
                 dc->pc - pc_start, tcg_op_buf_count());
    }
#endif
#endif
    assert(!dc->abort_at_next_insn);
    assert (!(dc->tb_flags&D_FLAG));
    assert (!(dc->tb_flags&IMM_FLAG));
    assert (!dc->delayed_branch);
    assert (dc->imm == 0);

}

void gen_intermediate_code (CPUXTCState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(xtc_env_get_cpu(env), tb, false);
}

void gen_intermediate_code_pc (CPUXTCState *env, struct TranslationBlock *tb)
{
    gen_intermediate_code_internal(xtc_env_get_cpu(env), tb, true);
}

void xtc_cpu_dump_state(CPUState *cs, FILE *f, fprintf_function cpu_fprintf,
                       int flags)
{
    XTCCPU *cpu = XTC_CPU(cs);
    CPUXTCState *env = &cpu->env;
    int i;

    if (!env || !f)
        return;

    cpu_fprintf(f, "IN: PC=%x %s\n",
                env->sregs[SR_PC], lookup_symbol(env->sregs[SR_PC]));
    cpu_fprintf(f, "psr=%x spsr=%x y=%x\n",
                env->sregs[SR_PSR], env->sregs[SR_SPSR], env->sregs[SR_Y]
               );
/*    cpu_fprintf(f, "btaken=%d btarget=%x mode=%s(saved=%s) eip=%d ie=%d\n",
                env->btaken, env->btarget,
                (env->sregs[SR_MSR] & MSR_UM) ? "user" : "kernel",
                (env->sregs[SR_MSR] & MSR_UMS) ? "user" : "kernel",
                (env->sregs[SR_MSR] & MSR_EIP),
                (env->sregs[SR_MSR] & MSR_IE));
  */
    for (i = 0; i < 16; i++) {
        cpu_fprintf(f, "r%2.2d=%8.8x ", i, env->regs[i]);
        if ((i + 1) % 4 == 0)
            cpu_fprintf(f, "\n");
    }
    cpu_fprintf(f, "\n\n");
    dumptrace();
}

XTCCPU *cpu_xtc_init(const char *cpu_model)
{
    XTCCPU *cpu;

    cpu = XTC_CPU(object_new(TYPE_XTC_CPU));

    object_property_set_bool(OBJECT(cpu), true, "realized", NULL);

    return cpu;
}

void xtc_tcg_init(void)
{
    int i;

    cpu_env = tcg_global_reg_new_ptr(TCG_AREG0, "env");

    env_debug = tcg_global_mem_new(TCG_AREG0,
                                   offsetof(CPUXTCState, debug),
                                   "debug0");
    env_iflags = tcg_global_mem_new(TCG_AREG0,
                                    offsetof(CPUXTCState, iflags),
                                    "iflags");
    env_imm = tcg_global_mem_new(TCG_AREG0,
                                 offsetof(CPUXTCState, imm),
                                 "imm");
    env_btarget = tcg_global_mem_new(TCG_AREG0,
                                     offsetof(CPUXTCState, btarget),
                                     "btarget");
    env_btaken = tcg_global_mem_new(TCG_AREG0,
                                    offsetof(CPUXTCState, btaken),
                                    "btaken");

    cpu_cc_src = tcg_global_mem_new(TCG_AREG0,
                                    offsetof(CPUXTCState, cc_src),
                                    "cc_src");

    cpu_cc_src2 = tcg_global_mem_new(TCG_AREG0,
                                    offsetof(CPUXTCState, cc_src2),
                                    "cc_src2");

    for (i = 0; i < ARRAY_SIZE(cpu_R); i++) {
        cpu_R[i] = tcg_global_mem_new(TCG_AREG0,
                                      offsetof(CPUXTCState, regs[i]),
                                      regnames[i % 16]);
    }
    for (i = 0; i < ARRAY_SIZE(cpu_SR); i++) {
        cpu_SR[i] = tcg_global_mem_new(TCG_AREG0,
                                       offsetof(CPUXTCState, sregs[i]),
                                       special_regnames[i]);
    }
}

void restore_state_to_opc(CPUXTCState *env, TranslationBlock *tb, int pc_pos)
{
    env->sregs[SR_PC] = tcg_ctx.gen_opc_pc[pc_pos];
}
