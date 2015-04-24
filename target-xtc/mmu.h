/*
 *  XThunderCore MMU emulation for qemu.
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

#define TLB_ENTRIES    16

struct xtc_mmu
{
    /* Data and tag brams.  */
    uint32_t rams[2][TLB_ENTRIES];
    /* We keep a separate ram for the tids to avoid the 48 bit tag width.  */
    uint8_t tids[TLB_ENTRIES];
    /* Control flops.  */
    uint32_t regs[8];

    int c_mmu;
    int c_mmu_tlb_access;
    int c_mmu_zones;
};

struct xtc_mmu_lookup
{
    uint32_t paddr;
    uint32_t vaddr;
    unsigned int size;
    unsigned int idx;
    int prot;
    enum {
        ERR_PROT, ERR_MISS, ERR_HIT
    } err;
};

void mmu_flip_um(CPUXTCState *env, unsigned int um);
unsigned int mmu_translate(struct xtc_mmu *mmu,
                           struct xtc_mmu_lookup *lu,
                           target_ulong vaddr, int rw, int mmu_idx);
uint32_t mmu_read(CPUXTCState *env, uint32_t rn);
void mmu_write(CPUXTCState *env, uint32_t rn, uint32_t v);
void mmu_init(struct xtc_mmu *mmu);
