#include "cpu.h"

/* rw - 0 = read, 1 = write, 2 = fetch.  */
unsigned int mmu_translate(struct xtc_mmu *mmu,
                           struct xtc_mmu_lookup *lu,
                           target_ulong vaddr, int rw, int mmu_idx)
{
    abort();
    return 1;
}
#if 0
void mmu_init(struct xtc_mmu *mmu)
{
    abort();
}
#endif
uint32_t mmu_read(CPUXTCState *env, uint32_t rn)
{
    abort();
}

void mmu_write(CPUXTCState *env, uint32_t rn, uint32_t v)
{
    abort();
}
