/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARMv8 AArch32 L1-cache maintenance operations.
 *
 * This module implement the cache API for ARMv8 AArch32 cores using CMSIS.
 * Only L1-cache maintenance operations is supported.
 */

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/arch/arm/aarch32/cortex_a_r/cmsis.h>

/* Cache Type Register */
#define	CTR_DMINLINE_SHIFT	16
#define	CTR_DMINLINE_MASK	BIT_MASK(4)

static size_t dcache_line_size;

/**
 * @brief Get the smallest D-cache line size.
 *
 * Get the smallest D-cache line size of all the data and unified caches that
 * the processor controls.
 */
size_t arch_dcache_line_size_get(void)
{
	uint32_t val;
	uint32_t dminline;

	if (!dcache_line_size) {
		val = read_sysreg(ctr);
		dminline = (val >> CTR_DMINLINE_SHIFT) & CTR_DMINLINE_MASK;
		/* Log2 of the number of words */
		dcache_line_size = 2 << dminline;
	}

	return dcache_line_size;
}

/**
 * @brief Write-back/Invalidate/Write-back+Invalidate all D-cache.
 *
 * @param op cache operation to perform
 * @return int 0 if success; otherwise error code
 */
int arch_dcache_all(int op)
{
	int err = 0;

	if (op == K_CACHE_INVD) {
		L1C_InvalidateDCacheAll();
	} else if (op == K_CACHE_WB_INVD) {
		L1C_CleanInvalidateDCacheAll();
	} else if (op == K_CACHE_WB) {
		L1C_CleanDCacheAll();
	} else {
		err = -ENOTSUP;
	}

	return err;
}

/**
 * @brief Write-back/Invalidate/Write-back+Invalidate D-cache lines by VA.
 *
 * Apply the D-cache maintenance operation to PoC.
 *
 * @param addr start address (virtual address)
 * @param size size of memory to apply operation
 * @param op cache operation to perform
 * @return int 0 if success; otherwise error code
 */
int arch_dcache_range(void *addr, size_t size, int op)
{
	size_t line_size;
	uintptr_t start_addr = (uintptr_t)addr;
	uintptr_t end_addr = start_addr + size;

	if (op != K_CACHE_INVD && op != K_CACHE_WB && op != K_CACHE_WB_INVD) {
		return -ENOTSUP;
	}

	/* Align address to line size */
	line_size = arch_dcache_line_size_get();
	start_addr &= ~(line_size - 1);

	while (start_addr < end_addr) {
		if (op == K_CACHE_INVD) {
			L1C_InvalidateDCacheMVA(addr);
		} else if (op == K_CACHE_WB_INVD) {
			L1C_CleanInvalidateDCacheMVA(addr);
		} else if (op == K_CACHE_WB) {
			L1C_CleanDCacheMVA(addr);
		}
		start_addr += line_size;
	}

	return 0;
}

/**
 * @brief Invalidate all I-cache.
 *
 * @param op cache operation to perform (only K_CACHE_INVD)
 * @return int 0 if success; otherwise error code
 */
int arch_icache_all(int op)
{
	int err = 0;

	if (op == K_CACHE_INVD) {
		L1C_InvalidateICacheAll();
	} else {
		err = -ENOTSUP;
	}

	return err;
}

/**
 * @brief Write-back/Invalidate/Write-back+Invalidate I-cache lines by VA.
 *
 * Operation not supported on this architecture.
 *
 * @param addr start address (virtual address)
 * @param size size of memory to apply operation
 * @param op cache operation to perform
 * @return int 0 if success; otherwise error code
 */
int arch_icache_range(void *addr, size_t size, int op)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);
	ARG_UNUSED(op);

	return -ENOTSUP;
}

/**
 * @brief Enable D-cache.
 *
 * All D-cache is invalidated before enabled.
 */
void arch_dcache_enable(void)
{
	uint32_t val;

	arch_dcache_all(K_CACHE_INVD);

	val = __get_SCTLR();
	val |= SCTLR_C_Msk;
	__DSB();
	__set_SCTLR(val);
	__ISB();
}

/**
 * @brief Disable D-cache.
 *
 * All D-cache is clean and invalidated after disabled.
 */
void arch_dcache_disable(void)
{
	uint32_t val;

	val = __get_SCTLR();
	val &= ~SCTLR_C_Msk;
	__DSB();
	__set_SCTLR(val);
	__ISB();

	arch_dcache_all(K_CACHE_WB_INVD);
}

/**
 * @brief Enable I-cache.
 *
 * All I-cache is invalidated before enabled.
 */
void arch_icache_enable(void)
{
	uint32_t val;

	arch_icache_all(K_CACHE_INVD);

	val = __get_SCTLR();
	val |= SCTLR_I_Msk;
	__DSB();
	__set_SCTLR(val);
	__ISB();
}

/**
 * @brief Disable I-cache.
 */
void arch_icache_disable(void)
{
	uint32_t val;

	val = __get_SCTLR();
	val &= ~SCTLR_I_Msk;
	__DSB();
	__set_SCTLR(val);
	__ISB();
}
