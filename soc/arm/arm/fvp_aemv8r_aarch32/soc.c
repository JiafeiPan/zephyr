/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cache.h>
#include <zephyr/arch/arm/aarch32/cortex_a_r/cmsis.h>

void z_arm_platform_init(void)
{
	if (IS_ENABLED(CONFIG_ICACHE)) {
		if (!(__get_SCTLR() & SCTLR_I_Msk)) {
			arch_icache_all(K_CACHE_INVD);
			arch_icache_enable();
		}
	}

	if (IS_ENABLED(CONFIG_DCACHE)) {
		if (!(__get_SCTLR() & SCTLR_C_Msk)) {
			arch_dcache_all(K_CACHE_INVD);
			arch_dcache_enable();
		}
	}
}
