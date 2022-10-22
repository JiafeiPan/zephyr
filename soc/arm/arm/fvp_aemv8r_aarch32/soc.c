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
			cache_instr_all(K_CACHE_INVD);
			cache_instr_enable();
		}
	}

	if (IS_ENABLED(CONFIG_DCACHE)) {
		if (!(__get_SCTLR() & SCTLR_C_Msk)) {
			cache_data_all(K_CACHE_INVD);
			cache_data_enable();
		}
	}
}
