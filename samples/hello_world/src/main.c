/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

static __aligned(64) uint8_t mytestbuf[64] __used __attribute__((__section__(".nocache")));

void main(void)
{
	printk("writing\n");
	for (int i = 0; i < 64; i++) {
		mytestbuf[i] = i;
	}
	printk("reading\n");
	for (int i = 0; i < 64; i++) {
		printk("%d ", mytestbuf[i]);
	}
	printk("\ndone\n");
}
