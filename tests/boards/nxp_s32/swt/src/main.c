/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/watchdog.h>

#define DEVICE_FOR_DT_COMPAT(n, compat) DEVICE_DT_GET(DT_INST(n, compat)),
#define GET_DEVICE_FOR_DT_COMPAT(compat) \
	COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(compat), \
			(LISTIFY(DT_NUM_INST_STATUS_OKAY(compat), \
				DEVICE_FOR_DT_COMPAT, (), compat)), ())

#define SWT_FEED_TRIES	5
#define SWT_MAX_WINDOW	1000
#define SWT_TIMEOUT		K_MSEC(1100)

static volatile int swt_interrupted_flag;
static int swt_feed_flag;

static const struct device * const devices[] = {
	GET_DEVICE_FOR_DT_COMPAT(nxp_s32_swt)
};

static void swt_callback(const struct device *dev, int channel_id)
{
	swt_interrupted_flag = 1;
	zassert_equal(SWT_FEED_TRIES, swt_feed_flag,
			"%d: Invalid number of feeding (expected: %d)",
			swt_feed_flag, SWT_FEED_TRIES);
}

static void swt_test_instance(const struct device *dev)
{
	int err, channel_id;

	zassert_not_null(dev, "Cannot get SWT device");

	struct wdt_timeout_cfg swt_config = {
		.window.min = 0U,
		.window.max = SWT_MAX_WINDOW,
		.callback = swt_callback,
	};

	channel_id = wdt_install_timeout(dev, &swt_config);
	zassert_true(channel_id >= 0, "Watchdog install error\n");

	err = wdt_setup(dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
	zassert_ok(err, "Watchdog setup error");

	TC_PRINT("Feeding watchdog %d times\n", SWT_FEED_TRIES);
	swt_feed_flag = 0;
	swt_interrupted_flag = 0;
	for (int i = 0; i < SWT_FEED_TRIES; ++i) {
		TC_PRINT("Feeding %d\n", i+1);
		wdt_feed(dev, channel_id);
		swt_feed_flag++;
		k_sleep(K_MSEC(500));
	}

	k_timeout_t timeout = SWT_TIMEOUT;
	uint64_t start_time = k_uptime_ticks();

	while (swt_interrupted_flag == 0) {
		if (k_uptime_ticks() - start_time >= timeout.ticks) {
			break;
		}
	}

	zassert_equal(swt_interrupted_flag, 1, "SWT did not expire");

	err = wdt_disable(dev);
	zassert_equal(err, 0, "Disable watchdog error\n");
}

ZTEST(nxp_s32_swt_test, test_swt_interrupt)
{
	for (int i = 0; i < ARRAY_SIZE(devices); i++) {
		TC_PRINT("Testing on device %p\n", devices[i]);
		swt_test_instance(devices[i]);
	};
}

ZTEST_SUITE(nxp_s32_swt_test, NULL, NULL, NULL, NULL, NULL);
