/*
 * Copyright 2022, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/drivers/pinctrl.h>
#include "fsl_power.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/*!< Power down all unnecessary blocks */
#define NODE_ID DT_INST(0, nxp_pdcfg_power)
#define EXCLUDE_FROM_DEEPSLEEP ((const uint32_t[]) \
					DT_PROP_OR(NODE_ID, deep_sleep_config, {}))

#define EXCLUDE_FROM_DEEP_POWERDOWN ((const uint32_t[]){0, 0, 0, 0})

/* System clock frequency. */
extern uint32_t SystemCoreClock;

#define FLEXSPI_DEVICE DT_NODELABEL(flexspi)
#if DT_NODE_HAS_STATUS(FLEXSPI_DEVICE, okay)

/*
 * Declare pin configuration state for flexspi pins.
 * These pins will be reconfigured for power savings
 */
PINCTRL_DT_DEFINE(FLEXSPI_DEVICE);
const struct pinctrl_dev_config *flexspi_pincfg = PINCTRL_DT_DEV_CONFIG_GET(FLEXSPI_DEVICE);
const struct pinctrl_state *flexspi_pins_default_state;
const struct pinctrl_state *flexspi_pins_sleep_state;
/* Flag indicates if we can reconfigure flexspi pins for additional power savings */
static bool flexspi_pin_reconfig;

#endif

#define FLEXSPI2_DEVICE DT_NODELABEL(flexspi2)
#if DT_NODE_HAS_STATUS(FLEXSPI2_DEVICE, okay)

/*
 * Declare pin configuration state for flexspi2 pins.
 * These pins will be reconfigured for power savings
 */
PINCTRL_DT_DEFINE(FLEXSPI2_DEVICE);
const struct pinctrl_dev_config *flexspi2_pincfg = PINCTRL_DT_DEV_CONFIG_GET(FLEXSPI2_DEVICE);
const struct pinctrl_state *flexspi2_pins_default_state;
const struct pinctrl_state *flexspi2_pins_sleep_state;
/* Flag indicates if we can reconfigure flexspi pins for additional power savings */
static bool flexspi2_pin_reconfig;

#endif

static uint32_t isp_pin[3];

__ramfunc void set_deepsleep_pin_config(void)
{
	/* Backup Pin configuration. */
	isp_pin[0] = IOPCTL->PIO[1][15];
	isp_pin[1] = IOPCTL->PIO[3][28];
	isp_pin[2] = IOPCTL->PIO[3][29];

	/* Disable ISP Pin pull-ups and input buffers to avoid current leakage */
	IOPCTL->PIO[1][15] = 0;
	IOPCTL->PIO[3][28] = 0;
	IOPCTL->PIO[3][29] = 0;

#if DT_NODE_HAS_STATUS(FLEXSPI_DEVICE, okay)
	/* Apply flexspi pinctrl state for Sleep state if defined through device tree */
	if (flexspi_pin_reconfig) {
		pinctrl_configure_pins(flexspi_pins_sleep_state->pins,
				       flexspi_pins_sleep_state->pin_cnt,
				       PINCTRL_REG_NONE);
	}
#endif

#if DT_NODE_HAS_STATUS(FLEXSPI2_DEVICE, okay)
	/* Apply flexspi2 pinctrl state for Sleep state if defined through device tree */
	if (flexspi2_pin_reconfig) {
		pinctrl_configure_pins(flexspi2_pins_sleep_state->pins,
				       flexspi2_pins_sleep_state->pin_cnt,
				       PINCTRL_REG_NONE);
	}
#endif

}

__ramfunc void restore_deepsleep_pin_config(void)
{
	/* Restore the Pin configuration. */
	IOPCTL->PIO[1][15] = isp_pin[0];
	IOPCTL->PIO[3][28] = isp_pin[1];
	IOPCTL->PIO[3][29] = isp_pin[2];

#if DT_NODE_HAS_STATUS(FLEXSPI_DEVICE, okay)
	/* Apply flexspi pinctrl state for default state */
	if (flexspi_pin_reconfig) {
		pinctrl_configure_pins(flexspi_pins_default_state->pins,
				       flexspi_pins_default_state->pin_cnt,
				       PINCTRL_REG_NONE);
	}
#endif

#if DT_NODE_HAS_STATUS(FLEXSPI2_DEVICE, okay)
	/* Apply flexspi2 pinctrl state for default state */
	if (flexspi2_pin_reconfig) {
		pinctrl_configure_pins(flexspi2_pins_default_state->pins,
				       flexspi2_pins_default_state->pin_cnt,
				       PINCTRL_REG_NONE);
	}
#endif
}

/* Invoke Low Power/System Off specific Tasks */
__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	/* FIXME: When this function is entered the Kernel has disabled
	 * interrupts using BASEPRI register. This is incorrect as it prevents
	 * waking up from any interrupt which priority is not 0. Work around the
	 * issue and disable interrupts using PRIMASK register as recommended
	 * by ARM.
	 */

	/* Set PRIMASK */
	__disable_irq();

	/* Set BASEPRI to 0 */
	irq_unlock(0);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		POWER_EnterSleep();
		break;
	case PM_STATE_SUSPEND_TO_IDLE:
		set_deepsleep_pin_config();
		POWER_EnterDeepSleep(EXCLUDE_FROM_DEEPSLEEP);
		restore_deepsleep_pin_config();
		break;
	case PM_STATE_SOFT_OFF:
		set_deepsleep_pin_config();
		POWER_EnterDeepPowerDown(EXCLUDE_FROM_DEEP_POWERDOWN);
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	/* Clear PRIMASK */
	__enable_irq();
}

/* Initialize power system */
static int rt5xx_power_init(const struct device *dev)
{
	int ret = 0;

#if DT_NODE_HAS_STATUS(FLEXSPI_DEVICE, okay)
	flexspi_pin_reconfig = false;

	ret = pinctrl_lookup_state(flexspi_pincfg,
				   PINCTRL_STATE_DEFAULT,
				   &flexspi_pins_default_state);

	if (ret == 0) {
		/*
		 * Default flexspi pin configuration is available, check if flexspi pin
		 * configuration for sleep state is available.
		 */
		ret = pinctrl_lookup_state(flexspi_pincfg,
					   PINCTRL_STATE_SLEEP,
					   &flexspi_pins_sleep_state);
		if (ret == 0) {
			/* Flexspi pin configuation is available */
			flexspi_pin_reconfig = true;
		}
	}
#endif

#if DT_NODE_HAS_STATUS(FLEXSPI2_DEVICE, okay)
	flexspi2_pin_reconfig = false;

	ret = pinctrl_lookup_state(flexspi2_pincfg,
				   PINCTRL_STATE_DEFAULT,
				   &flexspi2_pins_default_state);

	if (ret == 0) {
		/*
		 * Default flexspi2 pin configuration is available, check if flexspi2 pin
		 * configuration for sleep state is available.
		 */
		ret = pinctrl_lookup_state(flexspi2_pincfg,
					   PINCTRL_STATE_SLEEP,
					   &flexspi2_pins_sleep_state);
		if (ret == 0) {
			/* Flexspi pin configuation is available */
			flexspi2_pin_reconfig = true;
		}
	}
#endif

	/* This function is called to set vddcore low voltage detection
	 * falling trip voltage, this is not impacting the voltage in anyway.
	 */
	POWER_SetLdoVoltageForFreq(SystemCoreClock, 0);

#if CONFIG_REGULATOR
	/* Indicate to power library that PMIC is used. */
	POWER_UpdatePmicRecoveryTime(1);
#endif

	return ret;
}

SYS_INIT(rt5xx_power_init, PRE_KERNEL_2, 0);
