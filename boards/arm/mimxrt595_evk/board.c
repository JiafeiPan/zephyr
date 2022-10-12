/*
 * Copyright 2022,  NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/dt-bindings/regulator/pca9420_i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/pm/policy.h>
#include "fsl_power.h"

#if CONFIG_REGULATOR && CONFIG_PM
#define NODE_SW1	DT_NODELABEL(pca9420_sw1)
#define NODE_SW2	DT_NODELABEL(pca9420_sw2)
#define NODE_LDO1	DT_NODELABEL(pca9420_ldo1)
#define NODE_LDO2	DT_NODELABEL(pca9420_ldo2)
const struct device *sw1 = DEVICE_DT_GET(NODE_SW1);
const struct device *sw2 = DEVICE_DT_GET(NODE_SW2);
const struct device *ldo1 = DEVICE_DT_GET(NODE_LDO1);
const struct device *ldo2 = DEVICE_DT_GET(NODE_LDO2);

/*
 * This function sets up the PMIC operation in different low power modes.
 */
static int setup_pmic_modes(void)
{
	int ret = 0;
	int volt;

	/*
	 * Initial mode is set to Mode0 through the device tree property.
	 * Also the regulators are configured to be enabled in the initial mode.
	 */

	/* SW1 setting in Run mode. If no device tree property is specified for
	 * Run mode then SW1 is enabled at 1V.
	 */
	ret = regulator_set_mode(sw1, PCA9420_MODECFG0_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_SW1, regulator_state_active), regulator_off)
	ret = regulator_mode_disable(sw1, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(sw1, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_SW1, regulator_state_active),
				regulator_microvolt,
				1000000);
	ret = regulator_set_voltage(sw1, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* SW2 setting in Run mode. If no device tree property is specified for
	 * Run mode then SW2 is enabled at 1.8V.
	 */
	ret = regulator_set_mode(sw2, PCA9420_MODECFG0_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_SW2, regulator_state_active), regulator_off)
	ret = regulator_mode_disable(sw2, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(sw2, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_SW2, regulator_state_active),
				regulator_microvolt,
				1800000);
	ret = regulator_set_voltage(sw2, volt, volt);

	if (ret != 0) {
		return ret;
	}
#endif

	/* LDO1 setting in Run mode. If no device tree property is specified for
	 * Run mode then LDO1 is enabled at 1.8V.
	 */
	ret = regulator_set_mode(ldo1, PCA9420_MODECFG0_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_LDO1, regulator_state_active), regulator_off)
	ret = regulator_mode_disable(ldo1, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(ldo1, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_LDO1, regulator_state_active),
				regulator_microvolt,
				1800000);
	ret = regulator_set_voltage(ldo1, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* LDO2 setting in Run mode. If no device tree property is specified for
	 * Run mode then SW1 is enabled at 3.3V.
	 */
	ret = regulator_set_mode(ldo2, PCA9420_MODECFG0_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_LDO2, regulator_state_active), regulator_off)
	ret = regulator_mode_disable(ldo2, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(ldo2, PCA9420_MODECFG0_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_LDO2, regulator_state_active),
				regulator_microvolt,
				3300000);
	ret = regulator_set_voltage(ldo2, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* SW1 setting in Deep Sleep mode. If no device tree property is specified for
	 * Deep Sleep mode then SW1 is enabled at 0.6V.
	 */
	ret = regulator_set_mode(sw1, PCA9420_MODECFG1_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_SW1, regulator_state_suspend_to_idle), regulator_off)
	ret = regulator_mode_disable(sw1, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(sw1, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_SW1, regulator_state_suspend_to_idle),
				regulator_microvolt,
				600000);
	ret = regulator_set_mode_voltage(sw1, PCA9420_MODECFG1_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* SW2 setting in Deep Sleep mode. If no device tree property is specified for
	 * Deep Sleep mode then SW1 is enabled at 1.8V.
	 */
	ret = regulator_set_mode(sw2, PCA9420_MODECFG1_PIN);
	if (ret != 0) {
		return ret;
	}
#if DT_PROP(DT_CHILD(NODE_SW2, regulator_state_suspend_to_idle), regulator_off)
	ret = regulator_mode_disable(sw2, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(sw2, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_SW2, regulator_state_suspend_to_idle),
				regulator_microvolt,
				1800000);
	ret = regulator_set_mode_voltage(sw2, PCA9420_MODECFG1_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* LDO1 setting in Deep Sleep mode. If no device tree property is specified for
	 * Deep Sleep mode then LDO1 is enabled at 1.8V.
	 */
	ret = regulator_set_mode(ldo1, PCA9420_MODECFG1_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_LDO1, regulator_state_suspend_to_idle), regulator_off)
	ret = regulator_mode_disable(ldo1, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(ldo1, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_LDO1, regulator_state_suspend_to_idle),
				regulator_microvolt,
				1800000);
	ret = regulator_set_mode_voltage(ldo1, PCA9420_MODECFG1_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* LDO2 setting in Deep Sleep mode. If no device tree property is specified for
	 * Deep Sleep mode then LDO2 is enabled at 3.3V.
	 */
	ret = regulator_set_mode(ldo2, PCA9420_MODECFG1_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_LDO2, regulator_state_suspend_to_idle), regulator_off)
	ret = regulator_mode_disable(ldo2, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(ldo2, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_LDO2, regulator_state_suspend_to_idle),
				regulator_microvolt,
				3300000);
	ret = regulator_set_mode_voltage(ldo2, PCA9420_MODECFG1_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* SW1 setting in Deep Powerdown mode. If no device tree property is specified for
	 * Deep Powerdown mode then SW1 is disabled.
	 */
	ret = regulator_set_mode(sw1, PCA9420_MODECFG2_PIN);
	if (ret != 0) {
		return ret;
	}
#if DT_PROP(DT_CHILD(NODE_SW1, regulator_state_soft_off), regulator_on)
	ret = regulator_mode_enable(sw1, PCA9420_MODECFG1_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_SW1, regulator_state_soft_off),
				regulator_microvolt,
				600000);
	ret = regulator_set_mode_voltage(sw1, PCA9420_MODECFG1_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#else
	ret = regulator_mode_disable(sw1, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}
#endif

	/* SW2 setting in Deep Powerdown mode. If no device tree property is specified for
	 * Deep Powerdown mode then SW2 is enabled at 1.8V.
	 */
	ret = regulator_set_mode(sw2, PCA9420_MODECFG2_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_SW2, regulator_state_soft_off), regulator_off)
	ret = regulator_mode_disable(sw2, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(sw2, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_SW2, regulator_state_soft_off),
				regulator_microvolt,
				1800000);
	ret = regulator_set_mode_voltage(sw2, PCA9420_MODECFG2_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* LDO1 setting in Deep Powerdown mode. If no device tree property is specified for
	 * Deep Powerdown mode then LDO1 is enabled at 1.8V.
	 */
	ret = regulator_set_mode(ldo1, PCA9420_MODECFG2_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_LDO1, regulator_state_soft_off), regulator_off)
	ret = regulator_mode_disable(ldo1, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(ldo1, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_LDO1, regulator_state_soft_off),
				regulator_microvolt,
				1800000);
	ret = regulator_set_mode_voltage(ldo1, PCA9420_MODECFG2_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	/* LDO2 setting in Deep Powerdown mode. If no device tree property is specified for
	 * Deep Powerdown mode then LDO2 is enabled at 3.3V.
	 */
	ret = regulator_set_mode(ldo2, PCA9420_MODECFG2_PIN);
	if (ret != 0) {
		return ret;
	}

#if DT_PROP(DT_CHILD(NODE_LDO2, regulator_state_soft_off), regulator_off)
	ret = regulator_mode_disable(ldo2, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}
#else
	ret = regulator_mode_enable(ldo2, PCA9420_MODECFG2_PIN);
	if (ret < 0) {
		return ret;
	}

	volt = DT_PROP_OR(DT_CHILD(NODE_LDO2, regulator_state_soft_off),
				regulator_microvolt,
				3300000);
	ret = regulator_set_mode_voltage(ldo2, PCA9420_MODECFG2_PIN, volt, volt);
	if (ret != 0) {
		return ret;
	}
#endif

	return ret;
}

static int board_config_pmic(const struct device *dev)
{
	int ret = 0;

	ret = setup_pmic_modes();

	/* We can enter deep low power modes */
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	return ret;
}

#endif

static int mimxrt595_evk_init(const struct device *dev)
{
	/* Set the correct voltage range according to the board. */
	power_pad_vrange_t vrange = {
		.Vdde0Range = kPadVol_171_198,
		.Vdde1Range = kPadVol_171_198,
		.Vdde2Range = kPadVol_171_198,
		.Vdde3Range = kPadVol_300_360,
		.Vdde4Range = kPadVol_171_198
	};

	POWER_SetPadVolRange(&vrange);

#if CONFIG_REGULATOR && CONFIG_PM
	/* Do not enter deep low power modes until the PMIC modes have been initialized */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#endif

	return 0;
}

#if CONFIG_REGULATOR && CONFIG_PM
/* PMIC setup is dependent on the regulator API */
SYS_INIT(board_config_pmic, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

SYS_INIT(mimxrt595_evk_init, PRE_KERNEL_1, CONFIG_BOARD_INIT_PRIORITY);
