/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CAN_NXP_S32_CANXL_H_
#define ZEPHYR_DRIVERS_CAN_NXP_S32_CANXL_H_

#define CAN_NXP_S32_MAX_RX	CONFIG_CAN_NXP_S32_MAX_RX
#define CAN_NXP_S32_MAX_TX	CONFIG_CAN_NXP_S32_MAX_TX

#if defined(CONFIG_CAN_FD_MODE)
#define CAN_NXP_S32_MAX_DLC CANFD_MAX_DLC
#else
#define CAN_NXP_S32_MAX_DLC CAN_MAX_DLC
#endif

/*
 * Convert from RX message buffer index to allocated filter ID and
 * vice versa.
 */
#define RX_MBIDX_TO_ALLOC_IDX(x)	(x - CAN_NXP_S32_MAX_TX)
#define ALLOC_IDX_TO_RXMB_IDX(x)	(x + CAN_NXP_S32_MAX_TX)

/*
 * Convert from TX message buffer index to allocated TX ID and vice
 * versa.
 */
#define TX_MBIDX_TO_ALLOC_IDX(x) (x)
#define ALLOC_IDX_TO_TXMB_IDX(x) (x)

struct can_nxp_s32_config {
	CANXL_SIC_Type *base_sic;
	CANXL_GRP_CONTROL_Type *base_grp_ctrl;
	CANXL_DSC_CONTROL_Type *base_dsc_ctrl;
	uint8 instance;
	uint32_t clock_can;
	uint32_t bitrate;
	uint32_t sample_point;
	uint32_t sjw;
	uint32_t prop_seg;
	uint32_t phase_seg1;
	uint32_t phase_seg2;
#ifdef CONFIG_CAN_FD_MODE
	uint32_t bitrate_data;
	uint32_t sample_point_data;
	uint32_t sjw_data;
	uint32_t prop_seg_data;
	uint32_t phase_seg1_data;
	uint32_t phase_seg2_data;
#endif
	uint32_t max_bitrate;
	const struct device *phy;
	const struct pinctrl_dev_config *pin_cfg;
	Canexcel_Ip_ConfigType *can_cfg;
	void (*irq_config_func)(void);
};

struct can_nxp_s32_tx_callback {
	Canexcel_Ip_DataInfoType tx_info;
	can_tx_callback_t function;
	void *arg;
};

struct can_nxp_s32_rx_callback {
	struct can_filter filter;
	Canexcel_Ip_DataInfoType rx_info;
	can_rx_callback_t function;
	void *arg;
};

struct can_nxp_s32_data {
	Canexcel_Ip_StateType *can_state;

	ATOMIC_DEFINE(rx_allocs, CAN_NXP_S32_MAX_RX);
	struct k_mutex rx_mutex;
	struct can_nxp_s32_rx_callback rx_cbs[CAN_NXP_S32_MAX_RX];
	Canexcel_RxFdMsg *rx_msg;

	ATOMIC_DEFINE(tx_allocs, CAN_NXP_S32_MAX_TX);
	struct k_sem tx_allocs_sem;
	struct can_nxp_s32_tx_callback tx_cbs[CAN_NXP_S32_MAX_TX];
	Canexcel_TxFdMsgType *tx_msg;

	struct can_timing timing;
#ifdef CONFIG_CAN_FD_MODE
	struct can_timing timing_data;
#endif
	enum can_state state;
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	bool started;
};

#endif /* ZEPHYR_DRIVERS_CAN_NXP_S32_CANXL_H_ */
