/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <CanEXCEL_Ip_HwAccess.h>
#include <CanEXCEL_Ip_Irq.h>
#include "can_nxp_s32_canxl.h"

#define DT_DRV_COMPAT nxp_s32_canxl

#define CAN_NXP_S32_TIMEOUT_MS  1
#define CAN_NXP_S32_MAX_BITRATE	16000000

LOG_MODULE_REGISTER(nxp_s32_canxl, CONFIG_CAN_LOG_LEVEL);

#define SP_AND_TIMING_NOT_SET(inst)				\
	(!DT_INST_NODE_HAS_PROP(inst, sample_point) &&		\
	!(DT_INST_NODE_HAS_PROP(inst, prop_seg) &&		\
	DT_INST_NODE_HAS_PROP(inst, phase_seg1) &&		\
	DT_INST_NODE_HAS_PROP(inst, phase_seg2))) ||

#if DT_INST_FOREACH_STATUS_OKAY(SP_AND_TIMING_NOT_SET) 0
#error You must either set a sampling-point or timings (phase-seg* and prop-seg)
#endif

#ifdef CONFIG_CAN_FD_MODE

#define SP_AND_TIMING_DATA_NOT_SET(inst)			\
	(!DT_INST_NODE_HAS_PROP(inst, sample_point_data) &&	\
	!(DT_INST_NODE_HAS_PROP(inst, prop_seg_data) &&		\
	DT_INST_NODE_HAS_PROP(inst, phase_seg1_data) &&		\
	DT_INST_NODE_HAS_PROP(inst, phase_seg2_data))) ||

#if DT_INST_FOREACH_STATUS_OKAY(SP_AND_TIMING_DATA_NOT_SET) 0
#error You must either set a sampling-point-data or timings (phase-seg-data* and prop-seg-data)
#endif
#endif


static int can_nxp_s32_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_FD;

	return 0;
}

static int can_nxp_s32_start(const struct device *dev)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	int err;

	if (data->started) {
		return -EALREADY;
	}

	Canexcel_Ip_EnterFreezeMode(config->instance);

	if (config->phy != NULL) {
		err = can_transceiver_enable(config->phy);
		if (err != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", err);
			return err;
		}
	}

	Canexcel_Ip_ExitFreezeMode(config->instance);

	data->started = true;

	return 0;
}

static int can_nxp_s32_abort_msg(const struct can_nxp_s32_config *config, int mb_idx)
{
	uint32_t time_start = 0;
	int ret = 0;

	Canexcel_Ip_EnterFreezeMode(config->instance);

	CanXL_ClearMsgBuffIntCmd(config->base_grp_ctrl, mb_idx);
	CanXL_ClearMsgDescIntStatusFlag(config->base_grp_ctrl, mb_idx);

	time_start = k_uptime_get();
	/* Set system lock Status */
	(void)config->base_dsc_ctrl->DSCMBCTRLAR[mb_idx].SYSLOCK.DCSYSLOCK;
	while (CanXL_GetDescControlStatus(config->base_dsc_ctrl, mb_idx)
			== CANEXCEL_DESCNTSTATUS_LOCKED_HW) {
		if (k_uptime_get() - time_start >= CAN_NXP_S32_TIMEOUT_MS) {
			ret = CANEXCEL_STATUS_TIMEOUT;
			break;
		}
	}

	/* Inactive descriptor */
	config->base_dsc_ctrl->DSCMBCTRLAR[mb_idx].ACT.DCACT = 0;

	Canexcel_Ip_ExitFreezeMode(config->instance);

	return ret;
}

static int can_nxp_s32_stop(const struct device *dev)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	can_tx_callback_t function;
	void *arg;
	int alloc;
	int err;

	if (!data->started) {
		return -EALREADY;
	}

	data->started = false;

	/* Abort any pending TX frames before entering freeze mode */
	for (alloc = 0; alloc < CAN_NXP_S32_MAX_TX; alloc++) {
		function = data->tx_cbs[alloc].function;
		arg = data->tx_cbs[alloc].arg;

		if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
			if (can_nxp_s32_abort_msg(config,
					ALLOC_IDX_TO_TXMB_IDX(alloc))) {
				LOG_ERR("Can't abort message !");
			};

			function(dev, -ENETDOWN, arg);
			k_sem_give(&data->tx_allocs_sem);
		}
	}

	Canexcel_Ip_EnterFreezeMode(config->instance);

	if (config->phy != NULL) {
		err = can_transceiver_disable(config->phy);
		if (err != 0) {
			LOG_ERR("failed to disable CAN transceiver (err %d)", err);
			return err;
		}
	}

	Canexcel_Ip_ExitFreezeMode(config->instance);

	return 0;
}


static int can_nxp_s32_set_mode(const struct device *dev, can_mode_t mode)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	Canexcel_Ip_ModesType can_nxp_s32_mode = CAN_MODE_NORMAL;
	bool is_canfd;

	if (data->started) {
		return -EBUSY;
	}
#ifdef CONFIG_CAN_FD_MODE
	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_FD)) != 0) {
#else
	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY)) != 0) {
#endif
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	is_canfd = !!(mode & CAN_MODE_FD);

	if (mode & CAN_MODE_LISTENONLY) {
		can_nxp_s32_mode = CANEXCEL_LISTEN_ONLY_MODE;
	} else if (mode & CAN_MODE_LOOPBACK) {
		can_nxp_s32_mode = CANEXCEL_LOOPBACK_MODE;
	}

	Canexcel_Ip_EnterFreezeMode(config->instance);
#ifdef CONFIG_CAN_FD_MODE
	CanXL_SetFDEnabled(config->base_sic, is_canfd, CONFIG_CAN_NXP_S32_BRS);
#endif
	CanXL_SetOperationMode(config->base_sic, can_nxp_s32_mode);

	Canexcel_Ip_ExitFreezeMode(config->instance);

	return 0;
}

static int can_nxp_s32_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_nxp_s32_config *config = dev->config;

	__ASSERT(rate != NULL, "Rateptr to get core clock is NULL");

	*rate = config->clock_can;

	return 0;
}

static int can_nxp_s32_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CAN_NXP_S32_MAX_RX;
}

static int can_nxp_s32_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
	const struct can_nxp_s32_config *config = dev->config;

	*max_bitrate = config->max_bitrate;

	return 0;
}

static int can_nxp_s32_get_state(const struct device *dev, enum can_state *state,
						struct can_bus_err_cnt *err_cnt)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	uint32_t sys_status = config->base_sic->SYSS;

	if (state) {
		if (!data->started) {
			*state = CAN_STATE_STOPPED;
		} else {
			if (sys_status & CANXL_INT_BUSOFF) {
				*state = CAN_STATE_BUS_OFF;
			} else if (sys_status & CANXL_INT_PASIVE_ERR) {
				*state = CAN_STATE_ERROR_PASSIVE;
			} else if (sys_status & (CANXL_INT_RX_WARNING | CANXL_INT_TX_WARNING)) {
				*state = CAN_STATE_ERROR_WARNING;
			} else {
				*state = CAN_STATE_ERROR_ACTIVE;
			}
		}
	}

	return 0;
}

static void can_nxp_s32_set_state_change_callback(const struct device *dev,
							can_state_change_callback_t callback,
							void *user_data)
{
	struct can_nxp_s32_data *data = dev->data;

	data->state_change_cb = callback;
	data->state_change_cb_data = user_data;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static int can_nxp_s32_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	enum can_state state;
	uint64_t start_time;
	int ret = 0;

	if (!data->started) {
		return -ENETDOWN;
	}

	can_nxp_s32_get_state(dev, &state, NULL);
	if (state != CAN_STATE_BUS_OFF) {
		return 0;
	}

	start_time = k_uptime_ticks();
	config->base_sic->BCFG1 &= (~CANXL_SIC_BCFG1_ABRDIS_MASK);

	if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		can_nxp_s32_get_state(dev, &state, NULL);

		while (state == CAN_STATE_BUS_OFF) {
			if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
				k_uptime_ticks() - start_time >= timeout.ticks) {
				ret = -EAGAIN;
			}

			can_nxp_s32_get_state(dev, &state, NULL);
		}
	}

	config->base_sic->BCFG1 |= CANXL_SIC_BCFG1_ABRDIS_MASK;

	return ret;
}
#endif

static void can_nxp_s32_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	int mb_indx = ALLOC_IDX_TO_RXMB_IDX(filter_id);

	__ASSERT_NO_MSG(filter_id >= 0 && filter_id < CAN_NXP_S32_MAX_RX);

	k_mutex_lock(&data->rx_mutex, K_FOREVER);

	if (atomic_test_and_clear_bit(data->rx_allocs, filter_id)) {
		if (can_nxp_s32_abort_msg(config, mb_indx)) {
			LOG_ERR("Can't abort message !");
		};

		data->rx_cbs[filter_id].function = NULL;
		data->rx_cbs[filter_id].arg = NULL;
		data->rx_cbs[filter_id].filter = (struct can_filter){0};
	} else {
		LOG_WRN("Filter ID %d already detached", filter_id);
	}

	k_mutex_unlock(&data->rx_mutex);
}

static int can_nxp_s32_add_rx_filter(const struct device *dev,
				can_rx_callback_t callback,
				void *user_data,
				const struct can_filter *filter)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	int alloc = -ENOSPC;
	int mb_indx;
	uint32_t mask;

	__ASSERT_NO_MSG(callback);
#if defined(CONFIG_CAN_FD_MODE) && !defined(CONFIG_NET_SOCKETS_CAN)
	if ((filter->flags & ~(CAN_FILTER_IDE | CAN_FILTER_DATA | CAN_FILTER_FDF)) != 0) {
#else
	if ((filter->flags & ~(CAN_FILTER_IDE | CAN_FILTER_DATA)) != 0) {
#endif
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->rx_mutex, K_FOREVER);

	/* Find and allocate RX message buffer */
	for (int i = 0; i < CAN_NXP_S32_MAX_RX; i++) {
		if (!atomic_test_and_set_bit(data->rx_allocs, i)) {
			alloc = i;
			break;
		}
	}

	if (alloc == -ENOSPC) {
		LOG_ERR("No free filter bank found");
		return alloc;
	}

	data->rx_cbs[alloc].function = callback;
	data->rx_cbs[alloc].arg = user_data;
	data->rx_cbs[alloc].filter = *filter;

	data->rx_cbs[alloc].rx_info = (Canexcel_Ip_DataInfoType) {
#if defined(CONFIG_CAN_FD_MODE) && !defined(CONFIG_NET_SOCKETS_CAN)
		.frame = !!(filter->flags & CAN_FILTER_FDF) ?
				CANEXCEL_FD_FRAME : CANEXCEL_CLASIC_FRAME,
#else
		.frame = CANEXCEL_CLASIC_FRAME,
#endif
		.idType = !!(filter->flags & CAN_FILTER_IDE) ?
				CANEXCEL_MSG_ID_EXT : CANEXCEL_MSG_ID_STD,
		.dataLength = 64,
	};

	/* Set Rx Mb individual mask for */
	mb_indx = ALLOC_IDX_TO_RXMB_IDX(alloc);
	if (!!(filter->flags & CAN_FILTER_IDE)) {
		mask =  (filter->mask & CANXL_IP_ID_EXT_MASK);
	} else {
		mask = ((filter->mask << CANXL_IP_ID_STD_SHIFT) & CANXL_IP_ID_STD_MASK);
	}

	Canexcel_Ip_EnterFreezeMode(config->instance);

	Canexcel_Ip_SetRxIndividualMask(config->instance, mb_indx,
						data->rx_cbs[alloc].rx_info.frame, mask);

	Canexcel_Ip_ConfigRx(config->instance, mb_indx, filter->id,
					&data->rx_cbs[alloc].rx_info);

	Canexcel_Ip_ReceiveFD(config->instance, mb_indx, &data->rx_msg[alloc], FALSE);

	Canexcel_Ip_ExitFreezeMode(config->instance);

	k_mutex_unlock(&data->rx_mutex);

	return alloc;
}

static int can_nxp_s32_send(const struct device *dev,
				const struct can_frame *frame,
				k_timeout_t timeout,
				can_tx_callback_t callback, void *user_data)
{
	const struct can_nxp_s32_config *config = dev->config;
	uint8_t data_length = can_dlc_to_bytes(frame->dlc);
	struct can_nxp_s32_data *data = dev->data;
	Canexcel_Ip_StatusType status;
	enum can_state state;
	int alloc, mb_indx;

	__ASSERT_NO_MSG(callback != NULL);

	if (frame->dlc > CAN_NXP_S32_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_NXP_S32_MAX_DLC);
		return -EINVAL;
	}

#if defined(CONFIG_CAN_FD_MODE) && !defined(CONFIG_NET_SOCKETS_CAN)
	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_FDF | CAN_FRAME_BRS)) != 0) {
#else
	if ((frame->flags & ~CAN_FRAME_IDE) != 0) {
#endif
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (!data->started) {
		return -ENETDOWN;
	}

	can_nxp_s32_get_state(dev, &state, NULL);
	if (state == CAN_STATE_BUS_OFF) {
		LOG_ERR("Transmit failed, bus-off");
		return -ENETUNREACH;
	}

	if (k_sem_take(&data->tx_allocs_sem, timeout) != 0) {
		return -EAGAIN;
	}

	for (alloc = 0; alloc < CAN_NXP_S32_MAX_TX; alloc++) {
		if (!atomic_test_and_set_bit(data->tx_allocs, alloc)) {
			break;
		}
	}

	data->tx_cbs[alloc].function = callback;
	data->tx_cbs[alloc].arg = user_data;
	mb_indx = ALLOC_IDX_TO_TXMB_IDX(alloc);
	data->tx_cbs[alloc].tx_info = (Canexcel_Ip_DataInfoType) {
#if defined(CONFIG_CAN_FD_MODE) && !defined(CONFIG_NET_SOCKETS_CAN)
		.frame = !!(frame->flags & CAN_FRAME_FDF) ?
				CANEXCEL_FD_FRAME : CANEXCEL_CLASIC_FRAME,
#else
		.frame = CANEXCEL_CLASIC_FRAME,
#endif
		.idType = !!(frame->flags & CAN_FRAME_IDE) ?
				CANEXCEL_MSG_ID_EXT : CANEXCEL_MSG_ID_STD,
		.priority = 0,
		.retransmission = 0,
		.enable_brs = !!(frame->flags & CAN_FRAME_BRS) ? TRUE : FALSE,
		.fd_padding = 0,
		.dataLength = data_length,
		.is_polling = FALSE
	};

	LOG_DBG("%s: Sending %d bytes Tx Mb %d, "
		"Tx Id: 0x%x, "
		"Id type: %s %s %s %s"
		, dev->name, data_length
		, mb_indx, frame->id
		, !!(frame->flags & CAN_FRAME_IDE) ?
						"standard" : "extended"
		, !!(frame->flags & CAN_FRAME_RTR) ? "RTR" : ""
		, !!(frame->flags & CAN_FRAME_FDF) ? "FD frame" : ""
		, !!(frame->flags & CAN_FRAME_BRS) ? "BRS" : "");

	/* Send MB Interrupt */
	status = Canexcel_Ip_SendFDMsg(config->instance, mb_indx, &data->tx_cbs[alloc].tx_info,
				frame->id, (uint8_t *)&frame->data, &data->tx_msg[alloc]);

	if (status != CANEXCEL_STATUS_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static void nxp_s32_zcan_timming_to_canxl_timming(const struct can_timing *timing,
							Canexcel_Ip_TimeSegmentType *canxl_timing)
{
	LOG_DBG("propSeg: %d, phase_seg1: %d, phase_seg2: %d, prescaler: %d, sjw: %d",
		timing->prop_seg, timing->phase_seg1, timing->phase_seg2,
		timing->prescaler, timing->sjw);

	canxl_timing->propSeg = timing->prop_seg - 1U;
	canxl_timing->phaseSeg1 = timing->phase_seg1 - 1U;
	canxl_timing->phaseSeg2 = timing->phase_seg2 - 1U;
	canxl_timing->preDivider = timing->prescaler - 1U;
	canxl_timing->rJumpwidth = timing->sjw - 1U;
}

static int can_nxp_s32_set_timing(const struct device *dev,
			const struct can_timing *timing)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	Canexcel_Ip_TimeSegmentType can_time_segment = {0};

	if (data->started) {
		return -EBUSY;
	}

	LOG_DBG("Setting CAN bitrate %d:", config->bitrate);
	nxp_s32_zcan_timming_to_canxl_timming(timing, &can_time_segment);

	/* Set timing for CAN instance*/
	CanXL_SetBaudRate(config->base_sic, &can_time_segment);

	return 0;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_nxp_s32_set_timing_data(const struct device *dev,
			const struct can_timing *timing_data)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	Canexcel_Ip_TimeSegmentType can_fd_time_segment = {0};

	if (data->started) {
		return -EBUSY;
	}

	LOG_DBG("Setting CAN-FD bitrate %d:", config->bitrate_data);
	nxp_s32_zcan_timming_to_canxl_timming(timing_data, &can_fd_time_segment);

	/* Set timing for CAN FD instance*/
	CanXL_SetFDBaudRate(config->base_sic, &can_fd_time_segment);

	return 0;
}
#endif

void can_int_error_isr(const struct device *dev)
{
	const struct can_nxp_s32_config *config = dev->config;

	Canexcel_Ip_ErrIRQHandler(config->instance);
}

void can_int_rx_tx_data_isr(const struct device *dev)
{
	const struct can_nxp_s32_config *config = dev->config;

	Canexcel_Ip_RxTxIRQHandler(config->instance);
}

static void can_nxp_s32_err_callback(const struct device *dev,
					Canexcel_Ip_EventType eventType,
					uint32 u32SysStatus,
					const Canexcel_Ip_StateType *canexcelState)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	void *cb_data = data->state_change_cb_data;
	can_tx_callback_t function;
	int alloc;
	void *arg;

	switch (eventType) {
	case CANEXCEL_EVENT_TX_WARNING:
		LOG_WRN("Tx Warning (error 0x%x)", u32SysStatus);
		break;
	case CANEXCEL_EVENT_RX_WARNING:
		LOG_WRN("Rx Warning (error 0x%x)", u32SysStatus);
		break;
	case CANEXCEL_EVENT_BUSOFF:
		LOG_ERR("Bus Off (error 0x%x)", u32SysStatus);
		break;
	case CANEXCEL_EVENT_ERROR:
		LOG_ERR("Error Format Frames (error 0x%x)", u32SysStatus);
		break;
	case CANEXCEL_EVENT_ERROR_FD:
		LOG_ERR("Error Data Phase (error 0x%x)", u32SysStatus);
		break;
	case CANEXCEL_EVENT_PASSIVE:
		LOG_ERR("Error Passive (error 0x%x)", u32SysStatus);
		break;
	default:
		break;
	}

	can_nxp_s32_get_state(dev, &state, &err_cnt);
	if (data->state != state) {
		data->state = state;
		if (data->state_change_cb) {
			data->state_change_cb(dev, state, err_cnt, cb_data);
		}
	}

	if (state == CAN_STATE_BUS_OFF) {
		/* Abort any pending TX frames in case of bus-off */
		for (alloc = 0; alloc < CAN_NXP_S32_MAX_TX; alloc++) {
			/* Copy callback function and argument before clearing bit */
			function = data->tx_cbs[alloc].function;
			arg = data->tx_cbs[alloc].arg;

			if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
				if (can_nxp_s32_abort_msg(config,
						ALLOC_IDX_TO_TXMB_IDX(alloc))) {
					LOG_ERR("Can't abort message !");
				};

				function(dev, -ENETUNREACH, arg);
				k_sem_give(&data->tx_allocs_sem);
			}
		}
	}
}

static void nxp_s32_msg_data_to_zcan_frame(Canexcel_RxFdMsg msg_data,
							struct can_frame *frame)
{
	if (!!(msg_data.Header.Id & CANXL_TX_HEADER_IDE_MASK)) {
		frame->flags |= CAN_FRAME_IDE;
	}

	if (!!(msg_data.Header.Id & CANXL_TX_HEADER_RTR_MASK)) {
		frame->flags |= CAN_FRAME_RTR;
	}

	if (!!(frame->flags & CAN_FRAME_IDE)) {
		frame->id = (msg_data.Header.Id & CANXL_IP_ID_EXT_MASK);
	} else {
		frame->id = ((msg_data.Header.Id & CANXL_IP_ID_STD_MASK)
						>> CANXL_IP_ID_STD_SHIFT);
	}

	frame->dlc = (msg_data.Header.Control & CANXL_TX_HEADER_DLC_MASK)
						>> CANXL_TX_HEADER_DLC_SHIFT;

	if (!!(msg_data.Header.Control & CANXL_TX_HEADER_FDF_MASK)) {
		frame->flags |= CAN_FRAME_FDF;
	}

	if (!!(msg_data.Header.Control & CANXL_TX_HEADER_BRS_MASK)) {
		frame->flags |= CAN_FRAME_BRS;
	}

	memcpy(frame->data, msg_data.data, can_dlc_to_bytes(frame->dlc));

#ifdef CONFIG_CAN_RX_TIMESTAMP
	frame->timestamp = msg_data.timeStampH << 32 | msg_data.timeStampL;
#endif /* CAN_RX_TIMESTAMP */
}

static void can_nxp_s32_ctrl_callback(const struct device *dev,
					Canexcel_Ip_EventType eventType, uint32 buffidx,
					const Canexcel_Ip_StateType *canexcelState)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	struct can_frame frame = {0};
	can_tx_callback_t tx_func;
	can_rx_callback_t rx_func;
	Canexcel_Ip_StatusType status;
	Canexcel_Ip_DataInfoType tx_info;
	int alloc;

	if (eventType == CANEXCEL_EVENT_TX_COMPLETE) {
		alloc = TX_MBIDX_TO_ALLOC_IDX(buffidx);
		tx_func = data->tx_cbs[alloc].function;
		tx_info = data->tx_cbs[alloc].tx_info;
		LOG_DBG("%s: Sent Tx Mb %d", dev->name, buffidx);
		if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) {
			tx_func(dev, 0, data->tx_cbs[alloc].arg);
			k_sem_give(&data->tx_allocs_sem);
		}
	} else if (eventType == CANEXCEL_EVENT_RX_COMPLETE) {
		alloc = RX_MBIDX_TO_ALLOC_IDX(buffidx);
		rx_func = data->rx_cbs[alloc].function;
		if (atomic_test_bit(data->rx_allocs, alloc)) {
			nxp_s32_msg_data_to_zcan_frame(data->rx_msg[alloc], &frame);

			LOG_DBG("%s: Received %d bytes Rx Mb %d, "
				"Rx Id: 0x%x, "
				"Id type: %s %s %s %s"
				, dev->name, can_dlc_to_bytes(frame.dlc)
				, buffidx, frame.id
				, !!(frame.flags & CAN_FRAME_IDE) ?
								"standard" : "extended"
				, !!(frame.flags & CAN_FRAME_RTR) ? "RTR" : ""
				, !!(frame.flags & CAN_FRAME_FDF) ? "FD frame" : ""
				, !!(frame.flags & CAN_FRAME_BRS) ? "BRS" : "");

			rx_func(dev, &frame, data->rx_cbs[alloc].arg);

			status = Canexcel_Ip_ReceiveFD(config->instance, buffidx,
							&data->rx_msg[alloc], FALSE);
			if (status != CANEXCEL_STATUS_SUCCESS) {
				LOG_ERR("Can't enable interrupt MB %d for next message!", buffidx);
			}
		}
	}
}

static int can_nxp_s32_init(const struct device *dev)
{
	const struct can_nxp_s32_config *config = dev->config;
	struct can_nxp_s32_data *data = dev->data;
	int err;

	if (config->phy != NULL) {
		if (!device_is_ready(config->phy)) {
			LOG_ERR("CAN transceiver not ready");
			return -ENODEV;
		}
	}

	k_mutex_init(&data->rx_mutex);
	k_sem_init(&data->tx_allocs_sem, CAN_NXP_S32_MAX_TX, CAN_NXP_S32_MAX_TX);

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Enable CANXL HW */
	IP_MC_RGM->PRST_0[0].PRST_0 &=
		~(MC_RGM_PRST_0_PERIPH_16_RST_MASK | MC_RGM_PRST_0_PERIPH_24_RST_MASK);

	data->timing.sjw = config->sjw;
	if (config->sample_point) {
		err = can_calc_timing(dev, &data->timing, config->bitrate,
						config->sample_point);
		if (err == -EINVAL) {
			LOG_ERR("Can't find timing for given param");
			return -EIO;
		}
		if (err > 0) {
			LOG_WRN("Sample-point error : %d", err);
		}
	} else {
		data->timing.prop_seg = config->prop_seg;
		data->timing.phase_seg1 = config->phase_seg1;
		data->timing.phase_seg2 = config->phase_seg2;
		err = can_calc_prescaler(dev, &data->timing, config->bitrate);
		if (err) {
			LOG_WRN("Bitrate error: %d", err);
		}
	}

	LOG_DBG("Setting CAN bitrate %d:", config->bitrate);
	nxp_s32_zcan_timming_to_canxl_timming(&data->timing, &config->can_cfg->bitrate);

#ifdef CONFIG_CAN_FD_MODE
	data->timing_data.sjw = config->sjw_data;
	if (config->sample_point_data) {
		err = can_calc_timing_data(dev, &data->timing_data, config->bitrate_data,
						config->sample_point_data);
		if (err == -EINVAL) {
			LOG_ERR("Can't find timing data for given param");
			return -EIO;
		}
		if (err > 0) {
			LOG_WRN("Sample-point-data err : %d", err);
		}
	} else {
		data->timing_data.prop_seg = config->prop_seg_data;
		data->timing_data.phase_seg1 = config->phase_seg1_data;
		data->timing_data.phase_seg2 = config->phase_seg2_data;
		err = can_calc_prescaler(dev, &data->timing_data, config->bitrate_data);
		if (err) {
			LOG_WRN("Bitrate data error: %d", err);
		}
	}

	LOG_DBG("Setting CAN-FD bitrate %d:", config->bitrate_data);
	nxp_s32_zcan_timming_to_canxl_timming(&data->timing_data, &config->can_cfg->Fd_bitrate);
#endif

	/* Initialize CAN structure*/
	Canexcel_Ip_Init(config->instance, config->can_cfg, data->can_state);

	/* Enable Interrupt */
	Canexcel_Ip_EnableInterrupts(config->instance);

	config->irq_config_func();

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	config->base_sic->BCFG1 &= (~CANXL_SIC_BCFG1_ABRDIS_MASK);
#endif

	can_nxp_s32_get_state(dev, &data->state, NULL);

	return 0;
}

static const struct can_driver_api can_nxp_s32_driver_api = {
	.get_capabilities = can_nxp_s32_get_capabilities,
	.start = can_nxp_s32_start,
	.stop = can_nxp_s32_stop,
	.set_mode = can_nxp_s32_set_mode,
	.set_timing = can_nxp_s32_set_timing,
	.send = can_nxp_s32_send,
	.add_rx_filter = can_nxp_s32_add_rx_filter,
	.remove_rx_filter = can_nxp_s32_remove_rx_filter,
	.get_state = can_nxp_s32_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_nxp_s32_recover,
#endif
	.set_state_change_callback = can_nxp_s32_set_state_change_callback,
	.get_core_clock = can_nxp_s32_get_core_clock,
	.get_max_filters = can_nxp_s32_get_max_filters,
	.get_max_bitrate = can_nxp_s32_get_max_bitrate,
	.timing_min = {
		.sjw = 0x01,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x02,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x04,
		.prop_seg = 0x08,
		.phase_seg1 = 0x08,
		.phase_seg2 = 0x08,
		.prescaler = 0x100
	},
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_nxp_s32_set_timing_data,
	.timing_data_min = {
		.sjw = 0x01,
		.prop_seg = 0x01,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x02,
		.prescaler = 0x01
	},
	.timing_data_max = {
		.sjw = 0x04,
		.prop_seg = 0x08,
		.phase_seg1 = 0x08,
		.phase_seg2 = 0x08,
		.prescaler = 0x100
	}
#endif
};

#define CAN_NXP_S32_NODE(n)		DT_NODELABEL(can##n)

#define CAN_NXP_S32_IRQ_INIT(n, name)								\
	do {											\
		IRQ_CONNECT(DT_IRQ_BY_NAME(CAN_NXP_S32_NODE(n), name, irq),			\
				DT_IRQ_BY_NAME(CAN_NXP_S32_NODE(n), name, priority),		\
				can_##name##_isr,						\
				DEVICE_DT_GET(CAN_NXP_S32_NODE(n)),				\
				DT_IRQ_BY_NAME(CAN_NXP_S32_NODE(n), name, flags));		\
		irq_enable(DT_IRQ_BY_NAME(CAN_NXP_S32_NODE(n), name, irq));			\
	} while (0)

#define CAN_NXP_S32_IRQ(n, name)								\
	COND_CODE_1(DT_IRQ_HAS_NAME(CAN_NXP_S32_NODE(n), name),					\
				(CAN_NXP_S32_IRQ_INIT(n, name)), ())

#define CAN_NXP_S32_IRQ_CONFIG(n)								\
	static void can_irq_config_##n(void)							\
	{											\
		CAN_NXP_S32_IRQ(n, int_error);							\
		CAN_NXP_S32_IRQ(n, int_rx_tx_data);						\
	}

#define CAN_NXP_S32_ERR_CALLBACK(n)								\
	void nxp_s32_can_##n##_err_callback(uint8 instance, Canexcel_Ip_EventType eventType,	\
			uint32 u32SysStatus, const Canexcel_Ip_StateType *canexcelState)	\
	{											\
		const struct device *dev = DEVICE_DT_GET(CAN_NXP_S32_NODE(n));			\
		can_nxp_s32_err_callback(dev, eventType, u32SysStatus, canexcelState);		\
	}

#define CAN_NXP_S32_CTRL_CALLBACK(n)								\
	void nxp_s32_can_##n##_ctrl_callback(uint8 instance, Canexcel_Ip_EventType eventType,	\
				uint32 buffIdx, const Canexcel_Ip_StateType *canexcelState)	\
	{											\
		const struct device *dev = DEVICE_DT_GET(CAN_NXP_S32_NODE(n));			\
		can_nxp_s32_ctrl_callback(dev, eventType, buffIdx, canexcelState);		\
	}

#if defined(CONFIG_CAN_FD_MODE)
#define CAN_NXP_S32_TIMING_DATA_CONFIG(n)							\
		.bitrate_data = DT_PROP(CAN_NXP_S32_NODE(n), bus_speed_data),			\
		.sjw_data = DT_PROP(CAN_NXP_S32_NODE(n), sjw_data),				\
		.prop_seg_data = DT_PROP_OR(CAN_NXP_S32_NODE(n), prop_seg_data, 0),		\
		.phase_seg1_data = DT_PROP_OR(CAN_NXP_S32_NODE(n), phase_seg1_data, 0),		\
		.phase_seg2_data = DT_PROP_OR(CAN_NXP_S32_NODE(n), phase_seg2_data, 0),		\
		.sample_point_data = DT_PROP_OR(CAN_NXP_S32_NODE(n), sample_point_data, 0),
#define CAN_NXP_S32_FD_MODE	1
#define CAN_NXP_S32_BRS		CONFIG_CAN_NXP_S32_BRS
#else
#define CAN_NXP_S32_TIMING_DATA_CONFIG(n)
#define CAN_NXP_S32_FD_MODE	0
#define CAN_NXP_S32_BRS		0
#endif

#define CAN_NXP_S32_INIT_DEVICE(n)								\
	CAN_NXP_S32_CTRL_CALLBACK(n)								\
	CAN_NXP_S32_ERR_CALLBACK(n)								\
	CAN_NXP_S32_IRQ_CONFIG(n)								\
	PINCTRL_DT_DEFINE(CAN_NXP_S32_NODE(n));							\
	Canexcel_Ip_ConfigType can_nxp_s32_default_config##n = {				\
		.rx_mbdesc = (uint8)CAN_NXP_S32_MAX_RX,						\
		.tx_mbdesc = (uint8)CAN_NXP_S32_MAX_TX,						\
		.CanxlMode = CANEXCEL_LOOPBACK_MODE,						\
		.fd_enable = (boolean)CAN_NXP_S32_FD_MODE,					\
		.bitRateSwitch = (boolean)CAN_NXP_S32_BRS,					\
		.ctrlOptions = (uint32)CANXL_IP_BUSOFF_RECOVERY_U32,				\
		.Callback = nxp_s32_can_##n##_ctrl_callback,					\
		.ErrorCallback = nxp_s32_can_##n##_err_callback					\
	};											\
	__nocache Canexcel_Ip_StateType can_nxp_s32_state##n;					\
	__nocache Canexcel_TxFdMsgType tx_msg##n[CAN_NXP_S32_MAX_TX];				\
	__nocache Canexcel_RxFdMsg rx_msg_##n[CAN_NXP_S32_MAX_RX];				\
	static struct can_nxp_s32_data can_nxp_s32_data_##n = {					\
		.can_state = (Canexcel_Ip_StateType *)&can_nxp_s32_state##n,			\
		.tx_msg = tx_msg##n,								\
		.rx_msg = rx_msg_##n,								\
	};											\
	static struct can_nxp_s32_config can_nxp_s32_config_##n = {				\
		.base_sic = (CANXL_SIC_Type *)DT_REG_ADDR_BY_NAME(CAN_NXP_S32_NODE(n), sic),	\
		.base_grp_ctrl = (CANXL_GRP_CONTROL_Type *)					\
					DT_REG_ADDR_BY_NAME(CAN_NXP_S32_NODE(n), grp_ctrl),	\
		.base_dsc_ctrl = (CANXL_DSC_CONTROL_Type *)					\
					DT_REG_ADDR_BY_NAME(CAN_NXP_S32_NODE(n), dsc_ctrl),	\
		.instance = n,									\
		.clock_can = DT_PROP(CAN_NXP_S32_NODE(n), clock_frequency),			\
		.bitrate = DT_PROP(CAN_NXP_S32_NODE(n), bus_speed),				\
		.sjw = DT_PROP(CAN_NXP_S32_NODE(n), sjw),					\
		.prop_seg = DT_PROP_OR(CAN_NXP_S32_NODE(n), prop_seg, 0),			\
		.phase_seg1 = DT_PROP_OR(CAN_NXP_S32_NODE(n), phase_seg1, 0),			\
		.phase_seg2 = DT_PROP_OR(CAN_NXP_S32_NODE(n), phase_seg2, 0),			\
		.sample_point = DT_PROP_OR(CAN_NXP_S32_NODE(n), sample_point, 0),		\
		CAN_NXP_S32_TIMING_DATA_CONFIG(n)						\
		.max_bitrate = DT_CAN_TRANSCEIVER_MAX_BITRATE(CAN_NXP_S32_NODE(n),		\
								CAN_NXP_S32_MAX_BITRATE),	\
		.phy = NULL,									\
		.pin_cfg = PINCTRL_DT_DEV_CONFIG_GET(CAN_NXP_S32_NODE(n)),			\
		.can_cfg = (Canexcel_Ip_ConfigType *)&can_nxp_s32_default_config##n,		\
		.irq_config_func = can_irq_config_##n						\
	};											\
	static int can_nxp_s32_##n##_init(const struct device *dev)				\
	{											\
		return can_nxp_s32_init(dev);							\
	}											\
	DEVICE_DT_DEFINE(CAN_NXP_S32_NODE(n),							\
			&can_nxp_s32_##n##_init,						\
			NULL,									\
			&can_nxp_s32_data_##n,							\
			&can_nxp_s32_config_##n,						\
			POST_KERNEL,								\
			CONFIG_CAN_INIT_PRIORITY,						\
			&can_nxp_s32_driver_api);

#if DT_NODE_HAS_STATUS(CAN_NXP_S32_NODE(0), okay)
CAN_NXP_S32_INIT_DEVICE(0)
#endif

#if DT_NODE_HAS_STATUS(CAN_NXP_S32_NODE(1), okay)
CAN_NXP_S32_INIT_DEVICE(1)
#endif
