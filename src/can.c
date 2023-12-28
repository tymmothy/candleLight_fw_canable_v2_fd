/*

   The MIT License (MIT)

   Copyright (c) 2016 Hubert Denkmair

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.

 */
#include <string.h>
#include "can.h"
#include "config.h"
#include "device.h"
#include "gpio.h"
#include "gs_usb.h"
#include "hal_include.h"


void can_init(can_data_t *hcan, FDCAN_GlobalTypeDef *instance)
{
	device_can_init(hcan, instance);
}

bool can_set_bittiming(can_data_t *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if (  (brp>0) && (brp<=1024)
	   && (phase_seg1>0) && (phase_seg1<=16)
	   && (phase_seg2>0) && (phase_seg2<=8)
	   && (sjw>0) && (sjw<=4)
		  ) {
		hcan->brp = brp & 0x3FF;
		hcan->phase_seg1 = phase_seg1;
		hcan->phase_seg2 = phase_seg2;
		hcan->sjw = sjw;

		return true;
	} else {
		return false;
	}
}

bool can_set_data_bittiming(can_data_t *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if (  (brp>0) && (brp<=1024)
	   && (phase_seg1>0) && (phase_seg1<=16)
	   && (phase_seg2>0) && (phase_seg2<=8)
	   && (sjw>0) && (sjw<=4)
		  ) {
		hcan->dbrp = brp & 0x3FF;
		hcan->dphase_seg1 = phase_seg1;
		hcan->dphase_seg2 = phase_seg2;
		hcan->dsjw = sjw;

		return true;
	} else {
		return false;
	}
}

void can_enable(can_data_t *hcan, bool loop_back, bool listen_only, bool one_shot, bool fd)
{
	uint32_t mode = FDCAN_MODE_NORMAL;

	HAL_FDCAN_Stop(&hcan->handle);

	hcan->fd = fd;

	if (listen_only && loop_back) {
		mode |= FDCAN_MODE_INTERNAL_LOOPBACK;

	} else if (listen_only) {
			mode |= FDCAN_MODE_BUS_MONITORING;

	} else if (loop_back) {
		mode |= FDCAN_MODE_EXTERNAL_LOOPBACK;
	}

	hcan->handle.Init.Mode = mode;

	/* Sorting out heuristics for FDCAN_FRAME_FD_BRS / FDCAN_FRAME_FD_NO_BRS is too hard;
	 * just always enable bitrate switching for FDCAN
	 */
	hcan->handle.Init.FrameFormat = fd ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
	hcan->handle.Init.AutoRetransmission = one_shot ? DISABLE : ENABLE;

	hcan->handle.Init.NominalPrescaler = hcan->brp;
	hcan->handle.Init.NominalTimeSeg1 = hcan->phase_seg1;
	hcan->handle.Init.NominalTimeSeg2 = hcan->phase_seg2;
	hcan->handle.Init.NominalSyncJumpWidth = hcan->sjw;

   hcan->handle.Init.DataPrescaler = hcan->dbrp;
   hcan->handle.Init.DataTimeSeg1 = hcan->dphase_seg1;
   hcan->handle.Init.DataTimeSeg2 = hcan->dphase_seg2;
   hcan->handle.Init.DataSyncJumpWidth = hcan->dsjw;

	HAL_FDCAN_Init(&hcan->handle);

	/* Could try to calculate delay comp... */
	HAL_FDCAN_ConfigTxDelayCompensation(&hcan->handle, 5, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hcan->handle);

   HAL_FDCAN_Start(&hcan->handle);

   HAL_FDCAN_ActivateNotification(&hcan->handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                  | FDCAN_IT_LIST_BIT_LINE_ERROR
                                  | FDCAN_IT_LIST_PROTOCOL_ERROR, 0);

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, !GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif
}

void can_disable(can_data_t *hcan)
{
#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif

   HAL_FDCAN_Stop(&hcan->handle);
   HAL_FDCAN_DeactivateNotification(&hcan->handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                    | FDCAN_IT_LIST_BIT_LINE_ERROR
                                    | FDCAN_IT_LIST_PROTOCOL_ERROR);
}

bool can_is_enabled(can_data_t *hcan)
{
	HAL_FDCAN_StateTypeDef state;

	state = HAL_FDCAN_GetState(&hcan->handle);

	return (state != HAL_FDCAN_STATE_RESET);
}

bool can_is_rx_pending(can_data_t *hcan)
{
	if (!can_is_enabled(hcan)) {
		return false;
	}
	return HAL_FDCAN_GetRxFifoFillLevel(&hcan->handle, FDCAN_RX_FIFO0) > 0;
}

bool can_receive(can_data_t *hcan, struct gs_host_frame *rx_frame)
{
	if (can_is_enabled(hcan) && can_is_rx_pending(hcan)) {
	   FDCAN_RxHeaderTypeDef header;
	   uint8_t data[sizeof(struct canfd)] = {0};
	   HAL_StatusTypeDef status;

	   status = HAL_FDCAN_GetRxMessage(&hcan->handle, FDCAN_RX_FIFO0, &header, data);
	   (void)status;

		if (header.IdType == FDCAN_EXTENDED_ID) {
			rx_frame->can_id = CAN_EFF_FLAG | (header.Identifier & 0x1FFFFFFF);
		} else {
			rx_frame->can_id = (header.Identifier & 0x7FF);
		}

		if (header.RxFrameType == FDCAN_REMOTE_FRAME) {
			rx_frame->can_id |= CAN_RTR_FLAG;
		}

		if (header.FDFormat == FDCAN_FD_CAN) {
			rx_frame->flags |= GS_CAN_FLAG_FD;

			if (header.BitRateSwitch == FDCAN_BRS_ON) {
		   	rx_frame->flags |= GS_CAN_FLAG_BRS;
			}

		   if (header.ErrorStateIndicator == FDCAN_ESI_ACTIVE) {
		   	rx_frame->flags |= GS_CAN_FLAG_ESI;
		   }
		}

      rx_frame->can_dlc = (header.DataLength >> 16) & 0xf;
		memcpy(rx_frame->raw_data, data, sizeof(data));

		return true;
	} else {
		return false;
	}
}

bool can_send(can_data_t *hcan, struct gs_host_frame *frame)
{
	FDCAN_TxHeaderTypeDef frame_header = {
		.TxFrameType = FDCAN_DATA_FRAME,
		.FDFormat = FDCAN_CLASSIC_CAN, // default to classic frame
		.IdType = FDCAN_STANDARD_ID, // default to standard ID
		.BitRateSwitch = FDCAN_BRS_OFF, // no bitrate switch
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE, // error active
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS, // don't record tx events
      .MessageMarker = 0, // ?
	};

	if (frame->can_id & CAN_EFF_FLAG) {
		frame_header.IdType = FDCAN_EXTENDED_ID;
	   frame_header.Identifier = frame->can_id & 0x1FFFFFFF;
	} else {
	   frame_header.IdType = FDCAN_STANDARD_ID;
   	frame_header.Identifier = frame->can_id & 0x7FF;
	}

	if (frame->can_id & CAN_RTR_FLAG) {
		frame_header.TxFrameType = FDCAN_REMOTE_FRAME;
	}

	if (frame->flags & GS_CAN_FLAG_FD) {
		frame_header.FDFormat = FDCAN_FD_CAN;

		//if (frame->flags & GS_CAN_FLAG_BRS) {
			frame_header.BitRateSwitch = FDCAN_BRS_ON;
		//}

		if (frame->flags & GS_CAN_FLAG_ESI) {
			frame_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		}
	}
 
   // Convert to HAL code
   frame_header.DataLength = (frame->can_dlc & 0x0f) << 16;

   // Transmit can frame
   HAL_StatusTypeDef status;
   status = HAL_FDCAN_AddMessageToTxFifoQ(&hcan->handle, &frame_header, frame->raw_data);

   if (status == HAL_OK) {
    	return true;
   } else {
    	return false;
   }
}

void can_get_error_status(can_data_t *hcan, FDCAN_ProtocolStatusTypeDef *status, FDCAN_ErrorCountersTypeDef *counters)
{

	HAL_FDCAN_GetProtocolStatus(&hcan->handle, status);
	HAL_FDCAN_GetErrorCounters(&hcan->handle, counters);
}

static bool status_is_active(FDCAN_ProtocolStatusTypeDef *status)
{
	return !(status->BusOff || status->ErrorPassive);
}

bool can_parse_error_status(can_data_t *hcan, struct gs_host_frame *frame, FDCAN_ProtocolStatusTypeDef *status, FDCAN_ErrorCountersTypeDef *counters)
{
   /* We build up the detailed error information at the same time as we decide
	 * whether there's anything worth sending. This variable tracks that final
	 * result. */
	bool should_send = false;

	FDCAN_ProtocolStatusTypeDef last_status = hcan->status_old;
	HAL_FDCAN_GetProtocolStatus(&hcan->handle, status);

	hcan->status_old = *status;

	frame->flags = 0;
	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->classic_can.data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->classic_can.data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->classic_can.data[2] = CAN_ERR_PROT_UNSPEC;
	frame->classic_can.data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->classic_can.data[4] = CAN_ERR_TRX_UNSPEC;
	frame->classic_can.data[5] = 0;
	frame->classic_can.data[6] = 0;
	frame->classic_can.data[7] = 0;

	if (status->BusOff) {
		if (!last_status.BusOff) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
		// - tec (overflowed) / rec (looping, likely used for recessive counting)
		//   are not valid in the bus-off state.
		// - The warning flags remains set, error passive will cleared.
		// - LEC errors will be reported, while the device isn't even allowed to send.
		//
		// Hence only report bus-off, ignore everything else.
		return should_send;
	}

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(&last_status) && status_is_active(status)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->classic_can.data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	 * place as any. */
	frame->classic_can.data[6] = counters->TxErrorCnt;
	frame->classic_can.data[7] = counters->RxErrorCnt;

	if (status->ErrorPassive) {
		if (!last_status.ErrorPassive) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	} else if (status->Warning) {
		if (!last_status.Warning) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	for (uint8_t i = 0; i < 2; i++) {
		uint8_t lec = (i == 0) ? status->LastErrorCode : status->DataLastErrorCode;

		switch (lec) {
			case 0x01: /* stuff error */
				frame->can_id |= CAN_ERR_PROT;
				frame->classic_can.data[2] |= CAN_ERR_PROT_STUFF;
				should_send = true;
				break;
			case 0x02: /* form error */
				frame->can_id |= CAN_ERR_PROT;
				frame->classic_can.data[2] |= CAN_ERR_PROT_FORM;
				should_send = true;
				break;
			case 0x03: /* ack error */
				frame->can_id |= CAN_ERR_ACK;
				should_send = true;
				break;
			case 0x04: /* bit recessive error */
				frame->can_id |= CAN_ERR_PROT;
				frame->classic_can.data[2] |= CAN_ERR_PROT_BIT1;
				should_send = true;
				break;
			case 0x05: /* bit dominant error */
				frame->can_id |= CAN_ERR_PROT;
				frame->classic_can.data[2] |= CAN_ERR_PROT_BIT0;
				should_send = true;
				break;
			case 0x06: /* CRC error */
				frame->can_id |= CAN_ERR_PROT;
				frame->classic_can.data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
				should_send = true;
				break;
			default: /* 0=no error, 7=no change */
				break;
		}
	}

	return should_send;
}
