/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_COM COM MSG layer functions
 * @brief Hardware communication layer
 * @{
 *
 * @file       pios_com_msg.c  
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      COM MSG layer functions
 * @see        The GNU Public License (GPL) Version 3
 * 
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Project Includes */
#include "pios.h"
#include "pios_com.h"
#include "pios_com_priv.h"
#include "pios_can.h"
#include "pios_can_priv.h"

static void PIOS_CAN_RegisterRxChannelCallback(uintptr_t ch_id, pios_com_callback rx_in_cb, uintptr_t context);

const struct pios_com_driver pios_can_com_channel = {
   .tx_start   = NULL,
   .rx_start   = NULL,
   .bind_tx_cb = NULL,
   .bind_rx_cb = PIOS_CAN_RegisterRxChannelCallback,
};

static void PIOS_CAN_RegisterRxChannelCallback(uintptr_t ch_id, pios_com_callback rx_in_cb, uintptr_t context)
{
   struct pios_can_channel *can_ch = (struct pios_can_channel *)ch_id;
   can_ch->rx_in_cb = rx_in_cb;
}

uint32_t PIOS_COM_CAN_CreateChannel(uint32_t com_id)
{
   struct pios_can_channel *can_ch = NULL;

   can_ch = (struct pios_can_channel *) PIOS_malloc(sizeof(*can_ch));
   CAN_Assert(can_ch);
   memset(can_ch, 0, sizeof(*can_ch));
   can_ch->magic = PIOS_CAN_CH_MAGIC;

   int32_t ret = PIOS_COM_Init(&can_ch->com_id, &pios_can_com_channel,
         (uintptr_t)can_ch, can_ch->rx_buf, CAN_CHANNEL_BUFSIZE, NULL, 0);
   CAN_DEBUG_Assert(ret >= 0);
   CAN_DEBUG_Assert(can_ch->com_id);

   return can_ch->com_id;
}

int32_t PIOS_COM_CAN_SendMsg(uint32_t com_id,
      const struct pios_can_msgheader *msgHdr, const uint8_t *payload)
{
   CAN_DEBUG_Assert(msgHdr->DLC <= PIOS_CAN_MAX_LEN);

   uint8_t buf[sizeof(struct pios_can_msgheader) + PIOS_CAN_MAX_LEN];
   memcpy(buf, msgHdr, sizeof(struct pios_can_msgheader));
   if (msgHdr->DLC > 0)  memcpy(buf + sizeof(struct pios_can_msgheader), payload, msgHdr->DLC);

   return PIOS_COM_SendBufferNonBlocking(com_id, buf, sizeof(struct pios_can_msgheader) + msgHdr->DLC);
}

uint16_t PIOS_COM_CAN_ReceiveMsg(uint32_t channel_id,
      struct pios_can_msgheader *msgHdr, uint8_t *payload, uint32_t timeout_ms)
{
   uint16_t bytesReceived = PIOS_COM_ReceiveBuffer(channel_id, (uint8_t *)msgHdr,
         sizeof(*msgHdr), timeout_ms);
   if (bytesReceived == 0) return 0;

   CAN_DEBUG_Assert(bytesReceived == sizeof(*msgHdr) && msgHdr->DLC <= PIOS_CAN_MAX_LEN);

   bytesReceived += PIOS_COM_ReceiveBuffer(channel_id, payload, msgHdr->DLC, timeout_ms);

   return bytesReceived;
}


/**
 * @}
 * @}
 */
