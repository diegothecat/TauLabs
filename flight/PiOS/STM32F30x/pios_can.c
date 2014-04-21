/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_CAN PiOS CAN interface layer
 * @brief CAN interface for PiOS
 * @{
 *
 * @file       pios_can.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 * @brief      PiOS CAN interface header
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


#include "pios.h"
#include "pios_com.h"
#include "pios_com_priv.h"

#if defined(PIOS_INCLUDE_CAN)

#include "pios_can_priv.h"

#if defined(PIOS_INCLUDE_DIEGO_ESC)
#  include <candefs.h>
#  define MY_ADDRESS 0xA
#  define DEBUG_PRINT_MSG
#endif

/* Provide a COM driver */
static void PIOS_CAN_RegisterRxCallback(uintptr_t can_id, pios_com_callback rx_in_cb, uintptr_t context);
static void PIOS_CAN_RegisterTxCallback(uintptr_t can_id, pios_com_callback tx_out_cb, uintptr_t context);
static void PIOS_CAN_TxStart(uintptr_t can_id, uint16_t tx_bytes_avail);
static void PIOS_CAN_RxStart(uintptr_t can_id, uint16_t rx_bytes_avail);

static bool GetCanTxMsg(CanTxMsg *msg, bool *ptx_need_yield);
static void ReceiveCanMsg(struct pios_can_msgheader *canMsgHdr, CanRxMsg *canMsg);

const struct pios_com_driver pios_can_com_driver = {
	.tx_start   = PIOS_CAN_TxStart,
	.rx_start   = PIOS_CAN_RxStart,
	.bind_tx_cb = PIOS_CAN_RegisterTxCallback,
	.bind_rx_cb = PIOS_CAN_RegisterRxCallback,
};

enum pios_can_dev_magic {
	PIOS_CAN_DEV_MAGIC = 0x41fa834A,
};

//! Structure for an initialized CAN handle
struct pios_can_dev {
	enum pios_can_dev_magic     magic;
	const struct pios_can_cfg  *cfg;
	pios_com_callback rx_in_cb;
	uintptr_t rx_in_context;
	pios_com_callback tx_out_cb;
	uintptr_t tx_out_context;
};

// Local constants
#define CAN_COM_ID      0x11
#define MAX_SEND_LEN    8


#define CAN_MAX_FILTER_BANKS  14 // STM32F3 has 14 filter banks
#define CAN_MAX_FILTERS  (CAN_MAX_FILTER_BANKS * 4) // Each filter bank can have 1,2 or 4 filters

static int NumFilters;
static int NumFilterBanks;

/*
 * RxMessage->FMI is the filter match index. Map to corresponding channel.
 */
static struct pios_can_channel *FilterMapping[CAN_MAX_FILTERS];



void USB_HP_CAN1_TX_IRQHandler(void);

static bool PIOS_CAN_validate(struct pios_can_dev *can_dev)
{
	return (can_dev->magic == PIOS_CAN_DEV_MAGIC);
}

#if !defined(PIOS_INCLUDE_FREERTOS)
#error PIOS_CAN REQUIRES FREERTOS
#endif

static struct pios_can_dev *PIOS_CAN_alloc(void)
{
	struct pios_can_dev *can_dev;

	can_dev = (struct pios_can_dev *)PIOS_malloc(sizeof(*can_dev));
	if (!can_dev) return(NULL);

	memset(can_dev, 0, sizeof(*can_dev));
	can_dev->magic = PIOS_CAN_DEV_MAGIC;

	return(can_dev);
}

//! The local handle for the CAN device
static struct pios_can_dev *can_dev;

/**
 * Initialize the CAN driver and return an opaque id
 * @param[out]   id the CAN interface handle
 * @param[in]    cfg the configuration structure
 * @return 0 if successful, negative otherwise
 */
int32_t PIOS_CAN_Init(uintptr_t *can_id, const struct pios_can_cfg *cfg)
{
	CAN_DEBUG_Assert(can_id);
	CAN_DEBUG_Assert(cfg);

	can_dev = (struct pios_can_dev *) PIOS_CAN_alloc();
	if (!can_dev) goto out_fail;

	/* Bind the configuration to the device instance */
	can_dev->cfg = cfg;

	/* Configure the CAN device */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* Map pins to CAN function */
	if (can_dev->cfg->remap) {
		if (can_dev->cfg->rx.gpio != 0)
			GPIO_PinAFConfig(can_dev->cfg->rx.gpio,
				can_dev->cfg->rx.pin_source,
				can_dev->cfg->remap);
		if (can_dev->cfg->tx.gpio != 0)
			GPIO_PinAFConfig(can_dev->cfg->tx.gpio,
				can_dev->cfg->tx.pin_source,
				can_dev->cfg->remap);
	}

	/* Initialize the CAN Rx and Tx pins */
	if (can_dev->cfg->rx.gpio != 0)
		GPIO_Init(can_dev->cfg->rx.gpio, (GPIO_InitTypeDef *)&can_dev->cfg->rx.init);
	if (can_dev->cfg->tx.gpio != 0)
		GPIO_Init(can_dev->cfg->tx.gpio, (GPIO_InitTypeDef *)&can_dev->cfg->tx.init);

	*can_id = (uintptr_t)can_dev;

	CAN_DeInit(can_dev->cfg->regs);
	CAN_Init(can_dev->cfg->regs, (CAN_InitTypeDef *)&can_dev->cfg->init);

	// Enable the receiver IRQ
 	NVIC_Init((NVIC_InitTypeDef*) &can_dev->cfg->rx_irq.init);
 	NVIC_Init((NVIC_InitTypeDef*) &can_dev->cfg->tx_irq.init);

	return(0);

out_fail:
	return(-1);
}

static void PIOS_CAN_RxStart(uintptr_t can_id, uint16_t rx_bytes_avail)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;
	
   CAN_DEBUG_Assert(PIOS_CAN_validate(can_dev));
	
	CAN_ITConfig(can_dev->cfg->regs, CAN_IT_FMP1, ENABLE);
}

static void PIOS_CAN_TxStart(uintptr_t can_id, uint16_t tx_bytes_avail)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;
	
   CAN_DEBUG_Assert(PIOS_CAN_validate(can_dev));

 	CAN_ITConfig(can_dev->cfg->regs, CAN_IT_TME, ENABLE);
	
 	// Try to start the transfer. If any mbox is not empty, the ISR is called if mbox goes empty.
	USB_HP_CAN1_TX_IRQHandler();
}

static void PIOS_CAN_RegisterRxCallback(uintptr_t can_id, pios_com_callback rx_in_cb, uintptr_t context)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;

   CAN_DEBUG_Assert(PIOS_CAN_validate(can_dev));
	
	/* 
	 * Order is important in these assignments since ISR uses _cb
	 * field to determine if it's ok to dereference _cb and _context
	 */
	can_dev->rx_in_context = context;
	can_dev->rx_in_cb = rx_in_cb;
}

static void PIOS_CAN_RegisterTxCallback(uintptr_t can_id, pios_com_callback tx_out_cb, uintptr_t context)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;

   CAN_DEBUG_Assert(PIOS_CAN_validate(can_dev));
	
	/* 
	 * Order is important in these assignments since ISR uses _cb
	 * field to determine if it's ok to dereference _cb and _context
	 */
	can_dev->tx_out_context = context;
	can_dev->tx_out_cb = tx_out_cb;
}


#if defined(DEBUG_PRINT_MSG)
static void PrinCanMsg(const char *prfx, uint32_t extId, uint8_t *canbuf, uint8_t buflen)
{
   if (buflen <= 0) return;

#define BUFLEN 64
   char buf[BUFLEN];

   uint8_t slen = sprintf(buf, "%s 0x%08x (%d) ", prfx, (unsigned int) extId, buflen);
   for (int i = 0; i < buflen; ++i) slen += sprintf(buf + slen, "%02x:", canbuf[i]);
   slen += sprintf(buf + slen, "\r\n");

   PIOS_COM_SendBuffer(pios_com_debug_id, (uint8_t *) buf, slen);
}
#endif // DEBUG_PRINT_MSG

/**
 * @brief  This function handles CAN1 RX1 request.
 * @note   We are using RX1 instead of RX0 to avoid conflicts with the
 *         USB IRQ handler.
 */
void CAN1_RX1_IRQHandler(void)
{
   CAN_ClearITPendingBit(can_dev->cfg->regs, CAN_IT_FMP1);

   CAN_DEBUG_Assert(PIOS_CAN_validate(can_dev));

   bool rx_need_yield = false;

   uint8_t buf[sizeof(struct pios_can_msgheader) + PIOS_CAN_MAX_LEN] __attribute__ ((aligned (4)));
   struct pios_can_msgheader *canMsgHdr = (struct pios_can_msgheader *)buf;
   CanRxMsg rxMsg = { 0 };

   // Each FIFO has 3 mailboxes. Read until FIFO1 is empty
   for (int i = 0; (can_dev->cfg->regs->RF1R & CAN_RF1R_FMP1) && i < 3; ++i)
   {
      // Receive msg from hardware
      ReceiveCanMsg(canMsgHdr, &rxMsg);

      if (rxMsg.DLC)  memcpy(buf + sizeof(struct pios_can_msgheader), rxMsg.Data, rxMsg.DLC);

      CAN_DEBUG_Assert(rxMsg.FMI < CAN_MAX_FILTERS);

      // Lookup the channel based on the filter match index
      struct pios_can_channel *can_ch = FilterMapping[rxMsg.FMI];
      CAN_DEBUG_Assert(can_ch && can_ch->magic == PIOS_CAN_CH_MAGIC);

      bool yield = false;
      // Write received msg into the channel FIFO. This is actually PIOS_COM_RxInCallback()
      (void) (can_ch->rx_in_cb)(can_ch->com_id, buf,
            sizeof(struct pios_can_msgheader) + canMsgHdr->DLC, NULL, &yield);
      rx_need_yield |= yield;

#  if defined(DEBUG_PRINT_MSG) && 0
      PrinCanMsg("RX", rxMsg.ExtId, rxMsg.Data, rxMsg.DLC);
#  endif
   }

   portEND_SWITCHING_ISR(rx_need_yield);
}

/**
 * @brief  This function handles CAN1 TX irq and sends more data if available
 */
void USB_HP_CAN1_TX_IRQHandler(void)
{
   CAN_ClearITPendingBit(can_dev->cfg->regs, CAN_IT_TME);

   //PIOS_COM_SendFormattedString(pios_com_debug_id, "CANTXINT TX (%x)\r\n", can_dev->cfg->regs->TSR);

   CAN_DEBUG_Assert(PIOS_CAN_validate(can_dev));

   bool tx_need_yield = false;

   // There a 3 TX mailboxes. Write up to three mailboxes, if empty.
   for (int i = 0; (can_dev->cfg->regs->TSR & CAN_TSR_TME) && i < 3; ++i)
   {
      // Any of the 3 TX mailboxes is empty

      bool yield = false;
      CanTxMsg msg;
      if (!GetCanTxMsg(&msg, &yield))
      {
         CAN_ITConfig(can_dev->cfg->regs, CAN_IT_TME, DISABLE);
         break;
      }
      tx_need_yield |= yield;

#  if defined(DEBUG_PRINT_MSG) && 0
      if (msg.ExtId != 0x10230f1a)  PrinCanMsg("TX", msg.ExtId, msg.Data, msg.DLC);
#  endif

      // Send message and get mailbox number
      CAN_Transmit(can_dev->cfg->regs, &msg);

   } // for

   portEND_SWITCHING_ISR(tx_need_yield);
}

static bool GetCanTxMsg(CanTxMsg *msg, bool *ptx_need_yield)
{
   struct pios_can_msgheader canMsgHdr = { 0 };

   // Get CAN message header from FIFO
   uint16_t msgLen = (can_dev->tx_out_cb)
         (can_dev->tx_out_context, (uint8_t *) &canMsgHdr, sizeof(canMsgHdr), NULL, ptx_need_yield);
   if (! msgLen) return false;

   CAN_DEBUG_Assert(msgLen == sizeof(canMsgHdr) && canMsgHdr.DLC <= MAX_SEND_LEN);

   // Get CAN payload data from FIFO
   msgLen = canMsgHdr.DLC
      ? (can_dev->tx_out_cb)(can_dev->tx_out_context, (uint8_t *) msg->Data, canMsgHdr.DLC, NULL, ptx_need_yield)
      : 0;

   CAN_DEBUG_Assert(msgLen == canMsgHdr.DLC);

   msg->StdId = canMsgHdr.CanId;
   msg->ExtId = canMsgHdr.CanId;
   msg->IDE = (canMsgHdr.IDE == PIOS_CAN_ID_EXT) ? CAN_Id_Extended : CAN_Id_Standard;
   msg->RTR = CAN_RTR_DATA;
   msg->DLC = msgLen;

   return true;
}

static void ReceiveCanMsg(struct pios_can_msgheader *canMsgHdr, CanRxMsg *canMsg)
{
   CAN_Receive(CAN1, CAN_FIFO1, canMsg);

   if (canMsg->IDE == CAN_Id_Extended)
   {
      canMsgHdr->CanId = canMsg->ExtId;
      canMsgHdr->IDE = PIOS_CAN_ID_EXT;
   }
   else
   {
      canMsgHdr->CanId = canMsg->StdId;
      canMsgHdr->IDE = PIOS_CAN_ID_STD;
   }
   canMsgHdr->DLC = canMsg->DLC;
}

static void CanFilterAddIdMask32(uint8_t filterNo, uint32_t mask, uint32_t filter)
{
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  CAN_FilterInitStructure.CAN_FilterNumber = filterNo;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = (filter << 3) >> 16; // Low 16 bits of CAN_FxR2
  CAN_FilterInitStructure.CAN_FilterIdLow = (filter == 0) ? 0 : (filter << 3 | 4); // Low 16 bits of CAN_FxR1
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (mask << 3) >> 16; // Upper 16 bits of CAN_FxR2
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = (filter == 0) ? 0 : (mask << 3 | 4); // Upper 16 bits of CAN_FxR1
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO1;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;

  CAN_FilterInit(&CAN_FilterInitStructure);
}

static void CanFilterAddIdList16(uint8_t filterNo, uint16_t id1, uint16_t id2, uint16_t id3, uint16_t id4)
{
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  CAN_FilterInitStructure.CAN_FilterNumber = filterNo;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = id1 << 5; // Low 16 bits of CAN_FxR2
  CAN_FilterInitStructure.CAN_FilterIdLow = id2 << 5; // Low 16 bits of CAN_FxR1
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = id3 << 5 | 0x10; // Upper 16 bits of CAN_FxR2
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = id4 << 5; // Upper 16 bits of CAN_FxR1
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO1;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;

  CAN_FilterInit(&CAN_FilterInitStructure);
}

/*
 * Public API
 */

pios_can_filterid PIOS_COM_CAN_AddFilter(uint32_t channel_com_id, const struct pios_can_filter *filter)
{
   CAN_DEBUG_Assert(NumFilterBanks <= CAN_MAX_FILTER_BANKS);

   int filterBankIdx = NumFilterBanks;
   struct pios_can_channel *can_ch = (struct pios_can_channel *)PIOS_COM_GetLowerId(channel_com_id);
   CAN_DEBUG_Assert(can_ch && can_ch->magic == PIOS_CAN_CH_MAGIC);

   if (filter->Flags & FILTER_FLAG_EXTID) // EXT_ID (32 Bit)
   {
      if (filter->Flags & FILTER_FLAG_IDLIST) // ID List (32-bit)
      {
         CAN_DEBUG_Assert(0);
         return PIOS_CAN_INVALID_FILTERID; // TODO
      }
      else // ID Mask (32-bit)
      {
         CanFilterAddIdMask32(filterBankIdx, filter->IdMask32.CanMask, filter->IdMask32.CanId);

         FilterMapping[NumFilters++] = can_ch;
      }
   }
   else // STD_ID (16 Bit)
   {
      if (filter->Flags & FILTER_FLAG_IDLIST) // ID List (16-bit)
      {
         CanFilterAddIdList16(filterBankIdx, filter->IdList16.CanId1, filter->IdList16.CanId2,
                                             filter->IdList16.CanId3, filter->IdList16.CanId4);

         FilterMapping[NumFilters++] = can_ch;
         FilterMapping[NumFilters++] = can_ch;
         FilterMapping[NumFilters++] = can_ch;
         FilterMapping[NumFilters++] = can_ch;
      }
      else // ID Mask (16-bit)
      {
         CAN_DEBUG_Assert(0);
         return PIOS_CAN_INVALID_FILTERID; // TODO
      }
   }

   NumFilterBanks++;
   return filterBankIdx;
}

#endif /* PIOS_INCLUDE_CAN */
/**
 * @}
 * @}
 */
