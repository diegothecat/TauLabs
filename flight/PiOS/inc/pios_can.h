/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_CAN PiOS CAN interface layer
 * @brief CAN interface for PiOS
 * @{
 *
 * @file       pios_can.h
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

#if !defined(PIOS_CAN_H)
#define PIOS_CAN_H

#define MAX_CANESC_CNT 4 // TODO move
#define MY_CAN_ADDRESS 0xA // TODO move

extern uintptr_t pios_com_can_id;

#define PIOS_CAN_ID_STD  0  /*!< Standard Id */
#define PIOS_CAN_ID_EXT  1  /*!< Extended Id */
#define PIOS_CAN_MAX_LEN  8

#define PIOS_CAN_INVALID_CHANNELID 0xFFFFFFFF
#define PIOS_CAN_INVALID_FILTERID 0xFFFFFFFF

#define FILTER_FLAG_EXTID  0x1
#define FILTER_FLAG_IDLIST  0x2

struct pios_can_msgheader
{
  uint32_t CanId;       /*!< Specifies the standard or extended identifier.
                           This parameter can be a value between 0 to 0x7FF (standard)
                           or between 0 to 0x1FFFFFFF (extended). */

  struct
  {
     uint8_t IDE:1;     /*!< Specifies the type of identifier for the message that
                           will be transmitted. This parameter can be a value
                           of PIOS_CAN_ID_STD or PIOS_CAN_ID_EXT */

     uint8_t DLC:4;     /*!< Specifies the length of the frame that will be
                           transmitted. This parameter can be a value between
                           0 to 8 */
  };
};

struct pios_can_filter
{
   uint8_t Flags;        // FILTER_FLAG_EXTID: Fill IdMask32 or IdList32
                         // FILTER_FLAG_IDLIST: Fill IdList32 or IdList16
   union
   {
      struct
      {
         uint32_t CanId;       // Specifies the standard or extended identifier.
                               // This parameter can be a value between 0 to 0x7FF (standard)
                               // or between 0 to 0x1FFFFFFF (extended).
         uint32_t CanMask;     // A filter matches, when <received_can_id> & mask == can_id & mask
      } IdMask32;

      struct
      {
         uint16_t CanId1;
         uint16_t CanMask1;
         uint16_t CanId2;
         uint16_t CanMask2;
      } IdMask16;   // STM32 specific extension. Not supported for all platforms.

      struct
      {
         uint32_t CanId1;
         uint32_t CanId2;
      } IdList32;   // STM32 specific extension. Not supported for all platforms.

      struct
      {
         uint16_t CanId1;
         uint16_t CanId2;
         uint16_t CanId3;
         uint16_t CanId4;
      } IdList16;   // STM32 specific extension. Not supported for all platforms.
   };
};

typedef uint32_t pios_can_channelid; /*!< Channel ID */
typedef uint32_t pios_can_filterid;  /*!< Filter ID */

extern int32_t PIOS_COM_CAN_SendMsg(uint32_t com_id, const struct pios_can_msgheader *msgHdr, const uint8_t *payload);
/*
 * PIOS_COM_CAN_CreateChannel() creates a virtual RX channel. Each channel has 0..n filter's associated
 * and there is a separate RX FIFO for each channel.
 * The channel_id returned should be used as input parameter to PIOS_COM_CAN_ReceiveMsg().
 */
extern uint32_t PIOS_COM_CAN_CreateChannel(uint32_t com_id);
extern pios_can_filterid PIOS_COM_CAN_AddFilter(uint32_t channel_id, const struct pios_can_filter *filter);
extern uint16_t PIOS_COM_CAN_ReceiveMsg(uint32_t channel_id, struct pios_can_msgheader *msgHdr,
                                        uint8_t *payload, uint32_t timeout_ms);

#endif /* PIOS_CAN_H */

/**
 * @}
 * @}
 */
