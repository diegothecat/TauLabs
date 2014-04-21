/**
 ******************************************************************************
 *
 * @file       DiegoESC.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Example module to be used as a template for actual modules.
 *             Threaded periodic version.
 *
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

#include "openpilot.h"
#include "pios.h"

#if defined(PIOS_INCLUDE_DIEGO_ESC)

// Shared headers
#include <candefs.h>
#include <utils.h>

//#include "diegoescfeedback.h"
#include "diegoescconfig.h"

// Private constants
#define STACK_SIZE_BYTES  1024
#define TASK_PRIORITY (tskIDLE_PRIORITY+1)
#define CYCLE_PERIOD_MS  50
#undef DEBUG_PRINT
#define MIN(x,y) ((x) < (y) ? (x) : (y))

// Private types
typedef struct
{
   uint8_t LastCfgSeqNo;
   uint8_t BytesRcvd;
   uint8_t PayloadLen;
   uint8_t Buf[CAN_CFG_BUFLEN];
   uint8_t *BufPtr;
} CfgMsgState;

// Private variables
static xTaskHandle taskHandle;
//static DiegoESCFeedbackData diegoESCFeedbackData;
static DiegoESCConfigData diegoESCConfigData;
static volatile bool diegoESCConfigUpdated;
static CfgMsgState gCfgMsgState[MAX_CANESC_CNT];
static uint32_t canChId;

// Private functions
static void DiegoESCTask(void *parameters);
static void DiegoESCRequestConfig(uint8_t escAddr);
static void DiegoESCProcessConfig();
static void DiegoESCProcessCANMsgs();
static uint16_t DiegoESCEncodeConfig(DiegoESCConfigData *cfg, uint8_t *buf);
static void DiegoESCSendConfig(uint8_t *buf, uint16_t buflen, uint8_t escAddr);
static void DiegoESCConfigUpdatedCb(UAVObjEvent * ev);
#ifdef DEBUG_PRINT
static void DiegoESCPrintConfig(DiegoESCConfigData *cfg);
#endif

/**
 * Initialize the module, called on startup
 * \returns 0 on success or -1 if initialization failed
 */
static int32_t DiegoESCInitialize()
{
   //DiegoESCFeedbackInitialize();
   DiegoESCConfigInitialize();
   DiegoESCConfigConnectCallback(DiegoESCConfigUpdatedCb);

	// Create virtual channel...
   canChId = PIOS_COM_CAN_CreateChannel(pios_com_can_id);

   // ...and attach message filter.
   struct pios_can_filter canFilter;
   canFilter.Flags = FILTER_FLAG_EXTID;
   canFilter.IdMask32.CanMask = (0xF << CAN_DSTUNIT_SHIFT)
                     | (0xF0 << CAN_CMD_SHIFT);
/*
   // Message Filter for CAN_BLDC_CMD_INF
   canFilter.IdMask32.CanId = (CAN_UNIT_FCTRL << CAN_DSTUNIT_SHIFT)
                   | (CAN_BLDC_CMD_INF << CAN_CMD_SHIFT);
   PIOS_COM_CAN_AddFilter(canChId, &canFilter);
*/
   // Message Filter for CAN_BLDC_CMD_CFG
   canFilter.IdMask32.CanId = (CAN_UNIT_FCTRL << CAN_DSTUNIT_SHIFT)
                   | (CAN_BLDC_CMD_CFG << CAN_CMD_SHIFT);
   PIOS_COM_CAN_AddFilter(canChId, &canFilter);

   return 0;
}

/**
 * Start the task.  Expects all objects to be initialized by this point.
 * \returns 0 on success or -1 if initialization failed
 */
static int32_t DiegoESCStart(void)
{
   // Start main task
   xTaskCreate(DiegoESCTask, (signed char *)"DiegoESC", STACK_SIZE_BYTES / 4, NULL, TASK_PRIORITY, &taskHandle);
   TaskMonitorAdd(TASKINFO_RUNNING_DIEGOESC, taskHandle);

   return 0;
}

MODULE_INITCALL(DiegoESCInitialize, DiegoESCStart)

/**
 * Module thread, should not return.
 */
static void DiegoESCTask(void *parameters)
{
   // Main task loop
   portTickType lastSysTime = xTaskGetTickCount();

   for (;;)
   {
      if (diegoESCConfigUpdated)
      {
         diegoESCConfigUpdated = false;
         DiegoESCProcessConfig();
      }

      DiegoESCProcessCANMsgs();

      vTaskDelayUntil(&lastSysTime, MS2TICKS(CYCLE_PERIOD_MS));
   }
}


#if 0
// Round to nearest value, keeping 2 decimal places.
// E.g. 10.1299 -> 10.13
static float rnd(float val)
{
   return floorf(val * 100 + 0.5f) / 100;
}

static void DiegoESCUpdateFeedback(const struct pios_can_msgheader *msgHdr, const uint8_t *buf)
{
   uint64_t payload = UnalignedRd64(buf);

   //DEBUG_PRINTF(0, "CAN rx %d 0x%x\r\n", bytesRcvd, canExtId);

   uint8_t blcIdx = msgHdr->CanId & 0xF;
   if (blcIdx < 0 || blcIdx >= MAX_CANESC_CNT) return;

   /*
     PAYLOAD: Cmd 2.1 (INF.SENSOR), 8 Byte:
     12 bit: Motor current (CentiAmpere, A / 100)
     12 bit: Battery voltage (CentiVolt, V / 100)
     12 bit: Motor RPM (Deci-RPM, RPM / 10)
     12 bit: Duty cycle (scaled to 12 bit, 0..4095). 4095 -> 100%
     16 bit: Status word. \sa EBldcStatusWord
    */

   // Motor current
   uint16_t tmp = (uint16_t) (payload & 0xFFF);
   diegoESCFeedbackData.Current[blcIdx] = rnd(tmp / 100.0f);

   // Battery voltage
   tmp = (uint16_t) ((payload >> 12) & 0xFFF);
   diegoESCFeedbackData.BatteryVoltage[blcIdx] = rnd(tmp / 100.0f);

   // RPM
   tmp = (uint16_t) ((payload >> 24) & 0xFFF);
   diegoESCFeedbackData.RPM[blcIdx] = tmp * 10;

   // Duty cycle
   tmp = (uint16_t) ((payload >> 36) & 0xFFF);
   diegoESCFeedbackData.PWMDuty[blcIdx] = rnd(tmp * 100.0f / 0xFFF);

   // P = U * I
   diegoESCFeedbackData.PowerConsumption[blcIdx] = rnd(diegoESCFeedbackData.BatteryVoltage[blcIdx]
                                                       * diegoESCFeedbackData.Current[blcIdx]);

   // Status word
   tmp = (uint16_t) (payload >> 48);

   uint8_t *errorFields = NULL;
   switch (blcIdx)
   {
      case 0: errorFields = diegoESCFeedbackData.ErrorESC0; break;
      case 1: errorFields = diegoESCFeedbackData.ErrorESC1; break;
      case 2: errorFields = diegoESCFeedbackData.ErrorESC2; break;
      case 3: errorFields = diegoESCFeedbackData.ErrorESC3; break;
   }

   if (errorFields == NULL) return;

   for (int i = 0; i < DIEGOESCFEEDBACK_ERRORESC0_NUMELEM; ++i)
   {
      errorFields[i] = (tmp & (1 << i)) ? 1 : 0;
   }

   DiegoESCFeedbackSet(&diegoESCFeedbackData);
}
#endif


static void DiegoESCConfigUpdatedCb(UAVObjEvent * ev)
{
   diegoESCConfigUpdated = true;
}

static void DiegoESCProcessConfig()
{
   DiegoESCConfigGet(&diegoESCConfigData);
   uint8_t escAddr = diegoESCConfigData.CmdAddr &= 0xF;
   uint8_t cmd = diegoESCConfigData.Cmd;

#ifdef DEBUG_PRINT
   DiegoESCPrintConfig(&diegoESCConfigData);
#endif

   if (cmd == DIEGOESCCONFIG_CMD_REQUESTCFG)
   {
      DiegoESCConfigSetDefaults(DiegoESCConfigHandle(), 0);
      DiegoESCRequestConfig(escAddr);
   }
   else if (cmd == DIEGOESCCONFIG_CMD_UPDATEALL || cmd == DIEGOESCCONFIG_CMD_UPDATETHIS)
   {
      uint8_t buf[CAN_CFG_BUFLEN];
      uint16_t buflen = DiegoESCEncodeConfig(&diegoESCConfigData, buf);

      DiegoESCSendConfig(buf, buflen,
            (cmd == DIEGOESCCONFIG_CMD_UPDATETHIS) ? escAddr : 0xFF);
   }
}

static void DiegoESCRequestConfig(uint8_t escAddr)
{
   struct pios_can_msgheader canMsgHdr = { 0 };
   uint8_t dummy = 0;

   // Request configuration from specific ESC
   canMsgHdr.CanId = CAN_RSVD_MASK
          | (CAN_UNIT_BLDC << CAN_DSTUNIT_SHIFT)
          | (CAN_BLDC_CMD_CFGREQ << CAN_CMD_SHIFT)
          | (escAddr << CAN_CMDARG_SHIFT)
          | (CAN_UNIT_FCTRL << CAN_SRCUNIT_SHIFT)
          | MY_CAN_ADDRESS;
   canMsgHdr.IDE = PIOS_CAN_ID_EXT;
   canMsgHdr.DLC = 1;

   // Send one dummy payload Byte to make the PIOS CAN driver happy!
   PIOS_COM_CAN_SendMsg(pios_com_can_id, &canMsgHdr, &dummy);
}

static bool DiegoESCDesequenceCANConfigMsg(const struct pios_can_msgheader *msgHdr,
      const uint8_t *buf, uint8_t buflen, uint8_t *escIdx)
{
   uint8_t seqNo = (msgHdr->CanId >> CAN_CMD_SHIFT) & 0xF;
   const uint8_t *bufp = buf;

   *escIdx = msgHdr->CanId & 0xF;
   if (*escIdx >= MAX_CANESC_CNT) return false;

   CfgMsgState *msgState = gCfgMsgState + *escIdx;

   if (seqNo == 0)
   {
      msgState->BufPtr = msgState->Buf;
      msgState->BytesRcvd = 0;
      msgState->PayloadLen = bufp[0];
   }
   else if (seqNo != msgState->LastCfgSeqNo + 1)
   {
      return false; // Sequence error
   }

   msgState->BytesRcvd += buflen;
   if (msgState->BytesRcvd > CAN_CFG_BUFLEN)
   {
      return false; // Shouldn't happen
   }

   memcpy(msgState->BufPtr, bufp, buflen);

   msgState->BufPtr += buflen;
   msgState->LastCfgSeqNo = seqNo;

   if (msgState->BytesRcvd != msgState->PayloadLen)
   {
      return false;
   }

   return true;
}

inline static uint8_t Decode32(uint32_t *dst, uint8_t *src, uint32_t valid, uint32_t mask)
{
   if (valid & mask)
   {
      *dst = UnalignedRd32(src);
      return 4;
   }
   return 0;
}

inline static uint8_t Decode16(uint16_t *dst, uint8_t *src, uint32_t valid, uint32_t mask)
{
   if (valid & mask)
   {
      *dst = UnalignedRd16(src);
      return 2;
   }
   return 0;
}

inline static uint8_t Decode8(uint8_t *dst, uint8_t *src, uint32_t valid, uint32_t mask)
{
   if (valid & mask)
   {
      *dst = *src;
      return 1;
   }
   return 0;
}

static void DiegoESCDecodeConfig(uint8_t *buf, DiegoESCConfigData *cfg)
{
   uint8_t buf8;
   uint16_t buf16;
   uint32_t buf32;
   uint8_t *bufp = buf;

   // Skip payload length field (1 Byte)
   bufp++;

   // ValidMask
   uint32_t validMask = UnalignedRd32(bufp);
   bufp += 4;

   // Flags
   if (Decode8(&buf8, bufp, validMask, EFlFlags))
   {
      bufp++;
      if (buf8 & ECfUartEn) cfg->Flags[DIEGOESCCONFIG_FLAGS_DEBUGPRINTEN] = 1;
      if (buf8 & ECfBrakeEn) cfg->Flags[DIEGOESCCONFIG_FLAGS_BRAKEEN] = 1;
   }

   // MotorPolePairs
   bufp += Decode8(&cfg->MotorPolePairs, bufp, validMask, EFlMotorPolePairs);

   // MotorKv
   bufp += Decode16(&cfg->MotorKv, bufp, validMask, EFlMotorKv);

   // MotorMaxCurrentMa
   if (Decode16(&buf16, bufp, validMask, EFlMotorMaxCurrentMa))
   {
      bufp += 2;
      cfg->MotorMaxCurrent = buf16 / 1000.0f;
   }

   // MotorMaxPower
   bufp += Decode16(&cfg->MotorMaxPower, bufp, validMask, EflMotorMaxPower);

   // MaxAcceleration
   bufp += Decode16(&cfg->MaxAcceleration, bufp, validMask, EflMaxAcceleration);

   // MaxCycleTimeMs
   if (Decode8(&buf8, bufp, validMask, EFlMaxCycleTimeMs))
   {
      bufp++;
      cfg->MaxCycleTime = buf8;
   }

   // AlignTimeMs
   bufp += Decode16(&cfg->AlignTime, bufp, validMask, EFlAlignTimeMs);

   // AlignCurrentMa
   bufp += Decode16(&cfg->AlignCurrent, bufp, validMask, EFlAlignCurrentMa);

   // RampUpTimeMs
   bufp += Decode16(&cfg->RampUpTime, bufp, validMask, EFlRampUpTimeMs);

   // RampUpStartPeriod
   bufp += Decode16(&cfg->RampUpStartPeriod, bufp, validMask, EFlRampUpStartPeriodUs);

   // RampUpEndPeriod
   bufp += Decode16(&cfg->RampUpEndPeriod, bufp, validMask, EFlRampUpEndPeriodUs);

   // MinPwmPerMil
   if (Decode16(&buf16, bufp, validMask, EFlMinPwmPerMil))
   {
      bufp += 2;
      cfg->MinPWM = buf16 / 10.0f;
   }

   // MaxPwmPerMil
   if (Decode16(&buf16, bufp, validMask, EFlMaxPwmPerMil))
   {
      bufp += 2;
      cfg->MaxPWM = buf16 / 10.0f;
   }

   // MinBatVoltageMv
   if (Decode16(&buf16, bufp, validMask, EFlMinBatVoltageMv))
   {
      bufp += 2;
      cfg->MinBatVoltage = buf16 / 1000.0f;
   }

   // BuildNo
   if (Decode16(&buf16, bufp, validMask, EFlBuildNo))
   {
      bufp += 2;
      cfg->BuildVersion[0] = buf16;
   }

   // BuildDate
   for (int i = 0; i < 6; ++i)
   {
      if (Decode8(&buf8, bufp, validMask, EFlBuildDate))
      {
         bufp++;
         cfg->BuildDate[i] = buf8;
      }
   }

   // BuildGitHash
   if (Decode32(&buf32, bufp, validMask, EFlBuildGitHash))
   {
      bufp += 4;
      cfg->BuildVersion[1] = buf32;
   }

   // Station address
   if (Decode8(&buf8, bufp, validMask, EFlEscAddr))
   {
      bufp++;
      cfg->ESCAddr = buf8;
   }

   // SerialNo
   for (int i = 0; i < 3; ++i)
   {
      if (Decode32(&buf32, bufp, validMask, EFlSerialNo))
      {
         bufp += 4;
         cfg->HwSerialNo[i] = buf32;
      }
   }
}

static void DiegoESCUpdateConfig(const struct pios_can_msgheader *msgHdr, const uint8_t *buf, uint8_t buflen)
{
   uint8_t updatedEscIdx;
   if (DiegoESCDesequenceCANConfigMsg(msgHdr, buf, buflen, &updatedEscIdx))
   {
      // Received a complete configuration msg

      DiegoESCConfigGet(&diegoESCConfigData);
      DiegoESCDecodeConfig(gCfgMsgState[updatedEscIdx].Buf, &diegoESCConfigData);

      // PrintEscSettings(&diegoESCConfigData);

      DiegoESCConfigSet(&diegoESCConfigData);
   }
}

static void DiegoESCProcessCANMsgs()
{
   struct pios_can_msgheader msgHdr = { 0 };
   uint8_t buf[PIOS_CAN_MAX_LEN];

   for (;;)
   {
      uint16_t bytesRcvd = PIOS_COM_CAN_ReceiveMsg(canChId, &msgHdr, buf, 0);
      if (bytesRcvd <= sizeof(msgHdr)) break;

      uint8_t cmd = ((msgHdr.CanId & CAN_CMD_MASK) >> CAN_CMD_SHIFT) & 0xF0;

      switch (cmd)
      {
         case CAN_BLDC_CMD_CFG:
         {
            DiegoESCUpdateConfig(&msgHdr, buf, msgHdr.DLC);
            break;
         }
         /*
         case CAN_BLDC_CMD_INF:
         {
            if (msgHdr.DLC != 8) break;
            DiegoESCUpdateFeedback(&msgHdr, buf);
            break;
         }
         */
      }
   }
}

static uint16_t DiegoESCEncodeConfig(DiegoESCConfigData *cfg, uint8_t *buf)
{
   uint8_t *bufp = buf;

   // Allocate room for the payload length field (1 Byte)
   bufp++;

   // ValidMask
   bufp = UnalignedWr32(CAN_CFG_VALIDFLAGS_ALLMSK, bufp);

   // Flags
   bufp = UnalignedWr8(
           (cfg->Flags[DIEGOESCCONFIG_FLAGS_DEBUGPRINTEN] ? ECfUartEn : 0)
         | (cfg->Flags[DIEGOESCCONFIG_FLAGS_BRAKEEN] ? ECfBrakeEn : 0)
         | (cfg->Flags[DIEGOESCCONFIG_FLAGS_LOADDEFAULTS] ? ECfLoadDefaults : 0)
         , bufp);

   if (! cfg->Flags[DIEGOESCCONFIG_FLAGS_LOADDEFAULTS])
   {
      // MotorPolePairs
      bufp = UnalignedWr8(cfg->MotorPolePairs, bufp);

      // MotorKv
      bufp = UnalignedWr16(cfg->MotorKv, bufp);

      // MotorMaxCurrentMa
      bufp = UnalignedWr16((uint16_t) (cfg->MotorMaxCurrent * 1000.0f + 0.5f), bufp);

      // MotorMaxPower
      bufp = UnalignedWr16(cfg->MotorMaxPower, bufp);

      // MaxAcceleration
      bufp = UnalignedWr16(cfg->MaxAcceleration, bufp);

      // MaxCycleTimeMs
      bufp = UnalignedWr8(cfg->MaxCycleTime, bufp);

      // AlignTimeMs
      bufp = UnalignedWr16(cfg->AlignTime, bufp);

      // AlignCurrentMa
      bufp = UnalignedWr16(cfg->AlignCurrent, bufp);

      // RampUpTimeMs
      bufp = UnalignedWr16(cfg->RampUpTime, bufp);

      // RampUpStartPeriodUs
      bufp = UnalignedWr16(cfg->RampUpStartPeriod, bufp);

      // RampUpEndPeriodUs
      bufp = UnalignedWr16(cfg->RampUpEndPeriod, bufp);

      // MinPwmPerMil
      bufp = UnalignedWr16((uint16_t) (cfg->MinPWM * 10.0f + 0.5f), bufp);

      // MaxPwmPerMil
      bufp = UnalignedWr16((uint16_t) (cfg->MaxPWM * 10.0f + 0.5f), bufp);

      // MinBatVoltageMv
      bufp = UnalignedWr16((uint16_t) (cfg->MinBatVoltage * 1000.0f + 0.5f), bufp);
   }

   // Store payload length in first byte of the payload
   *buf = bufp - buf;

   return *buf;
}

static void DiegoESCSendConfig(uint8_t *buf, uint16_t buflen, uint8_t escAddr)
{
   struct pios_can_msgheader canMsgHdr = { 0 };
   uint8_t *bufp = buf;

   for (int i = 0; buflen > 0; ++i)
   {
      uint8_t cmd = CAN_BLDC_CMD_CFG | (i & 0XF);
      canMsgHdr.CanId = CAN_RSVD_MASK
            | (CAN_UNIT_BLDC << CAN_DSTUNIT_SHIFT)
            | ((uint32_t) cmd << CAN_CMD_SHIFT)
            | ((uint32_t) escAddr << CAN_CMDARG_SHIFT)
            | (CAN_UNIT_FCTRL << CAN_SRCUNIT_SHIFT)
            | MY_CAN_ADDRESS;
      canMsgHdr.IDE = PIOS_CAN_ID_EXT;
      canMsgHdr.DLC = MIN(buflen, CAN_MAX_PAYLOADLEN);

#if 0
      PIOS_COM_SendFormattedString(pios_com_debug_id, "CAN TX (%d)\r\n", canMsgHdr.DLC);
#endif

      // Send CAN message
      PIOS_COM_CAN_SendMsg(pios_com_can_id, &canMsgHdr, bufp);

      bufp += canMsgHdr.DLC;
      buflen -= canMsgHdr.DLC;
   }
}

#ifdef DEBUG_PRINT
static void DiegoESCPrintConfig(DiegoESCConfigData *cfg)
{
   // Send buffer is limited to 128 Byte. Print out in chunks.
   PIOS_COM_SendFormattedString(pios_com_debug_id,
           "Diego-ESC Configuration:\r\n"
           " MotorPolePairs         %d\r\n"
           " MotorKv                %d\r\n",
           cfg->MotorPolePairs,
           cfg->MotorKv);

   PIOS_COM_SendFormattedString(pios_com_debug_id,
           " MotorMaxCurrentMa      %d\r\n"
           " MotorMaxPower          %d\r\n"
           " MaxAcceleration        %d\r\n",
         (uint16_t) (cfg->MotorMaxCurrent * 1000.0f + 0.5f),
         cfg->MotorMaxPower,
         cfg->MaxAcceleration);

   PIOS_COM_SendFormattedString(pios_com_debug_id,
           " MaxCycleTimeMs         %d\r\n"
           " AlignTimeMs            %d\r\n"
           " AlignCurrentMa         %d\r\n",
           cfg->MaxCycleTime,
           cfg->AlignTime,
           cfg->AlignCurrent);

   PIOS_COM_SendFormattedString(pios_com_debug_id,
           " RampUpTimeMs           %d\r\n"
           " RampUpStartPeriodUs    %d\r\n"
           " RampUpEndPeriodUs      %d\r\n",
           cfg->RampUpTime,
           cfg->RampUpStartPeriod,
           cfg->RampUpEndPeriod);

   PIOS_COM_SendFormattedString(pios_com_debug_id,
           " MinPwmPerMil           %d\r\n"
           " MaxPwmPerMil           %d\r\n"
           " MinBatVoltageMv        %d\r\n",
           (uint16_t) (cfg->MinPWM * 100.0f + 0.5f),
           (uint16_t) (cfg->MaxPWM * 100.0f + 0.5f),
           (uint16_t) (cfg->MinBatVoltage * 1000.0f + 0.5f) );
}
#endif // DEBUG_PRINT

#endif // PIOS_INCLUDE_DIEGO_ESC
