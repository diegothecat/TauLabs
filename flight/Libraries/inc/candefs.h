/*
 * candefs.h
 *
 * Diego-ESC CAN-Bus protocol description.
 * https://sites.google.com/site/diegodrone/
 *
 * This file is included from multiple projects.
 * Don't include any header files!
 *
 *  Created on: Oct 4, 2013
 *      Author: Kai Olbrich <kai.olbrich@googlemail.com>
 */

#ifndef CANDEFS_H_
#define CANDEFS_H_

/*

 *************************************
 * DIEGO-ESC CAN protocol description
 *************************************

 Usage CAN Ext. ID (29 Bit). Generic msg format:

 Bit   2   2 2 2 2  |  2 2 2 2   1 1 1 1   |   1 1 1 1   1 1      |                      |
       8 | 7 6 5 4  |  3 2 1 0 | 9 8 7 6   |   5 4 3 2 | 1 0 9 8  |  7 6 5 4 | 3 2 1 0   |
 -----------------------------------------------------------------------------------------
       1 | Dst Unt  |  Cmd     |           |   Cmd Arg.           |  Src Unt | Src Adr   |

  0.. 3  Src Adr     Unique sender address
  4.. 7  Src Unt     Sender unit / Baugruppe (0 BLDC, 1 FLight Controller, ...)
  8..15  Cmd Arg.    Command specific
 16..23  Cmd         Command
 24..27  Dst Unt     Destination unit (0 BLDC, 1 FLight Controller, ...)
 28      Reserved    Shall be 1 (1 normal data, 0 high prio - tbd)


 Destination: Unit 0 (BLDC)
 ==========================

 Bit |  2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1     |                   |
     |  8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
 --------------------------------------------------------------------------------
        1 | Dst Unt | Cmd               | Cmd Arg.          | Src Unt | Src Adr |

 2.x:   1 | 0 0 0 0 | x x x x | y y y y | 0 0 g g | Dst Vld | Src Unt | Src Id  |
 3.0    1 | 0 0 0 0 | x x x x | SeqNo   |           Addr    | Src Unt | Src Id  |

  16..23  Cmd          20..23 Command:
                        0 Reserved
                        1 (MOTORCTRL) Motor control (Motor on / off)
                        2 (SPEED) Set desired speed
                        3 (CFG) Configuration
                        4 (CFG_REQ) Request configuration

  Cmd 2.x (SPEED.x), 8 Byte
  -------------------------

  Header:

     8..11  Dst Vld     Bit marks if payload at index i is valid.
                        E.g. for Speed cmd: Bit 0 Data[0..1]-payload valid, ..., Bit 3 Data[6..7]-payload valid.
                        E.g. for Motor cmd: Bit 0 Data[0]-payload valid, ..., Bit 3 Data[3]-payload valid.

    12..13  Dst Grp     Payload sequence no. This 2 bits identifies a sequence of payload (4 values).
                        E.g. for Speed cmd: Group0: Speed 1-4, Group1: Speed 5-8, Group2: Speed 9-12, Group3 Speed 13-16

    16..19 Sub Command
                        2.1 (SPEEDCNT) Raw desired speed as PWM counter value
                        2.2 (SPEED16) Scaled desired speed (0..0xFFFF)
                        2.3 (SPEEDOP) OpenPilot / TauLabs desired speed (0..9999). Motor is started / stopped if !0 / 0.

  Payload:

    16 bit: ESC1 (Dst. valid, Bit0)
    16 bit: ESC2 (Dst. valid, Bit1)
    16 bit: ESC3 (Dst. valid, Bit2)
    16 bit: ESC4 (Dst. valid, Bit3)


  Cmd 3 (CFG)
  -----------

  Header:

     8..13: Address of BLDC (0..64)
    13..15: 0

    16..19: SeqNo.

  Payload:

    Configuration, split over 1..n CAN msg's. SeqNo shall be 0 for the first msg,
    and incremented for the following msg's. Byte 0 of the first msg (SeqNo == 0) is
    the total payload length in Byte (including the length field).

  Cmd 4 (CFG_REQ)
  ---------------

  Header:

     8..13: Address of BLDC (0..64)
    13..15: 0

  Payload:

    Empty (0 Byte)


 Destination: Unit 1 (Flight Controller)
 =======================================

 Bit |  2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1     |                   |
     |  8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
 --------------------------------------------------------------------------------
        1 | Dst Unt | Cmd               | Cmd. Arg.         | Src Unt | Src Adr |

        1 | 0 0 0 1 | x x x x | y y y y | c c c c   c c c c | Src Unt | Src Id  |

  Header:

   8..15  Dst Addr      Broadcast to all Flight Controllers
  16..19  Sub Cmd       See below
  20..23  Cmd
                        0 Reserved
                        1 (ALARM) Alarm
                        2 (INF) Info
   8..15  Counter       Incremented for each msg.


  Cmd 2.1 (INF.SENSOR), 8 Byte
  ----------------------------

  Header:

    16..19 Sub Cmd
                        2.1 (SENSOR) Send motor current, battery voltage, motor RPM and status word

  Payload:

    12 bit: Motor current (CentiAmpere, A / 100)
    12 bit: Battery voltage (CentiVolt, V / 100)
    12 bit: Motor RPM (Deci-RPM, RPM / 10)
    12 bit: Duty cycle (scaled to 12 bit, 0..4095). 4095 -> 100%
    16 bit: Status word. \sa EBldcStatusWord


  Cmd 3 (CFG)
  -----------
    \sa BLDC CFG

*/


/******************************************************************************
 * Common defines
 */

#define CAN_MAX_PAYLOADLEN 8

#define CAN_SPEED_500k 0
#define CAN_SPEED_1000k 1

#define CAN_MAX_BLDC_CNT 16

/******************************************************************************
 * Access masks
 */

#define CAN_SRCADDR_MASK 0xF
#define CAN_SRCADDR_SHIFT 0

#define CAN_SRCUNIT_MASK 0xF0
#define CAN_SRCUNIT_SHIFT 4

#define CAN_CMD_MASK 0xFF0000
#define CAN_CMD_SHIFT 16

#define CAN_CMDARG_MASK 0xFF00
#define CAN_CMDARG_SHIFT 8

#define CAN_DSTUNIT_MASK 0xF000000
#define CAN_DSTUNIT_SHIFT 24

#define CAN_RSVD_MASK (1 << 28)



/******************************************************************************
 * Unit ID's
 */

#define CAN_UNIT_BLDC 0
#define CAN_UNIT_FCTRL 1



/******************************************************************************
 * Commands
 */

#define CAN_BLDC_CMD_MOTORCTRL 0x10 // Motor On, Off (FCTRL -> BLDC)
#define CAN_BLDC_CMD_INF 0x20 // Send status (BLDC -> FCTRL)
#define CAN_BLDC_CMD_SPEED 0x20 // Speed control (FCTRL -> BLDC)
#define CAN_BLDC_CMD_CFG 0x30 // Send Configuration (BLDC -> FCTRL, FCTRL -> BLDC)
#define CAN_BLDC_CMD_CFGREQ 0x40 // Request Configuration (FCTRL -> BLDC)
#define CAN_BLDC_CMD_BOOTLDR 0x50 // Jump into bootloader for firmware upgrade (FCTRL -> BLDC)

// CAN_BLDC_CMD_INF
#define CAN_BLDC_SUBCMD_INFSENSOR 0x20 // Broadcast sensor data

// CAN_BLDC_CMD_SPEED
#define CAN_BLDC_SUBCMD_SPEEDCNT 0x1 // Raw desired speed as PWM counter value (dependent on firmware)
#define CAN_BLDC_SUBCMD_SPEED16 0x2 // Scaled desired speed in the range from 0 (Min) to 0xFFFF (Max)
#define CAN_BLDC_SUBCMD_SPEEDOP 0x3 // OpenPilot / TauLabs desired speed in the range from 0 (Min) to 9999 (Max)
                                    // Motor is started if desired speed > 0. Motor is stopped if desired speed == 0.
#define CAN_BLDC_SPEEDMSG_LEN 8 // 16 Bit * 4 BLDC's
#define CAN_BLDC_MOTORCTRL_LEN 4 // 8 Bit * 4 BLDC's

// CAN_BLDC_CMD_MOTORCTRL
#define CAN_BLDC_MOTOR_OFF 0
#define CAN_BLDC_MOTOR_ON 1

// CAN_BLDC_CMD_CFG
#define CAN_CFG_BUFLEN 128 // Size of buffer that can store the whole raw cfg msg

typedef enum
{
   ECfUartEn = 0x1, // Enable debug output on UART
   ECfBrakeEn = 0x2, // Brake motor on stop
   ECfLoadDefaults = 0x4, // Load hardcoded defaults
} ECanCfgFlag;

typedef enum // If bit is set, corresponding variable is present
{
   EFlFlags = 1 << 0,

   EFlMotorPolePairs = 1 << 1,
   EFlMotorKv = 1 << 2,
   EFlMotorMaxCurrentMa = 1 << 3,
   EflMotorMaxPower = 1 << 4,

   EflMaxAcceleration = 1 << 5,
   EFlMaxCycleTimeMs = 1 << 6,

   EFlAlignTimeMs = 1 << 7,
   EFlAlignCurrentMa = 1 << 8,

   EFlRampUpTimeMs = 1 << 9,
   EFlRampUpStartPeriodUs = 1 << 10,
   EFlRampUpEndPeriodUs = 1 << 11,

   EFlMinPwmPerMil = 1 << 12,
   EFlMaxPwmPerMil = 1 << 13,
   EFlMinBatVoltageMv = 1 << 14,

   EFlBuildNo = 1 << 15,
   EFlBuildDate = 1 << 16,
   EFlBuildGitHash = 1 << 17,
   EFlEscAddr = 1 << 18,
   EFlSerialNo = 1 << 19,

} ECanCfgValidFlag;

#define CAN_CFG_VALIDFLAGS_ALLMSK ( \
    EFlFlags \
  | EFlMotorPolePairs | EFlMotorKv | EFlMotorMaxCurrentMa | EflMotorMaxPower \
  | EflMaxAcceleration | EFlMaxCycleTimeMs \
  | EFlAlignTimeMs | EFlAlignCurrentMa \
  | EFlRampUpTimeMs | EFlRampUpStartPeriodUs | EFlRampUpEndPeriodUs \
  | EFlMinPwmPerMil | EFlMaxPwmPerMil | EFlMinBatVoltageMv \
  | EFlBuildNo | EFlBuildDate | EFlBuildGitHash | EFlEscAddr | EFlSerialNo )

// CAN_BLDC_CMD_INF
#define CAN_BLDC_INFSENSORMSG_LEN 8 // 4 x 12 bit sensor + 16 bit status

#define SOFT_ERRORS (ESwInvalidState | ESwSpeedChangeLimitExceeded) // Soft errors: Non critical errors. Motor will keep running.
#define HARD_ERRORS (~SOFT_ERRORS)

typedef enum // 16 bit status word
{
   ESwGeneralError = 0x1, // Misc. errors
   ESwOvercurrent = 0x2,
   ESwUndervoltage = 0x4,
   ESwSpeedChangeLimitExceeded = 0x8,
   ESwMotorStalled = 0x10,
   ESwNoZeroCrossing = 0x20,
   ESwDesiredSpeedLimitExceeded = 0x40,
   ESwInvalidState = 0x80
} EBldcStatusWord;

#endif /* CANDEFS_H_ */
