/*
 * candefs.h
 *
 * This file is included from multiple projects.
 * Don't include any header files!
 *
 *  Created on: Oct 4, 2013
 *      Author: kai
 */

#ifndef CANDEFS_H_
#define CANDEFS_H_

/*

 CAN protocol description
 ========================

 Usage CAN Ext. ID (29 Bit)

 Bit   2   2 2 2 2  |  2 2 2 2   1 1 1 1   |   1 1 1 1   1 1      |                      |
       8 | 7 6 5 4  |  3 2 1 0 | 9 8 7 6   |   5 4 3 2 | 1 0 9 8  |  7 6 5 4 | 3 2 1 0   |
 -----------------------------------------------------------------------------------------
       1 | Dst Unt  |  Cmd     |           |   SeqNo              |  Src Unt | Src Adr   |

  0.. 3  Src Adr     Unique sender address
  4.. 7  Src Unt     Sender unit / Baugruppe (0 BLDC, 1 FLight Controller, ...)
  8..15  SeqNo       Content sequence no (destination unit / command specific)
 16..23  Cmd         Command
 24..27  Dst Unt     Destination unit (0 BLDC, 1 FLight Controller, ...)
 28      Reserved    Shall be 1 (1 normal data, 0 high prio - tbd)


 Baugruppe 0 (BLDC)
 ------------------

 Bit |  2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1     |                   |
     |  8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
 --------------------------------------------------------------------------------
        1 | Dst Unt | Cmd               | SeqNo             | Src Unt | Src Adr |
        1 | 0 0 0 0 | x x x x | y y y y | Dst Grp | Dst Vld | Src Unt | Src Id  |

  8..11  Dst Vld     Bit marks if payload at index i is valid.
                     E.g. for Speed cmd: Bit 0 Data[0..1]-payload valid, ..., Bit 3 Data[6..7]-payload valid.
                     E.g. for Motor cmd: Bit 0 Data[0]-payload valid, ..., Bit 3 Data[3]-payload valid.

 12..15  Dst Grp     Payload sequence no. This 4 bits identifies a sequence of payload (4 values).
                     E.g. for Speed cmd: Group0: Speed 1-4, Group1: Speed 5-8, ..., Group15 Speed 60-64

 16..23  Cmd         20..23 Command:
                            0 Reserved
                            1 Motor control (Motor on / off)
                            2 Set speed
                     16..19 Sub Command
                            2.1 Unscaled velocity as PWM counter value
                            2.2 Scaled velocity (0..0xFFFF)

 Baugruppe 1 (Flight Controller)
 -------------------------------

 Bit |  2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1     |                   |
     |  8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
 --------------------------------------------------------------------------------
        1 | Dst Unt | Cmd               | SeqNo             | Src Unt | Src Adr |
        1 | 0 0 0 1 | x x x x | y y y y | 1 1 1 1   1 1 1 1 | Src Unt | Src Id  |

  8..15  Dst Addr    Broadcast to all Flight Controllers
 16..23  Cmd         20..23 Command:
                            0 Reserved
                            1 Alarm
                            2 Sensor data
                     16..19 Sub Command
                            1.0 tbd
                            2.1 Send Current, Voltage, RPM
*/

#define CAN_SPEED_500k 0
#define CAN_SPEED_1000k 1

/*
 * Access masks
 */

#define CAN_SRCADDR_MASK 0xF
#define CAN_SRCADDR_SHIFT 0

#define CAN_SRCUNIT_MASK 0xF0
#define CAN_SRCUNIT_SHIFT 4

#define CAN_CMD_MASK 0xFF0000
#define CAN_CMD_SHIFT 16

#define CAN_SEQ_MASK 0xFF00
#define CAN_SEQ_SHIFT 8

#define CAN_DSTUNIT_MASK 0xF000000
#define CAN_DSTUNIT_SHIFT 24

#define CAN_RSVD_MASK (1 << 28)

/*
 * Baugruppen ID's
 */

#define CAN_UNIT_BLDC 0
#define CAN_UNIT_FCTRL 1

/*
 * Baugruppe BLDC
 */

#define CAN_BLDC_CMD_MOTORCTRL 0x10 // Motor On, Off

#define CAN_BLDC_CMD_SPEED 0x20 // Speed control
#define CAN_BLDC_SUBCMD_SPEEDCNT 0x1 // Unscaled speed as PWM counter value
#define CAN_BLDC_SUBCMD_SPEED16 0x2 // Scaled speed in the range from 0 (Min) to 0xFFFF (Max)
#define CAN_BLDC_SUBCMD_SPEEDOP 0x3 // OpenPilot speed in the range from 0 (Min) to 9999 (Max)

// Length in Bytes
#define CAN_BLDC_SPEEDMSG_LEN 8 // 16 Bit * 4 BLDC's
#define CAN_BLDC_MOTORCTRL_LEN 4 // (8 Bit * 4 BLDC's)

// CAN_ID_MOTORCTRL
#define CAN_BLDC_MOTOR_OFF 0
#define CAN_BLDC_MOTOR_ON 1

#endif /* CANDEFS_H_ */
