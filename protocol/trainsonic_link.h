/**
 ******************************************************************************
 * @file    trainsonic_link.h
 * @brief   SINGLE SOURCE OF TRUTH for the ESP32 <-> STM32 UART link.
 *
 *          This header defines every message on the wire: IDs, packed
 *          payload structs, scaling, and the protocol version. It is meant to
 *          be shared VERBATIM by BOTH firmwares (git submodule), so the two
 *          sides can never drift. Do not fork or hand-copy it.
 *
 *          Transport: the MIN protocol (https://github.com/min-protocol/min).
 *            - MIN gives framing + CRC + optional reliable transport.
 *            - MIN message IDs are 6-bit (0..63); keep all IDs in that range.
 *            - "Reliable" msgs are sent as MIN TRANSPORT frames (seq/ACK/retx).
 *            - "Stream" msgs are sent as MIN NON-transport frames (newest-wins,
 *              never retransmitted -> no stale-setpoint latency on a motor).
 *
 *          Safety: the STM32 MUST ramp torque to 0 / enter safe state if a
 *          valid HEARTBEAT/command is not seen within TSL_LINK_TIMEOUT_MS.
 ******************************************************************************
 */
#ifndef TRAINSONIC_LINK_H
#define TRAINSONIC_LINK_H

#include <stdint.h>

/* Bump on ANY incompatible change (add/remove/resize a message or field).
   Exchanged in TSL_MSG_HELLO; a mismatch must be treated as a fatal link fault. */
#define TSL_PROTOCOL_VERSION      4u   /* v4: heartbeat carries brake_mA (speed-window control); v3: TSL_MSG_CALIBRATE */

/* Link watchdog: STM32 enters safe state if no valid ESP32 frame arrives within this. */
#define TSL_LINK_TIMEOUT_MS       200u

/* Reliable (R) messages: the on-wire frame is [id][seq][len][payload][crc16-LE].
   seq==0 => fire-and-forget (stream, never ACKed). seq!=0 (1..255, wrapping) => the
   receiver replies with TSL_MSG_ACK carrying that seq; the sender retransmits until
   ACKed. Stop-and-wait: one reliable frame outstanding at a time. */
#define TSL_RELIABLE_TIMEOUT_MS   150u  /* retransmit if no ACK within this (> the slower side's tick) */
#define TSL_RELIABLE_RETRIES      5u    /* give up (link error) after this many resends */
/* Recommended cadences (informative). */
#define TSL_HEARTBEAT_PERIOD_MS   7u     /* ESP32 -> STM32 keep-alive + setpoint (~143 Hz) */
#define TSL_STATUS_PERIOD_MS      7u     /* STM32 -> ESP32 fast telemetry (~143 Hz) */

/* ------------------------------------------------------------------------- */
/* Message IDs (0..63). R = reliable/transport frame, S = stream/fire-forget. */
/* ------------------------------------------------------------------------- */
typedef enum
{
  /* --- handshake / link mgmt --- */
  TSL_MSG_HELLO        = 0x00,  /* R  both ways: version + fw id (see tsl_hello_t) */
  TSL_MSG_HEARTBEAT    = 0x01,  /* S  ESP32->STM32: keep-alive + live setpoint     */
  TSL_MSG_ACK          = 0x02,  /* S  acknowledges a reliable frame's seq (tsl_ack_t) */

  /* --- commands: ESP32 -> STM32 --- */
  TSL_MSG_SET_MODE     = 0x10,  /* R  set control mode (tsl_set_mode_t)            */
  TSL_MSG_SETPOINT     = 0x11,  /* S  live torque/speed setpoint (tsl_setpoint_t)  */
  TSL_MSG_ENABLE       = 0x12,  /* R  enable/disable the drive (tsl_enable_t)      */
  TSL_MSG_ESTOP        = 0x13,  /* R  immediate safe stop (no payload)             */
  TSL_MSG_ACK_FAULT    = 0x14,  /* R  clear latched faults + restart at 0 A        */
  TSL_MSG_CONFIG_SET   = 0x15,  /* R  set a tunable by key (tsl_config_kv_t)       */
  TSL_MSG_CALIBRATE    = 0x16,  /* R  anti-cogging calibration control (tsl_calibrate_t) */

  /* --- telemetry / events: STM32 -> ESP32 --- */
  TSL_MSG_STATUS       = 0x40,  /* S  fast periodic status (tsl_status_t)          */
  TSL_MSG_FAULT_EVENT  = 0x41,  /* R  a fault occurred (tsl_fault_event_t)         */
  TSL_MSG_CONFIG_VAL   = 0x42,  /* R  reply to a config get/set (tsl_config_kv_t)  */
} tsl_msg_id_t;

/* ------------------------------------------------------------------------- */
/* Enums                                                                     */
/* ------------------------------------------------------------------------- */
typedef enum { TSL_MODE_IDLE = 0, TSL_MODE_TORQUE = 1, TSL_MODE_SPEED = 2 } tsl_mode_t;

typedef enum { TSL_STATE_IDLE=0, TSL_STATE_ALIGN=1, TSL_STATE_RUN=2,
               TSL_STATE_FAULT=3, TSL_STATE_STOP=4 } tsl_state_t;

/* TSL_MSG_CALIBRATE action (anti-cogging). START needs the motor in RUN; SAVE writes
   the map to flash and the STM32 REBOOTS (link drops ~1-2 s, then reconnects). */
typedef enum { TSL_CAL_START=0, TSL_CAL_ABORT=1, TSL_CAL_SAVE=2, TSL_CAL_ERASE=3 } tsl_cal_action_t;

/* fault_flags bitfield (tsl_status_t.fault_flags / tsl_fault_event_t.code) */
#define TSL_FAULT_NONE         0x0000u
#define TSL_FAULT_OVERCURRENT  0x0001u
#define TSL_FAULT_OVERVOLTAGE  0x0002u
#define TSL_FAULT_UNDERVOLTAGE 0x0004u
#define TSL_FAULT_OVERTEMP     0x0008u
#define TSL_FAULT_ENCODER      0x0010u
#define TSL_FAULT_GATE_DRIVER  0x0020u
#define TSL_FAULT_LINK_LOST    0x0040u   /* set by STM32 when the ESP32 link times out */

/* tsl_status_t.flags bits */
#define TSL_SFLAG_COGGING_ON   0x01u     /* anti-cogging FF active            */
#define TSL_SFLAG_FW_ON        0x02u     /* flux weakening active             */
#define TSL_SFLAG_CALIBRATING  0x04u     /* anti-cogging cal in progress (probe or sweep) */
#define TSL_SFLAG_READY        0x08u     /* boot + encoder alignment complete; latched once ready for commands */
#define TSL_SFLAG_AT_LIMIT     0x10u     /* speed-window: speed PI is on a torque limit (drive side or brake side) */

/* ------------------------------------------------------------------------- */
/* Payload structs. Little-endian, explicitly packed. Fixed scaling in comments. */
/* ------------------------------------------------------------------------- */
#pragma pack(push, 1)

typedef struct {                    /* TSL_MSG_HELLO */
  uint16_t protocol_version;        /* = TSL_PROTOCOL_VERSION            */
  uint32_t fw_version;              /* sender firmware id (git hash low / semver) */
} tsl_hello_t;

typedef struct {                    /* TSL_MSG_ACK */
  uint8_t  acked_seq;               /* frame seq being acknowledged (1..255) */
} tsl_ack_t;

typedef struct {                    /* TSL_MSG_HEARTBEAT (ESP32->STM32) */
  uint32_t seq;                     /* increments each heartbeat         */
  int16_t  torque_mA;              /* DRIVE-side torque limit, milliamps, SIGNED. The sign is the
                                      direction of the speed reference: <0 = inward (retract, the
                                      weight), >0 = outward (push the cable out, e.g. home wall).
                                      0 = no drive torque (slack), reference stays inward.
                                      Legacy torque-mode builds apply it as a plain torque setpoint. */
  int16_t  speed_rpm;              /* speed reference MAGNITUDE, rpm. 0 or above the STM32 hard
                                      ceiling -> the hard ceiling. In speed-window control the drive
                                      holds |torque_mA| until the drum reaches this speed in the
                                      reference direction, then eases off and holds the speed.
                                      (Legacy torque mode: the governor cap.) */
  int16_t  brake_mA;               /* v4: BRAKE-side torque limit, milliamps, >= 0: the most torque
                                      the drive may apply AGAINST motion faster than the reference
                                      (free-fall catch). 0 = never brake, only ease the drive to 0.
                                      A v3 (8-byte) heartbeat is still accepted: the STM32 then uses
                                      its console default ('%<mA>', 2000). */
} tsl_heartbeat_t;

typedef struct {                    /* TSL_MSG_SET_MODE */
  uint8_t  mode;                    /* tsl_mode_t                        */
} tsl_set_mode_t;

typedef struct {                    /* TSL_MSG_SETPOINT (stream) */
  int16_t  torque_mA;              /* torque setpoint, mA (signed)      */
  int16_t  speed_rpm;              /* speed setpoint, rpm (signed)      */
} tsl_setpoint_t;

typedef struct {                    /* TSL_MSG_ENABLE */
  uint8_t  enable;                  /* 0 = disable/coast, 1 = enable     */
} tsl_enable_t;

typedef struct {                    /* TSL_MSG_CALIBRATE */
  uint8_t  action;                  /* tsl_cal_action_t                  */
} tsl_calibrate_t;

typedef struct {                    /* TSL_MSG_STATUS (STM32->ESP32, fast) */
  int16_t  speed_rpm;              /* measured mechanical speed, rpm    */
  int16_t  torque_mA;              /* measured Iq, mA (signed)          */
  int16_t  id_mA;                  /* measured Id, mA (signed)          */
  uint16_t vbus_mV;                /* DC bus voltage, millivolts        */
  int16_t  temp_c_x10;            /* motor/board temp, deg C x10       */
  uint16_t fault_flags;            /* TSL_FAULT_* bitfield              */
  uint8_t  state;                  /* tsl_state_t                       */
  uint8_t  flags;                  /* bit0=cogging_on, bit1=fw_on, ...  */
  int32_t  position;               /* absolute multi-turn encoder count (16384 counts/motor-rev,
                                      0 at boot, monotonic with shaft travel; sign per build) */
} tsl_status_t;

typedef struct {                    /* TSL_MSG_FAULT_EVENT (reliable) */
  uint16_t code;                    /* TSL_FAULT_* bitfield              */
  uint8_t  severity;                /* 0=warn 1=throttle 2=disconnect    */
  uint32_t timestamp_ms;            /* STM32 uptime at fault             */
} tsl_fault_event_t;

typedef struct {                    /* TSL_MSG_CONFIG_SET / _VAL */
  uint16_t key;                     /* tunable id (define a tsl_cfg_key_t enum) */
  int32_t  value;                   /* scaled per-key                    */
} tsl_config_kv_t;

#pragma pack(pop)

/* Compile-time size locks so a struct can't silently change size across a bump.
   (Use _Static_assert in C11; both ESP-IDF and STM32 gcc support it.) */
#ifdef __STDC_VERSION__
#if __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(tsl_hello_t)       == 6,  "tsl_hello_t size");
_Static_assert(sizeof(tsl_ack_t)         == 1,  "tsl_ack_t size");
_Static_assert(sizeof(tsl_heartbeat_t)   == 10, "tsl_heartbeat_t size");
_Static_assert(sizeof(tsl_setpoint_t)    == 4,  "tsl_setpoint_t size");
_Static_assert(sizeof(tsl_calibrate_t)   == 1,  "tsl_calibrate_t size");
_Static_assert(sizeof(tsl_status_t)      == 18, "tsl_status_t size");
_Static_assert(sizeof(tsl_fault_event_t) == 7,  "tsl_fault_event_t size");
_Static_assert(sizeof(tsl_config_kv_t)   == 6,  "tsl_config_kv_t size");
#endif
#endif

#endif /* TRAINSONIC_LINK_H */
