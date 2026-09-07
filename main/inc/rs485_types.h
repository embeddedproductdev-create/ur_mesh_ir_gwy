#ifndef RS485_TYPES_H
#define RS485_TYPES_H
 
/**
 * ===================================================================
 * Gateway-side mirror of the RS485 node's common_types.h
 * ===================================================================
 * This is NOT a verbatim #include of the node's common_types.h — that
 * file declares its own `mqtt_packets` and `error_codes` enums, both of
 * which collide by name (and by several identical enumerator names —
 * SUCCESS, FAILURE, MISSING_PACKET_ID, ...) with this gateway's own
 * lte.h. Only the structs actually needed to relay RS485 traffic are
 * reproduced here.
 *
 * KEEP THIS FILE'S STRUCTS BYTE-IDENTICAL TO THE NODE'S. These are sent
 * as raw bytes over the BLE Mesh vendor model (no serialization), so any
 * field-order/size mismatch between this file and the node's
 * common_types.h silently corrupts every message.
 *
 * Two deliberate departures from a literal transcription, both to
 * remove ambiguity in the on-wire struct size:
 *   - `register_entry_t.data_type` is `uint8_t` here (holds a
 *     reg_data_type_t value) instead of the enum type itself.
 *   - Every `errorcode` field is `int8_t` here (holds an error_codes
 *     value) instead of the enum type itself.
 *   A plain C enum's size depends on whether the compiler is invoked
 *   with -fshort-enums, which is not something this gateway project can
 *   see or control on the node's build. Fixed-width types make the
 *   struct size independent of that flag. This assumes the equivalent
 *   change is applied to the node's common_types.h — see the diff
 *   provided alongside this file. If the node instead keeps the plain
 *   enum types, these struct sizes will NOT match and BLE Mesh messages
 *   will be misread; cheapest way to verify after building both sides:
 *   log sizeof(RS485CmdStruct), sizeof(MeterConfigCmd) and
 *   sizeof(ModbusReadingPkt) once on each side and compare.
 *
 * MAX_PARAMS_PER_DEVICE=30 and DEVICE_LABEL_LEN=20 here match the
 * updated common_types.h (bumped from 20 / 12) — keep these three
 * values in lockstep across both projects.
 * ===================================================================
 */
 
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
 
/*===========================================================
 * Constants shared with the node's common_types.h
 *===========================================================*/
#define RS485_DEVICE_LABEL_LEN       20   /* must match node's DEVICE_LABEL_LEN */
#define RS485_MAX_PARAMS_PER_DEVICE  30   /* must match node's MAX_PARAMS_PER_DEVICE */
#define RS485_SERIAL_NO_LEN          16   /* must match node's SERIAL_NO_LEN (== gateway's SERIAL_NO_LEN) */
#define RS485_GPIO_CHANNEL_COUNT     8
 
#define NODE_TYPE_IR_AC   0x01
#define NODE_TYPE_RS485   0x02
 
/*===========================================================
 * Register data type — how the node decodes raw register bytes.
 * Values only — stored on the wire as plain uint8_t (see note above).
 *===========================================================*/
typedef enum
{
    REG_TYPE_FLOAT        = 0,  /* 2 regs, IEEE 754 big-endian (ABCD) */
    REG_TYPE_LONG_INVERSE = 1,  /* 2 regs, CDAB word order — Elmeasure run hours */
    REG_TYPE_UINT16       = 2,  /* 1 reg,  16-bit unsigned integer */
    REG_TYPE_UINT32       = 3,  /* 2 regs, big-endian uint32 */
} reg_data_type_t;
 
/*===========================================================
 * Meter config action — replaces the old boolean `remove` flag.
 *===========================================================*/
typedef enum
{
    METER_ACTION_ADD    = 0,
    METER_ACTION_UPDATE = 1,
    METER_ACTION_REMOVE = 2,
} meter_config_action_t;
 
/*===========================================================
 * Register entry — one entry per parameter in a device's register map
 *===========================================================*/
typedef struct
{
    uint8_t  param_idx;    /* 0-based index — cloud maps to param name */
    uint8_t  available;    /* 1=read this register, 0=skip */
    uint16_t modbus_addr;  /* datasheet register address e.g. 40159 */
    uint8_t  data_type;    /* reg_data_type_t value, fixed 1-byte width */
} register_entry_t;
 
/*===========================================================
 * Generic RS485 node ACK / status struct
 * Node uses this for: NODE_CONF_ACK, NODE_UNPROV_PACKET ack,
 * NODE_HEARTBEAT_ACK, NODE_DEBUG_INFO_PACKET ack, NODE_PROV_PACKET ack,
 * and as the generic ACK for NODE_METER_CONFIG_PACKET (151),
 * NODE_GPIO_CONFIG_PACKET (154), NODE_GPIO_SET_PACKET (155) and
 * NODE_SUMAN_AC_CONTROL_PACKET (158) — same packetid echoed back,
 * errorcode carries the result.
 *===========================================================*/
typedef struct
{
    uint16_t packetid;             /* offset 0  — MUST be first */
    uint16_t msgseqno;              /* offset 2  */
    uint16_t elemaddr;               /* offset 4  */
    uint8_t  node_type;               /* offset 6  */
    uint16_t publishPeriodSec;         /* offset 7  */
    int8_t   errorcode;                /* offset 9  — error_codes value, fixed 1-byte width */
    uint8_t  resetDevice;              /* offset 13 */
    uint8_t  restartDevice;            /* offset 14 */
    uint8_t  majversion;               /* offset 15 */
    uint8_t  minversion;               /* offset 16 */
    uint8_t  patchversion;             /* offset 17 */
    uint8_t  provisioned;              /* offset 18 */
    bool     configured;               /* offset 19 */
    char     deviceName[RS485_SERIAL_NO_LEN]; /* offset 20 */
    float    deviceUpTimeHrs;          /* offset 36 */
    esp_err_t bleErrorCode;            /* offset 40 */
    int8_t   rssi;                     /* offset 44 */
} RS485CmdStruct;
 
/*===========================================================
 * Cloud -> Gateway -> Node: energy-meter register-map config
 * NODE_METER_CONFIG_PACKET = 151
 *===========================================================*/
typedef struct
{
    uint16_t         packetid;      /* NODE_METER_CONFIG_PACKET = 151 */
    uint16_t         msgseqno;
    uint16_t         elemaddr;
    uint8_t          node_type;     /* NODE_TYPE_RS485 */
    uint8_t          slave_addr;
    uint8_t          action;        /* meter_config_action_t value, fixed 1-byte width */
    char             label[RS485_DEVICE_LABEL_LEN];
    uint8_t          reg_count;
    register_entry_t regs[RS485_MAX_PARAMS_PER_DEVICE];
    char             deviceName[RS485_SERIAL_NO_LEN];
} MeterConfigCmd;
 
/*===========================================================
 * Node -> Gateway -> Cloud: one raw meter reading
 * Node reads raw bytes and sends them as-is — cloud decodes using the
 * data_type it originally sent in MeterConfigCmd for this param_idx.
 * Gateway does not decode these — it relays param_idx/modbus_addr/
 * available/raw through to MQTT unchanged.
 *===========================================================*/
typedef struct
{
    uint8_t  param_idx;
    uint8_t  available;
    uint16_t modbus_addr;
    uint32_t raw;
} meter_reading_entry_t;
 
/*===========================================================
 * Node -> Gateway -> Cloud: meter reading packet
 * NODE_METER_READING_PACKET (152) = 3-phase, NODE_METER_READING_1P_PACKET
 * (153) = single-phase — same struct, different packetid.
 *===========================================================*/
typedef struct
{
    uint16_t              packetid;
    uint16_t              msgseqno;
    uint16_t              elemaddr;
    uint8_t               node_type;
    uint8_t               slave_addr;
    char                  label[RS485_DEVICE_LABEL_LEN];
    uint8_t               reading_count;
    meter_reading_entry_t readings[RS485_MAX_PARAMS_PER_DEVICE];
    int8_t                errorcode;   /* error_codes value, fixed 1-byte width */
    char                  deviceName[RS485_SERIAL_NO_LEN];
} ModbusReadingPkt;
 
/*===========================================================
 * Cloud -> Gateway -> Node: GPIO channel config
 * NODE_GPIO_CONFIG_PACKET = 154
 *===========================================================*/
typedef struct
{
    uint16_t packetid;
    uint16_t msgseqno;
    uint16_t elemaddr;
    uint8_t  node_type;
    uint8_t  channel_mask;
    uint8_t  direction;
    uint8_t  pull_up_mask;    /* bit i = enable internal pull-up on channel i   */
    uint8_t  pull_down_mask;  /* bit i = enable internal pull-down on channel i */
    char     deviceName[RS485_SERIAL_NO_LEN];
} GpioConfigCmd;
 
/*===========================================================
 * Cloud -> Gateway -> Node: GPIO channel set
 * NODE_GPIO_SET_PACKET = 155
 *===========================================================*/
typedef struct
{
    uint16_t packetid;
    uint16_t msgseqno;
    uint16_t elemaddr;
    uint8_t  node_type;
    uint8_t  channel_mask;
    uint8_t  level;
    char     deviceName[RS485_SERIAL_NO_LEN];
} GpioSetCmd;
 
/*===========================================================
 * Node -> Gateway -> Cloud: GPIO status
 * NODE_GPIO_STATUS_PACKET = 156
 *===========================================================*/
typedef struct
{
    uint16_t packetid;
    uint16_t msgseqno;
    uint16_t elemaddr;
    uint8_t  node_type;
    uint8_t  channel_states;
    uint8_t  channel_directions;
    int8_t   errorcode;          /* error_codes value, fixed 1-byte width */
    char     deviceName[RS485_SERIAL_NO_LEN];
} GpioStatusPkt;
 
/*===========================================================
 * Node -> Gateway -> Cloud: Suman Electronics AC controller status
 * NODE_SUMAN_AC_STATUS_PACKET = 157
 *===========================================================*/
typedef struct
{
    uint16_t packetid;
    uint16_t msgseqno;
    uint16_t elemaddr;
    uint8_t  node_type;
    uint8_t  slave_addr;
    char     label[RS485_DEVICE_LABEL_LEN];
    bool     is_event_driven;
 
    uint16_t pc_set_temp;
    uint16_t pc_operating_mode;
    uint16_t control_option;
    uint16_t handset_ctrl_time;
 
    uint16_t num_compressors;
    uint16_t model_type;
 
    uint16_t ctrl_set_temp;
    uint16_t ctrl_operating_mode;
    uint16_t fault_reset_active;
    uint16_t return_air_temp;
    uint16_t comp1_run_hours;
    uint16_t comp2_run_hours;
    uint16_t fan_run_hours;
    uint16_t compressor_status;
    uint16_t fan_status;
    uint16_t alarm_status;
    uint16_t system_fault;
    uint16_t sensor_fault;
    uint16_t comp1_fault;
    uint16_t comp2_fault;
    uint16_t supply_air_temp;
    uint16_t humidity1;
    uint16_t humidity2;
    uint16_t comp1_current;
    uint16_t comp2_current;
 
    int8_t   errorcode;          /* error_codes value, fixed 1-byte width */
    char     deviceName[RS485_SERIAL_NO_LEN];
} SumanAcStatusPkt;
 
/*===========================================================
 * Cloud -> Gateway -> Node: Suman Electronics AC controller control
 * NODE_SUMAN_AC_CONTROL_PACKET = 158
 *===========================================================*/
typedef struct
{
    uint16_t packetid;
    uint16_t msgseqno;
    uint16_t elemaddr;
    uint8_t  node_type;
    uint8_t  slave_addr;
    uint8_t  write_mask;         /* bit0=set_temp bit1=operating_mode bit2=reset_fault
                                    bit3=control_option bit4=handset_ctrl_time */
    uint16_t set_temp;
    uint16_t operating_mode;
    uint16_t reset_fault;
    uint16_t control_option;
    uint16_t handset_ctrl_time;
    char     deviceName[RS485_SERIAL_NO_LEN];
} SumanAcControlCmd;
 
#endif /* RS485_TYPES_H */
 

