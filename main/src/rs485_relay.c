/**
 * ===================================================================
 * RS485 relay — gateway side
 * ===================================================================
 * The gateway has no RS485/Modbus peripheral of its own. For the
 * packet IDs below it is a pure relay between the cloud (MQTT) and an
 * RS485-capable BLE Mesh node (NODE_TYPE_RS485):
 *
 *   Cloud -> Gateway -> Node   (validated here, then forwarded as-is)
 *     NODE_METER_CONFIG_PACKET     (151)  MeterConfigCmd
 *     NODE_GPIO_CONFIG_PACKET      (154)  GpioConfigCmd
 *     NODE_GPIO_SET_PACKET         (155)  GpioSetCmd
 *     NODE_SUMAN_AC_CONTROL_PACKET (158)  SumanAcControlCmd
 *
 *   Node -> Gateway -> Cloud  (published as-is, gateway does not decode)
 *     NODE_METER_READING_PACKET    (152)  ModbusReadingPkt (3-phase)
 *     NODE_METER_READING_1P_PACKET (153)  ModbusReadingPkt (1-phase)
 *     NODE_GPIO_STATUS_PACKET      (156)  GpioStatusPkt
 *     NODE_SUMAN_AC_STATUS_PACKET  (157)  SumanAcStatusPkt
 *
 *   Node -> Gateway -> Cloud  (ACK for one of the four command types above)
 *     RS485CmdStruct, packetid = whichever command it's acking
 *
 * The gateway never stores any of this in NVS — every command is
 * validated, relayed, and forgotten; every report/ACK from the node is
 * republished to MQTT and forgotten.
 *
 * Two error-code spaces meet in this file and must not be confused:
 * gateway-side validation failures use THIS project's error_codes enum
 * (lte.h); node-side rejections carried back in a struct's `errorcode`
 * field use the RS485 node's OWN error_codes enum (common_types.h) —
 * a different, unrelated numbering. Every ACK this file builds includes
 * "ErrorSource": "GATEWAY" or "NODE" so the cloud knows which table to
 * read the numeric ErrorCode against.
 * ===================================================================
 */
#include <main.h>
 
#if (IS_GWY)
 
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include <cJSON.h>
#include <json_maker.h>
#include <lte.h>
#include <ble_new.h>
#include "rs485_types.h"
#include "rs485_relay.h"
 
#define RS485_TAG "RS485_RELAY"
 
/* Sized for a 30-entry Readings/RegisterMap array with pretty-printing
   headroom — see the size note in rs485_types.h. MQTT_ACK_BUFFER_LEN
   (1024, from lte.h) stays fine for the small generic ACK. */
#define RS485_JSON_BUFFER_LEN 4096
 
/*===========================================================
 * Small validation helpers — mirror the style of error_check_json()
 * in lte.c, but write into gateway-local out-params instead of the
 * AC-control CommandStruct, which has no room for these fields.
 *===========================================================*/
 
/** Required integer field, must satisfy min <= value <= max. */
static error_codes get_required_int(cJSON *json_obj, const char *key, int min, int max,
                                     error_codes missing_err, error_codes range_err,
                                     int *out)
{
    cJSON *item = cJSON_GetObjectItem(json_obj, key);
    if (item == NULL)
        return missing_err;
    if (!cJSON_IsNumber(item))
        return range_err;
    int v = item->valueint;
    if (v < min || v > max)
        return range_err;
    *out = v;
    return SUCCESS;
}
 
/** Optional integer field — if absent, *out keeps its caller-supplied default. */
static error_codes get_optional_int(cJSON *json_obj, const char *key, int min, int max,
                                     error_codes range_err, int *out)
{
    cJSON *item = cJSON_GetObjectItem(json_obj, key);
    if (item == NULL)
        return SUCCESS;
    if (!cJSON_IsNumber(item))
        return range_err;
    int v = item->valueint;
    if (v < min || v > max)
        return range_err;
    *out = v;
    return SUCCESS;
}
 
/** Required string field, copied into a fixed-size buffer (truncation is rejected, not silent). */
static error_codes get_required_str(cJSON *json_obj, const char *key, char *out, size_t out_len,
                                     error_codes missing_err, error_codes toolong_err)
{
    cJSON *item = cJSON_GetObjectItem(json_obj, key);
    if (item == NULL || !cJSON_IsString(item) || item->valuestring == NULL)
        return missing_err;
    if (strlen(item->valuestring) > out_len - 1)
        return toolong_err;
    strncpy(out, item->valuestring, out_len - 1);
    out[out_len - 1] = '\0';
    return SUCCESS;
}
 
/** Optional string field. */
static void get_optional_str(cJSON *json_obj, const char *key, char *out, size_t out_len)
{
    cJSON *item = cJSON_GetObjectItem(json_obj, key);
    if (item != NULL && cJSON_IsString(item) && item->valuestring != NULL)
    {
        strncpy(out, item->valuestring, out_len - 1);
        out[out_len - 1] = '\0';
    }
}
 
/*===========================================================
 * Gateway-side error ACK (validation failed before relay to node)
 *===========================================================*/
static void send_gateway_error_ack(mqtt_packets packetid, uint16_t msgseqno, error_codes err)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__);
        return;
    }
    ESP_LOGW(RS485_TAG, "Gateway rejected packetid=%d msgseqno=%d err=%d (%s)",
        packetid, msgseqno, err, get_error_code_name(err));
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", packetid);
    jwObj_int(&jwc, "MsgSeqNo", msgseqno);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "ErrorSource", "GATEWAY");
    jwObj_int(&jwc, "ErrorCode", err);
    jwObj_string(&jwc, "ErrorMsg", get_error_code_name(err));
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
/** Maps a command packetid to its dedicated ack packetid (160/161/162/163). */
static uint16_t ack_id_for_command(mqtt_packets cmd_packetid)
{
    switch (cmd_packetid)
    {
        case NODE_METER_CONFIG_PACKET:      return NODE_METER_CONFIG_ACK;
        case NODE_GPIO_CONFIG_PACKET:       return NODE_GPIO_CONFIG_ACK;
        case NODE_GPIO_SET_PACKET:          return NODE_GPIO_SET_ACK;
        case NODE_SUMAN_AC_CONTROL_PACKET:  return NODE_SUMAN_AC_CONTROL_ACK;
        default:                            return cmd_packetid;
    }
}
 
/** meter_config_action_t -> the string the MQTT API uses. */
static const char *meter_action_name(uint8_t action)
{
    switch (action)
    {
        case METER_ACTION_ADD:    return "ADD";
        case METER_ACTION_UPDATE: return "UPDATE";
        case METER_ACTION_REMOVE: return "REMOVE";
        default:                  return "ADD";
    }
}
 
/*===========================================================
 * In-flight command tracker — msgseqno-keyed
 * The node's ACK (RS485CmdStruct) only carries back packetid/msgseqno/
 * elemaddr/errorcode/rssi/deviceName/node_type — none of the
 * command-specific fields (SlaveAddr, ChannelMask, ...). To build the
 * richer ACK JSON the cloud expects, and to detect a command that never
 * gets ACKed at all, keep a small malloc'd record of each in-flight
 * command from the moment it's relayed until the node's ACK arrives
 * (freed then) or BLE_NODE_COMM_TIMEOUT_MS elapses (freed then too,
 * with a NODE_COMM_TIMEOUT ACK sent to cloud instead). Not persisted
 * anywhere — a gateway reboot drops whatever was in flight, same as
 * command_queue. Mirrors GroupAckTracker_t in group_table.c: malloc'd
 * singly-linked list, no mutex — everything that touches it runs on the
 * maintainMQTTConnection() task, same as the group tracker.
 *===========================================================*/
typedef struct rs485_pending_cmd
{
    uint16_t   ack_packetid;  /* JsonPacketID to use once the ACK is built (160/161/162) */
    uint16_t   msgseqno;
    uint16_t   elemaddr;
    TickType_t start_tick;
    union
    {
        struct { uint8_t slave_addr; uint8_t action; char label[RS485_DEVICE_LABEL_LEN]; uint8_t reg_count; } meter_cfg;
        struct { uint8_t channel_mask; uint8_t direction; uint8_t pull_up_mask; uint8_t pull_down_mask; } gpio_cfg;
        struct { uint8_t channel_mask; uint8_t level; } gpio_set;
    } req;
    struct rs485_pending_cmd *next;
} rs485_pending_cmd_t;
 
static rs485_pending_cmd_t *pending_head = NULL;
 
static rs485_pending_cmd_t *pending_alloc(uint16_t ack_packetid, uint16_t msgseqno, uint16_t elemaddr)
{
    rs485_pending_cmd_t *p = (rs485_pending_cmd_t *)malloc(sizeof(rs485_pending_cmd_t));
    if (p == NULL)
    {
        ESP_LOGE(RS485_TAG, "Pending-cmd malloc failed — heap: %" PRIu32, esp_get_free_heap_size());
        return NULL;
    }
    memset(p, 0, sizeof(*p));
    p->ack_packetid = ack_packetid;
    p->msgseqno     = msgseqno;
    p->elemaddr     = elemaddr;
    p->start_tick   = xTaskGetTickCount();
    p->next         = pending_head;
    pending_head    = p;
    return p;
}
 
static rs485_pending_cmd_t *pending_find_by_seq(uint16_t msgseqno)
{
    rs485_pending_cmd_t *curr = pending_head;
    while (curr != NULL)
    {
        if (curr->msgseqno == msgseqno)
            return curr;
        curr = curr->next;
    }
    return NULL;
}
 
static void pending_free(rs485_pending_cmd_t *p)
{
    if (pending_head == p)
    {
        pending_head = p->next;
    }
    else
    {
        rs485_pending_cmd_t *curr = pending_head;
        while (curr != NULL && curr->next != p)
            curr = curr->next;
        if (curr != NULL)
            curr->next = p->next;
    }
    free(p);
}
 
/*===========================================================
 * Type-specific gateway-side error ACKs — same field shape as the
 * corresponding success/node ACK, just with ErrorSource:"GATEWAY",
 * NodeType/SlaveAddr/etc. echoed from whatever was parsed before
 * validation failed (memset-zeroed/empty for the rest), and
 * Rssi:-128 (no real BLE reading — nothing was ever sent).
 *===========================================================*/
static void send_meter_config_error_ack(uint16_t msgseqno, const MeterConfigCmd *cmd, error_codes err)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer) { ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__); return; }
    ESP_LOGW(RS485_TAG, "Gateway rejected NODE_METER_CONFIG_PACKET msgseqno=%d err=%d (%s)",
        msgseqno, err, get_error_code_name(err));
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", NODE_METER_CONFIG_ACK);
    jwObj_int(&jwc, "MsgSeqNo", msgseqno);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", (char *)cmd->deviceName);
    jwObj_int(&jwc, "ElementAddr", cmd->elemaddr);
    jwObj_int(&jwc, "NodeType", cmd->node_type);
    jwObj_int(&jwc, "SlaveAddr", cmd->slave_addr);
    jwObj_string(&jwc, "Action", (char *)meter_action_name(cmd->action));
    jwObj_string(&jwc, "Label", (char *)cmd->label);
    jwObj_int(&jwc, "RegCount", cmd->reg_count);
    jwObj_string(&jwc, "ErrorSource", "GATEWAY");
    jwObj_int(&jwc, "ErrorCode", err);
    jwObj_string(&jwc, "ErrorMsg", get_error_code_name(err));
    jwObj_int(&jwc, "Rssi", -128);
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
static void send_gpio_config_error_ack(uint16_t msgseqno, const GpioConfigCmd *cmd, error_codes err)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer) { ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__); return; }
    ESP_LOGW(RS485_TAG, "Gateway rejected NODE_GPIO_CONFIG_PACKET msgseqno=%d err=%d (%s)",
        msgseqno, err, get_error_code_name(err));
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", NODE_GPIO_CONFIG_ACK);
    jwObj_int(&jwc, "MsgSeqNo", msgseqno);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", (char *)cmd->deviceName);
    jwObj_int(&jwc, "ElementAddr", cmd->elemaddr);
    jwObj_int(&jwc, "NodeType", cmd->node_type);
    jwObj_int(&jwc, "ChannelMask", cmd->channel_mask);
    jwObj_int(&jwc, "Direction", cmd->direction);
    jwObj_int(&jwc, "PullUp", cmd->pull_up_mask);
    jwObj_int(&jwc, "PullDown", cmd->pull_down_mask);
    jwObj_string(&jwc, "ErrorSource", "GATEWAY");
    jwObj_int(&jwc, "ErrorCode", err);
    jwObj_string(&jwc, "ErrorMsg", get_error_code_name(err));
    jwObj_int(&jwc, "Rssi", -128);
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
static void send_gpio_set_error_ack(uint16_t msgseqno, const GpioSetCmd *cmd, error_codes err)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer) { ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__); return; }
    ESP_LOGW(RS485_TAG, "Gateway rejected NODE_GPIO_SET_PACKET msgseqno=%d err=%d (%s)",
        msgseqno, err, get_error_code_name(err));
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", NODE_GPIO_SET_ACK);
    jwObj_int(&jwc, "MsgSeqNo", msgseqno);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", (char *)cmd->deviceName);
    jwObj_int(&jwc, "ElementAddr", cmd->elemaddr);
    jwObj_int(&jwc, "NodeType", cmd->node_type);
    jwObj_int(&jwc, "ChannelMask", cmd->channel_mask);
    jwObj_int(&jwc, "Level", cmd->level);
    jwObj_string(&jwc, "ErrorSource", "GATEWAY");
    jwObj_int(&jwc, "ErrorCode", err);
    jwObj_string(&jwc, "ErrorMsg", get_error_code_name(err));
    jwObj_int(&jwc, "Rssi", -128);
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
/*===========================================================
 * NODE_METER_CONFIG_PACKET (151) — cloud -> gateway -> node
 *===========================================================*/
static void handle_meter_config(cJSON *json_obj, uint16_t msgseqno)
{
    MeterConfigCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.packetid = NODE_METER_CONFIG_PACKET;
    cmd.msgseqno = msgseqno;
 
    int elemaddr = 0, node_type = 0, slave_addr = 0, reg_count = 0;
    char action_str[8] = {0};
    error_codes err;
 
    if ((err = get_required_int(json_obj, "ElementAddr", 3, 65534,
            MISSING_ELEMENT_ADDR, ELEMENT_ADDR_EXCEEDING_RANGE, &elemaddr)) != SUCCESS)
        goto reject;
    cmd.elemaddr = elemaddr;
 
    if ((err = get_required_int(json_obj, "NodeType", 0, 255,
            MISSING_NODE_TYPE, NODE_TYPE_INVALID_FORMAT, &node_type)) != SUCCESS)
        goto reject;
    if (node_type != NODE_TYPE_RS485)
    {
        err = NODE_TYPE_INVALID_FORMAT;
        goto reject;
    }
    cmd.node_type = (uint8_t)node_type;
 
    if ((err = get_required_int(json_obj, "SlaveAddr", 1, 247,
            MISSING_SLAVE_ADDR, SLAVE_ADDR_EXCEEDING_RANGE, &slave_addr)) != SUCCESS)
        goto reject;
    cmd.slave_addr = (uint8_t)slave_addr;
 
    if ((err = get_required_str(json_obj, "Action", action_str, sizeof(action_str),
            MISSING_ACTION, ACTION_INVALID_FORMAT)) != SUCCESS)
        goto reject;
    if (strcmp(action_str, "ADD") == 0)
        cmd.action = METER_ACTION_ADD;
    else if (strcmp(action_str, "UPDATE") == 0)
        cmd.action = METER_ACTION_UPDATE;
    else if (strcmp(action_str, "REMOVE") == 0)
        cmd.action = METER_ACTION_REMOVE;
    else
    {
        err = ACTION_INVALID_FORMAT;
        goto reject;
    }
 
    get_optional_str(json_obj, "NodeSerNo", cmd.deviceName, sizeof(cmd.deviceName));
 
    if (cmd.action != METER_ACTION_REMOVE)
    {
        // Label/RegCount/RegisterMap are only meaningful when adding/updating a device
        if ((err = get_required_str(json_obj, "Label", cmd.label, sizeof(cmd.label),
                MISSING_LABEL, LABEL_EXCEEDING_RANGE)) != SUCCESS)
            goto reject;
 
        if ((err = get_required_int(json_obj, "RegCount", 1, RS485_MAX_PARAMS_PER_DEVICE,
                MISSING_REG_COUNT, REG_COUNT_EXCEEDING_RANGE, &reg_count)) != SUCCESS)
            goto reject;
        cmd.reg_count = (uint8_t)reg_count;
 
        cJSON *reg_map = cJSON_GetObjectItem(json_obj, "RegisterMap");
        if (reg_map == NULL || !cJSON_IsArray(reg_map))
        {
            err = MISSING_REGISTER_MAP;
            goto reject;
        }
        int array_len = cJSON_GetArraySize(reg_map);
        if (array_len < reg_count)
        {
            err = REGISTER_MAP_INVALID_FORMAT;
            goto reject;
        }
 
        for (int i = 0; i < reg_count; i++)
        {
            cJSON *entry = cJSON_GetArrayItem(reg_map, i);
            if (entry == NULL || !cJSON_IsObject(entry))
            {
                err = REGISTER_MAP_ENTRY_INVALID;
                goto reject;
            }
            cJSON *paramIdx   = cJSON_GetObjectItem(entry, "ParamIdx");
            cJSON *modbusAddr = cJSON_GetObjectItem(entry, "ModbusAddr");
            cJSON *dataType   = cJSON_GetObjectItem(entry, "DataType");
            cJSON *available  = cJSON_GetObjectItem(entry, "Available");
 
            if (paramIdx == NULL || !cJSON_IsNumber(paramIdx) ||
                paramIdx->valueint < 0 || paramIdx->valueint >= RS485_MAX_PARAMS_PER_DEVICE ||
                modbusAddr == NULL || !cJSON_IsNumber(modbusAddr) ||
                modbusAddr->valueint < 0 || modbusAddr->valueint > 65535 ||
                dataType == NULL || !cJSON_IsNumber(dataType) ||
                dataType->valueint < 0 || dataType->valueint > 3 ||
                available == NULL || !cJSON_IsNumber(available) ||
                (available->valueint != 0 && available->valueint != 1))
            {
                err = REGISTER_MAP_ENTRY_INVALID;
                goto reject;
            }
 
            cmd.regs[i].param_idx   = (uint8_t)paramIdx->valueint;
            cmd.regs[i].modbus_addr = (uint16_t)modbusAddr->valueint;
            cmd.regs[i].data_type   = (uint8_t)dataType->valueint;
            cmd.regs[i].available   = (uint8_t)available->valueint;
        }
    }
 
    ESP_LOGI(RS485_TAG, "Relaying meter config to node 0x%04x: slave=%d action=%s reg_count=%d",
        cmd.elemaddr, cmd.slave_addr, meter_action_name(cmd.action), cmd.reg_count);
    send_rs485_struct_to_node(cmd.elemaddr, &cmd, sizeof(cmd));
 
    rs485_pending_cmd_t *p = pending_alloc(NODE_METER_CONFIG_ACK, msgseqno, cmd.elemaddr);
    if (p != NULL)
    {
        p->req.meter_cfg.slave_addr = cmd.slave_addr;
        p->req.meter_cfg.action     = cmd.action;
        p->req.meter_cfg.reg_count  = cmd.reg_count;
        strncpy(p->req.meter_cfg.label, cmd.label, sizeof(p->req.meter_cfg.label) - 1);
    }
    return;
 
reject:
    send_meter_config_error_ack(msgseqno, &cmd, err);
}
 
/*===========================================================
 * NODE_GPIO_CONFIG_PACKET (154) — cloud -> gateway -> node
 * Gateway only checks JSON shape/byte-range here; per-channel semantic
 * validation (GPIO_CHANNEL_INDEX_INVALID etc.) is the node's job and
 * comes back in the async ACK's errorcode.
 *===========================================================*/
static void handle_gpio_config(cJSON *json_obj, uint16_t msgseqno)
{
    GpioConfigCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.packetid = NODE_GPIO_CONFIG_PACKET;
    cmd.msgseqno = msgseqno;
 
    int elemaddr = 0, node_type = 0, channel_mask = 0, direction = 0, pull_up = 0, pull_down = 0;
    error_codes err;
 
    if ((err = get_required_int(json_obj, "ElementAddr", 3, 65534,
            MISSING_ELEMENT_ADDR, ELEMENT_ADDR_EXCEEDING_RANGE, &elemaddr)) != SUCCESS)
        goto reject;
    cmd.elemaddr = elemaddr;
 
    if ((err = get_required_int(json_obj, "NodeType", 0, 255,
            MISSING_NODE_TYPE, NODE_TYPE_INVALID_FORMAT, &node_type)) != SUCCESS)
        goto reject;
    if (node_type != NODE_TYPE_RS485) { err = NODE_TYPE_INVALID_FORMAT; goto reject; }
    cmd.node_type = (uint8_t)node_type;
 
    if ((err = get_required_int(json_obj, "ChannelMask", 0, 255,
            MISSING_CHANNEL_MASK, CHANNEL_MASK_EXCEEDING_RANGE, &channel_mask)) != SUCCESS)
        goto reject;
    cmd.channel_mask = (uint8_t)channel_mask;
 
    if ((err = get_required_int(json_obj, "Direction", 0, 255,
            MISSING_DIRECTION, DIRECTION_INVALID_FORMAT, &direction)) != SUCCESS)
        goto reject;
    cmd.direction = (uint8_t)direction;
 
    if ((err = get_required_int(json_obj, "PullUp", 0, 255,
            MISSING_PULL, PULL_INVALID_FORMAT, &pull_up)) != SUCCESS)
        goto reject;
    cmd.pull_up_mask = (uint8_t)pull_up;
 
    if ((err = get_required_int(json_obj, "PullDown", 0, 255,
            MISSING_PULL, PULL_INVALID_FORMAT, &pull_down)) != SUCCESS)
        goto reject;
    cmd.pull_down_mask = (uint8_t)pull_down;
 
    get_optional_str(json_obj, "NodeSerNo", cmd.deviceName, sizeof(cmd.deviceName));
 
    ESP_LOGI(RS485_TAG, "Relaying GPIO config to node 0x%04x: mask=0x%02x dir=0x%02x pullup=0x%02x pulldown=0x%02x",
        cmd.elemaddr, cmd.channel_mask, cmd.direction, cmd.pull_up_mask, cmd.pull_down_mask);
    send_rs485_struct_to_node(cmd.elemaddr, &cmd, sizeof(cmd));
 
    rs485_pending_cmd_t *p = pending_alloc(NODE_GPIO_CONFIG_ACK, msgseqno, cmd.elemaddr);
    if (p != NULL)
    {
        p->req.gpio_cfg.channel_mask   = cmd.channel_mask;
        p->req.gpio_cfg.direction      = cmd.direction;
        p->req.gpio_cfg.pull_up_mask   = cmd.pull_up_mask;
        p->req.gpio_cfg.pull_down_mask = cmd.pull_down_mask;
    }
    return;
 
reject:
    send_gpio_config_error_ack(msgseqno, &cmd, err);
}
 
/*===========================================================
 * NODE_GPIO_SET_PACKET (155) — cloud -> gateway -> node
 *===========================================================*/
static void handle_gpio_set(cJSON *json_obj, uint16_t msgseqno)
{
    GpioSetCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.packetid = NODE_GPIO_SET_PACKET;
    cmd.msgseqno = msgseqno;
 
    int elemaddr = 0, node_type = 0, channel_mask = 0, level = 0;
    error_codes err;
 
    if ((err = get_required_int(json_obj, "ElementAddr", 3, 65534,
            MISSING_ELEMENT_ADDR, ELEMENT_ADDR_EXCEEDING_RANGE, &elemaddr)) != SUCCESS)
        goto reject;
    cmd.elemaddr = elemaddr;
 
    if ((err = get_required_int(json_obj, "NodeType", 0, 255,
            MISSING_NODE_TYPE, NODE_TYPE_INVALID_FORMAT, &node_type)) != SUCCESS)
        goto reject;
    if (node_type != NODE_TYPE_RS485) { err = NODE_TYPE_INVALID_FORMAT; goto reject; }
    cmd.node_type = (uint8_t)node_type;
 
    if ((err = get_required_int(json_obj, "ChannelMask", 0, 255,
            MISSING_CHANNEL_MASK, CHANNEL_MASK_EXCEEDING_RANGE, &channel_mask)) != SUCCESS)
        goto reject;
    cmd.channel_mask = (uint8_t)channel_mask;
 
    if ((err = get_required_int(json_obj, "Level", 0, 255,
            MISSING_LEVEL, LEVEL_INVALID_FORMAT, &level)) != SUCCESS)
        goto reject;
    cmd.level = (uint8_t)level;
 
    get_optional_str(json_obj, "NodeSerNo", cmd.deviceName, sizeof(cmd.deviceName));
 
    ESP_LOGI(RS485_TAG, "Relaying GPIO set to node 0x%04x: mask=0x%02x level=0x%02x",
        cmd.elemaddr, cmd.channel_mask, cmd.level);
    send_rs485_struct_to_node(cmd.elemaddr, &cmd, sizeof(cmd));
 
    rs485_pending_cmd_t *p = pending_alloc(NODE_GPIO_SET_ACK, msgseqno, cmd.elemaddr);
    if (p != NULL)
    {
        p->req.gpio_set.channel_mask = cmd.channel_mask;
        p->req.gpio_set.level        = cmd.level;
    }
    return;
 
reject:
    send_gpio_set_error_ack(msgseqno, &cmd, err);
}
 
/*===========================================================
 * NODE_SUMAN_AC_CONTROL_PACKET (158) — cloud -> gateway -> node
 *===========================================================*/
static void handle_suman_ac_control(cJSON *json_obj, uint16_t msgseqno)
{
    SumanAcControlCmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.packetid = NODE_SUMAN_AC_CONTROL_PACKET;
    cmd.msgseqno = msgseqno;
 
    int elemaddr = 0, node_type = 0, slave_addr = 0, write_mask = 0;
    int set_temp = 0, operating_mode = 0, reset_fault = 0, control_option = 0, handset_ctrl_time = 0;
    error_codes err;
 
    if ((err = get_required_int(json_obj, "ElementAddr", 3, 65534,
            MISSING_ELEMENT_ADDR, ELEMENT_ADDR_EXCEEDING_RANGE, &elemaddr)) != SUCCESS)
        goto reject;
    cmd.elemaddr = elemaddr;
 
    if ((err = get_required_int(json_obj, "NodeType", 0, 255,
            MISSING_NODE_TYPE, NODE_TYPE_INVALID_FORMAT, &node_type)) != SUCCESS)
        goto reject;
    if (node_type != NODE_TYPE_RS485) { err = NODE_TYPE_INVALID_FORMAT; goto reject; }
    cmd.node_type = (uint8_t)node_type;
 
    if ((err = get_required_int(json_obj, "SlaveAddr", 1, 247,
            MISSING_SLAVE_ADDR, SLAVE_ADDR_EXCEEDING_RANGE, &slave_addr)) != SUCCESS)
        goto reject;
    cmd.slave_addr = (uint8_t)slave_addr;
 
    if ((err = get_required_int(json_obj, "WriteMask", 0, 31,
            MISSING_WRITE_MASK, MISSING_WRITE_MASK, &write_mask)) != SUCCESS)
        goto reject;
    cmd.write_mask = (uint8_t)write_mask;
 
    // Only validate the registers WriteMask actually selects — the rest
    // keep whatever default (0) memset gave them; the node ignores
    // unselected registers per write_mask.
    if (write_mask & 0x01)
    {
        if ((err = get_required_int(json_obj, "SetTemp", 19, 32,
                MISSING_SET_TEMP, SET_TEMP_EXCEEDING_RANGE, &set_temp)) != SUCCESS)
            goto reject;
        cmd.set_temp = (uint16_t)set_temp;
    }
    if (write_mask & 0x02)
    {
        if ((err = get_required_int(json_obj, "OperatingMode", 0, 65535,
                MISSING_OPERATING_MODE, MISSING_OPERATING_MODE, &operating_mode)) != SUCCESS)
            goto reject;
        cmd.operating_mode = (uint16_t)operating_mode;
    }
    if (write_mask & 0x04)
    {
        if ((err = get_required_int(json_obj, "ResetFault", 0, 1,
                MISSING_RESET_FAULT, MISSING_RESET_FAULT, &reset_fault)) != SUCCESS)
            goto reject;
        cmd.reset_fault = (uint16_t)reset_fault;
    }
    if (write_mask & 0x08)
    {
        if ((err = get_required_int(json_obj, "ControlOption", 0, 1,
                MISSING_CONTROL_OPTION, MISSING_CONTROL_OPTION, &control_option)) != SUCCESS)
            goto reject;
        cmd.control_option = (uint16_t)control_option;
    }
    if (write_mask & 0x10)
    {
        if ((err = get_required_int(json_obj, "HandsetCtrlTime", 0, 1440,
                MISSING_HANDSET_CTRL_TIME, MISSING_HANDSET_CTRL_TIME, &handset_ctrl_time)) != SUCCESS)
            goto reject;
        cmd.handset_ctrl_time = (uint16_t)handset_ctrl_time;
    }
 
    get_optional_str(json_obj, "NodeSerNo", cmd.deviceName, sizeof(cmd.deviceName));
 
    ESP_LOGI(RS485_TAG, "Relaying Suman AC control to node 0x%04x: write_mask=0x%02x",
        cmd.elemaddr, cmd.write_mask);
    send_rs485_struct_to_node(cmd.elemaddr, &cmd, sizeof(cmd));
    return;
 
reject:
    send_gateway_error_ack(NODE_SUMAN_AC_CONTROL_ACK, msgseqno, err);
}
 
/*===========================================================
 * Public entry point — cloud -> gateway -> node commands
 *===========================================================*/
void rs485_relay_handle_cloud_command(cJSON *json_obj, mqtt_packets packetid)
{
    // MsgSeqNo is common to all four command types and needed even on
    // a validation failure (to echo back in the gateway error ACK), so
    // pull it once here rather than duplicating this in every handler.
    uint16_t msgseqno = 0;
    cJSON *msgSeqItem = cJSON_GetObjectItem(json_obj, "MsgSeqNo");
    if (msgSeqItem == NULL || !cJSON_IsNumber(msgSeqItem))
    {
        send_gateway_error_ack(ack_id_for_command(packetid), 0, MISSING_MSG_SEQ_NO);
        return;
    }
    if (msgSeqItem->valueint < 0 || msgSeqItem->valueint > 65535)
    {
        send_gateway_error_ack(ack_id_for_command(packetid), 0, MSG_SEQ_NO_EXCEEDING_RANGE);
        return;
    }
    msgseqno = (uint16_t)msgSeqItem->valueint;
 
    switch (packetid)
    {
        case NODE_METER_CONFIG_PACKET:      handle_meter_config(json_obj, msgseqno);       break;
        case NODE_GPIO_CONFIG_PACKET:        handle_gpio_config(json_obj, msgseqno);        break;
        case NODE_GPIO_SET_PACKET:           handle_gpio_set(json_obj, msgseqno);           break;
        case NODE_SUMAN_AC_CONTROL_PACKET:   handle_suman_ac_control(json_obj, msgseqno);   break;
        default:
            ESP_LOGE(RS485_TAG, "rs485_relay_handle_cloud_command called with unexpected packetid=%d", packetid);
            break;
    }
}
 
/*===========================================================
 * Node -> Gateway -> Cloud: generic ACK for the four command types
 *===========================================================*/
void rs485_relay_handle_node_ack(RS485CmdStruct *ack)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__);
        return;
    }
    rs485_pending_cmd_t *p = pending_find_by_seq(ack->msgseqno);
 
    ESP_LOGI(RS485_TAG, "Node 0x%04x ACK for packetid=%d msgseqno=%d errorcode=%d",
        ack->elemaddr, ack->packetid, ack->msgseqno, ack->errorcode);
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", ack->packetid);
    jwObj_int(&jwc, "MsgSeqNo", ack->msgseqno);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", ack->deviceName);
    jwObj_int(&jwc, "ElementAddr", ack->elemaddr);
    jwObj_int(&jwc, "NodeType", ack->node_type);
 
    if (p != NULL)
    {
        switch (ack->packetid)
        {
            case NODE_METER_CONFIG_ACK:
                jwObj_int(&jwc, "SlaveAddr", p->req.meter_cfg.slave_addr);
                jwObj_string(&jwc, "Action", (char *)meter_action_name(p->req.meter_cfg.action));
                jwObj_string(&jwc, "Label", p->req.meter_cfg.label);
                jwObj_int(&jwc, "RegCount", p->req.meter_cfg.reg_count);
                break;
            case NODE_GPIO_CONFIG_ACK:
                jwObj_int(&jwc, "ChannelMask", p->req.gpio_cfg.channel_mask);
                jwObj_int(&jwc, "Direction", p->req.gpio_cfg.direction);
                jwObj_int(&jwc, "PullUp", p->req.gpio_cfg.pull_up_mask);
                jwObj_int(&jwc, "PullDown", p->req.gpio_cfg.pull_down_mask);
                break;
            case NODE_GPIO_SET_ACK:
                jwObj_int(&jwc, "ChannelMask", p->req.gpio_set.channel_mask);
                jwObj_int(&jwc, "Level", p->req.gpio_set.level);
                break;
            default:
                break;  /* e.g. Suman ack (163) — no tracker registered yet, generic fields only */
        }
    }
 
    jwObj_string(&jwc, "ErrorSource", "NODE");
    jwObj_int(&jwc, "ErrorCode", ack->errorcode);   // node's own error_codes space — see common_types.h.
                                                     // No ErrorMsg here: the gateway only knows its OWN
                                                     // error_codes strings (get_error_code_name(), lte.h);
                                                     // hardcoding a second string table for the node's
                                                     // codes here would drift the moment common_types.h
                                                     // changes. Cloud resolves ErrorCode -> text itself
                                                     // for ErrorSource:"NODE" using the node's own table.
    jwObj_int(&jwc, "Rssi", ack->rssi);
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
 
    if (p != NULL)
        pending_free(p);
}
 
/*===========================================================
 * Scan in-flight command trackers for timeout. Same shape/fields as
 * rs485_relay_handle_node_ack() above, just with ErrorSource:"GATEWAY",
 * ErrorCode:NODE_COMM_TIMEOUT, and Rssi:-128 (no real BLE reply arrived).
 *===========================================================*/
void rs485_relay_check_timeouts(void)
{
    rs485_pending_cmd_t *curr = pending_head;
    while (curr != NULL)
    {
        rs485_pending_cmd_t *next = curr->next;  // save before potential free
 
        TickType_t elapsed_ms = (xTaskGetTickCount() - curr->start_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms >= BLE_NODE_COMM_TIMEOUT_MS)
        {
            ESP_LOGW(RS485_TAG, "Pending cmd msgseqno=%d elemaddr=0x%04x ack_packetid=%d TIMED OUT after %ums",
                curr->msgseqno, curr->elemaddr, curr->ack_packetid, (unsigned)elapsed_ms);
 
            char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
            if (buffer != NULL)
            {
                jWriteControl_t jwc;
                jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
                jwObj_int(&jwc, "JsonPacketID", curr->ack_packetid);
                jwObj_int(&jwc, "MsgSeqNo", curr->msgseqno);
                jwObj_string(&jwc, "GwySerNo", serialNoStr);
                jwObj_int(&jwc, "ElementAddr", curr->elemaddr);
                jwObj_int(&jwc, "NodeType", NODE_TYPE_RS485);
 
                switch (curr->ack_packetid)
                {
                    case NODE_METER_CONFIG_ACK:
                        jwObj_int(&jwc, "SlaveAddr", curr->req.meter_cfg.slave_addr);
                        jwObj_string(&jwc, "Action", (char *)meter_action_name(curr->req.meter_cfg.action));
                        jwObj_string(&jwc, "Label", curr->req.meter_cfg.label);
                        jwObj_int(&jwc, "RegCount", curr->req.meter_cfg.reg_count);
                        break;
                    case NODE_GPIO_CONFIG_ACK:
                        jwObj_int(&jwc, "ChannelMask", curr->req.gpio_cfg.channel_mask);
                        jwObj_int(&jwc, "Direction", curr->req.gpio_cfg.direction);
                        jwObj_int(&jwc, "PullUp", curr->req.gpio_cfg.pull_up_mask);
                        jwObj_int(&jwc, "PullDown", curr->req.gpio_cfg.pull_down_mask);
                        break;
                    case NODE_GPIO_SET_ACK:
                        jwObj_int(&jwc, "ChannelMask", curr->req.gpio_set.channel_mask);
                        jwObj_int(&jwc, "Level", curr->req.gpio_set.level);
                        break;
                    default:
                        break;
                }
 
                jwObj_string(&jwc, "ErrorSource", "GATEWAY");
                jwObj_int(&jwc, "ErrorCode", NODE_COMM_TIMEOUT);
                jwObj_string(&jwc, "ErrorMsg", get_error_code_name(NODE_COMM_TIMEOUT));
                jwObj_int(&jwc, "Rssi", -128);
                jwEnd(&jwc);
                jwClose(&jwc);
                enqueue_for_publish(buffer);
            }
            else
            {
                ESP_LOGE(RS485_TAG, "%s() - malloc failed while building timeout ACK", __func__);
            }
 
            pending_free(curr);
        }
        curr = next;
    }
}
 
/*===========================================================
 * Node -> Gateway -> Cloud: meter reading (152/153) — relayed as-is,
 * gateway does not decode Raw; cloud decodes using the DataType it
 * originally sent for this ParamIdx in NODE_METER_CONFIG_PACKET.
 *===========================================================*/
void rs485_relay_publish_meter_reading(ModbusReadingPkt *pkt)
{
    char *buffer = (char *)malloc(sizeof(char) * RS485_JSON_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__);
        return;
    }
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, RS485_JSON_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", pkt->packetid);
    jwObj_int(&jwc, "MsgSeqNo", pkt->msgseqno);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", pkt->deviceName);
    jwObj_int(&jwc, "ElementAddr", pkt->elemaddr);
    jwObj_int(&jwc, "NodeType", pkt->node_type);
    jwObj_int(&jwc, "SlaveAddr", pkt->slave_addr);
    jwObj_string(&jwc, "Label", pkt->label);
    jwObj_int(&jwc, "ErrorCode", pkt->errorcode);   // node's own error_codes space
 
    jwObj_array(&jwc, "Readings");
    for (int i = 0; i < pkt->reading_count && i < RS485_MAX_PARAMS_PER_DEVICE; i++)
    {
        jwArr_object(&jwc);
            jwObj_int(&jwc, "ParamIdx", pkt->readings[i].param_idx);
            jwObj_int(&jwc, "ModbusAddr", pkt->readings[i].modbus_addr);
            jwObj_int(&jwc, "Available", pkt->readings[i].available);
            jwObj_long_int(&jwc, (char *)"Raw", pkt->readings[i].raw);
        jwEnd(&jwc);
    }
    jwEnd(&jwc);
 
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
/*===========================================================
 * Node -> Gateway -> Cloud: GPIO status (156) — relayed as-is
 *===========================================================*/
void rs485_relay_publish_gpio_status(GpioStatusPkt *pkt)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__);
        return;
    }
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", pkt->packetid);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", pkt->deviceName);
    jwObj_int(&jwc, "ElementAddr", pkt->elemaddr);
    jwObj_int(&jwc, "ErrorCode", pkt->errorcode);   // node's own error_codes space
    jwObj_int(&jwc, "ChannelStates", pkt->channel_states);
    jwObj_int(&jwc, "ChannelDirections", pkt->channel_directions);
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
/*===========================================================
 * Node -> Gateway -> Cloud: Suman AC controller status (157) — relayed as-is
 *===========================================================*/
void rs485_relay_publish_suman_ac_status(SumanAcStatusPkt *pkt)
{
    char *buffer = (char *)malloc(sizeof(char) * RS485_JSON_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(RS485_TAG, "%s() - malloc failed", __func__);
        return;
    }
 
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, RS485_JSON_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, "JsonPacketID", pkt->packetid);
    jwObj_string(&jwc, "GwySerNo", serialNoStr);
    jwObj_string(&jwc, "NodeSerNo", pkt->deviceName);
    jwObj_int(&jwc, "ElementAddr", pkt->elemaddr);
    jwObj_int(&jwc, "SlaveAddr", pkt->slave_addr);
    jwObj_string(&jwc, "Label", pkt->label);
    jwObj_bool(&jwc, (char *)"IsEventDriven", pkt->is_event_driven);
    jwObj_int(&jwc, "ErrorCode", pkt->errorcode);   // node's own error_codes space
 
    jwObj_int(&jwc, "PcSetTemp", pkt->pc_set_temp);
    jwObj_int(&jwc, "PcOperatingMode", pkt->pc_operating_mode);
    jwObj_int(&jwc, "ControlOption", pkt->control_option);
    jwObj_int(&jwc, "HandsetCtrlTime", pkt->handset_ctrl_time);
    jwObj_int(&jwc, "NumCompressors", pkt->num_compressors);
    jwObj_int(&jwc, "ModelType", pkt->model_type);
    jwObj_int(&jwc, "CtrlSetTemp", pkt->ctrl_set_temp);
    jwObj_int(&jwc, "CtrlOperatingMode", pkt->ctrl_operating_mode);
    jwObj_int(&jwc, "FaultResetActive", pkt->fault_reset_active);
    jwObj_int(&jwc, "ReturnAirTemp", pkt->return_air_temp);
    jwObj_int(&jwc, "Comp1RunHours", pkt->comp1_run_hours);
    jwObj_int(&jwc, "Comp2RunHours", pkt->comp2_run_hours);
    jwObj_int(&jwc, "FanRunHours", pkt->fan_run_hours);
    jwObj_int(&jwc, "CompressorStatus", pkt->compressor_status);
    jwObj_int(&jwc, "FanStatus", pkt->fan_status);
    jwObj_int(&jwc, "AlarmStatus", pkt->alarm_status);
    jwObj_int(&jwc, "SystemFault", pkt->system_fault);
    jwObj_int(&jwc, "SensorFault", pkt->sensor_fault);
    jwObj_int(&jwc, "Comp1Fault", pkt->comp1_fault);
    jwObj_int(&jwc, "Comp2Fault", pkt->comp2_fault);
    jwObj_int(&jwc, "SupplyAirTemp", pkt->supply_air_temp);
    jwObj_int(&jwc, "Humidity1", pkt->humidity1);
    jwObj_int(&jwc, "Humidity2", pkt->humidity2);
    jwObj_int(&jwc, "Comp1Current", pkt->comp1_current);
    jwObj_int(&jwc, "Comp2Current", pkt->comp2_current);
 
    jwEnd(&jwc);
    jwClose(&jwc);
 
    enqueue_for_publish(buffer);
}
 
#endif /* IS_GWY */