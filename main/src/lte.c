#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <cJSON.h>
#include <json_maker.h>
#include <led.h>
#include <main.h>
#include <lte.h>
#include <heartbeat.h>
#include <flash.h>
#include <ir.h>
#include <ble_new.h>
#include <temperature_sensor.h>

char LTE_UART_data[UART_BUFFER_LEN];

/*Basic AT commands*/
const char *AT_CMD = "AT\r";
const char *CHECK_FIRMWARE_CMD = "ATI\r";
const char *CHECK_OPERATOR_SELECTION_CMD = "AT+COPS?\r";
const char *CHECK_DOMAIN_REG_CMD = "AT+CREG?\r";
const char *ENABLE_SIM_HOTSWAP_CMD = "AT+QSIMDET=1,0\r";
const char *PING_CMD = "AT+QPING=1,\"google.com\",4,1\r";
const char *PING_RESP = "+QPING:";
const char *POWER_DOWN_CMD = "AT+QPOWD\r";
char SET_BAUD_RATE_CMD[MQTT_CMD_RESP_LEN];

/*PDP and TCP config*/
const char *TCP_CONFIG_AIRTEL_CMD = "AT+QICSGP=1,1,\"airtelgprs.com\",\"\",\"\",0\r";
const char *TCP_CONFIG_JIO_CMD = "AT+QICSGP=1,1,\"jionet\",\"\",\"\",0\r";
const char *TCP_CONFIG_VI_CMD = "AT+QICSGP=1,1,\"www\",\"\",\"\",0\r";
const char *TCP_CONFIG_BSNL_CMD = "AT+QICSGP=1,1,\"bsnlnet\",\"\",\"\",0\r";
/*Configure Bands to Be Searched*/
const char *BAND_CHECK_CMD = "AT+QCFG=\"band\"\r";
const char *BAND_CONFIG_CLR_CMD =  "AT+QCFG=\"band\",0,0,1\r";
const char *BAND_CONFIG_CMD = "AT+QCFG=\"band\",0x3,0x8000000014,1\r";//0x8000000014 
const char *REBOOT_CMD = "AT+CFUN=1,1\r";
const char *PDP_CONTXT_ACT_CMD = "AT+QIACT=1\r";
const char *APN_CONFIG_CMD  = "AT+CGDCONT=1,\"IPV4V6\",\"jionet\"\r";
/*MQTT Cmd and Resp*/
const char *CLEAN_SESSION_CMD = "AT+QMTCFG=\"session\",2,1\r";
char KEEP_ALIVE_CMD[MQTT_CMD_RESP_LEN];
const char *MSG_RECV_MODE_CMD = "AT+QMTCFG=\"recv/mode\",2,0\r";
char WILL_CMD[MQTT_CMD_RESP_LEN];

char MQTT_NETWORK_OPEN_CMD[MQTT_CMD_RESP_LEN];
char MQTT_NETWORK_OPEN_RESP[MQTT_CMD_RESP_LEN];
const char *MQTT_NETWORK_CHECK_CMD = "AT+QMTOPEN?\r";
char MQTT_NETWORK_CHECK_RESP[MQTT_CMD_RESP_LEN];

const char *MQTT_NETWORK_CLOSE_CMD = "AT+QMTCLOSE=2\r";
const char *MQTT_NETWORK_CLOSE_RESP = "+QMTCLOSE: 2,0";

char MQTT_CLIENT_CONN_CMD[MQTT_CMD_RESP_LEN];
const char *MQTT_CLIENT_CONN_RESP = "+QMTCONN: 2,0,0";
const char *CHECK_CLIENT_CONN_CMD = "AT+QMTCONN?\r";
const char *MQTT_CLIENT_CONN_OK_RESP = "+QMTCONN: 2,3";
const char *MQTT_CLIENT_DISCONN_CMD = "AT+QMTDISC=2";

const char *MQTT_READ_MSG_CMD = "AT+QMTRECV=2\r";
const char *OK_RESP = "OK\r\n";

char MQTT_SUB_CMD[MQTT_CMD_RESP_LEN];
const char *MQTT_SUB_RESP = "+QMTSUB: 2,2,0";

const char *QMTSTAT_1_ERROR = "+QMTSTAT: 2,1";
const char *QMTOPEN_2_ERROR = "+QMTOPEN: 2,2";
const char *QMTOPEN_3_ERROR = "+QMTOPEN: 2,3";

char will_msg[WILL_MSG_LEN];

/*MQTT Packet JSON Keys*/
const char *JSON_PACKET_ID_KEY = "JsonPacketID";
const char *JSON_ACK_NAME_KEY = "JsonAckName";
const char *MSG_SEQ_NO_KEY = "MsgSeqNo";
const char *GWY_SER_NO_KEY = "GwySerNo";
const char *NODE_SER_NO_KEY = "NodeSerNo";
const char *LOCATION_KEY = "Location";
const char *APP_KEY_INDEX = "AppKeyIndex";
const char *APP_KEY = "AppKey";
const char *NET_KEY_INDEX = "NetKeyIndex";
const char *NET_KEY = "NetKey";
const char *ELEMENT_ADDR_KEY = "ElementAddr";
const char *MAC_ID_KEY = "MacId";
const char *MODE_KEY = "Mode";
const char *POWER_KEY = "Power";
const char *FAN_SPEED_KEY = "FanSpeed";
const char *TEMPERATURE_KEY = "Temperature";
const char *SWING_H_KEY = "SwingH";
const char *SWING_V_KEY = "SwingV";
const char *ONTIMER_KEY = "OnTimer";
const char *OFFTIMER_KEY = "OffTimer";
const char *AC_LOCKING_KEY = "Locking";
const char *UPPER_TEMPERATURE_LIMIT_KEY = "TempLockUpLimit";
const char *LOWER_TEMPERATURE_LIMIT_KEY = "TempLockLowLimit";
const char *ERROR_CODE_KEY = "ErrorCode";
const char *ERROR_MSG_KEY = "ErrorMsg";
const char *AMBIENT_TEMPERATURE_DIGITAL_DATA_KEY = "AmbientTemperature(Digital)";
const char *AMBIENT_TEMPERATURE_ANALOG_DATA_KEY = "AmbientTemperature(Analog)";
const char *PUBLISH_PERIOD_KEY = "PublishPeriodSec";
const char *FIRMWARE_VERSION_KEY = "FirmwareVersion";
const char *REGISTERED_KEY = "Registered";
const char *PROTOCOL_SEL_NUM_KEY = "Protocol";
const char *PUBLISH_MESG_QUEUE_COUNT_KEY = "PubMsgQueueCount";
const char *PROV_QUEUE_COUNT_KEY = "ProvQueueCount";
const char *UNPROV_QUEUE_COUNT_KEY = "UnProvQueueCount";
const char *AC_CONTROL_QUEUE_COUNT_KEY = "ACControlQueueCount";
const char *RECONF_QUEUE_COUNT_KEY = "ReconfQueueCount";
const char *PUB_CONF_QUEUE_COUNT_KEY = "PubConfQueueCount";
const char *TEACHING_MODE_QUEUE_COUNT_KEY = "TeachingModeQueueCount";
const char *DEBUG_INFO_QUEUE_COUNT_KEY = "DebugInfoQueueCount";
const char *DEVICE_UPTIME_KEY = "DeviceUpTimeHrs";
const char *RESTART_DEVICE_KEY = "RestartDevice";
const char *RESET_DEVICE_KEY = "ResetDevice";
const char *LINK_KEY = "Link";
const char *TEACHING_START_KEY = "TeachingStart";
const char *STARTING_TEMPERATURE_KEY = "StartingTemperature";
const char *ENDING_TEMPERATURE_KEY = "EndingTemperature";
const char *POWER_ERROR_KEY = "PowerErrorCode";
const char *TEMPERATURE_ERROR_KEY = "TemperatureErrorCode";
const char *FANSPEED_ERROR_KEY = "FanspeedErrorCode";
const char *MODE_ERROR_KEY = "ModeErrorCode";
const char *LAST_CMD_KEY = "LastCommand";
const char *NEXT_CMD_KEY = "NextCommand";
const char *REMAINING_CMD_KEY = "RemainingCommands";
const char *DETECTED_TEMPERATURE_KEY = "DetectedTemperature";
const char *BLE_ERROR_CODE_KEY = "BleErrorCode";
const char *MESSAGE_KEY = "Message";
const char *ERROR_CHK_ENABLED_KEY = "ErrorCheckEnabled";
const char *RSSI_KEY = "RSSI_dBm";

bool powerDownFlag = false;

char subscribe_topic[MQTT_TOPIC_NAME_LEN];
char publish_topic[MQTT_TOPIC_NAME_LEN];
char alive_topic[MQTT_TOPIC_NAME_LEN];
char will_topic[MQTT_TOPIC_NAME_LEN];

bool need_to_activate_pdp = false;
bool LOG_DATA = true;

char node_macid[18];

TickType_t lastNetworkCheckedTime = 0;

// Queues
QueueHandle_t publish_queue;
QueueHandle_t command_queue;

/**
 * @brief Function that publishes messages to MQTT
 * @param ack
 * @return int
 */
int mqtt_publish(char *ack, char *topic)
{
    char MQTT_PUBLISH_MESG_CMD[1024];
    sprintf(MQTT_PUBLISH_MESG_CMD, "AT+QMTPUBEX=2,2,2,0,\"%s\",%d\r\n", topic, strlen(ack));
    if (send_cmd_and_check_response(LOG_DATA, MQTT_PUBLISH_MESG_CMD, "PUBLISH_TO_MQTT", ">", 1000) == SUCCESS)
    {
        if (uart_write_bytes(UART_NUM_1, ack, strlen(ack)) != FAILURE)
        {
            return SUCCESS;
        }
        else
        {
            ESP_LOGE(LTE_TAG, "Failed to write publish message to LTE UART");
            return FAILURE;
        }
    }
    else
    {
        ESP_LOGE(LTE_TAG, "Publish Prompt not received");
        return FAILURE;
    }
    return FAILURE;
}

// Publishes ACK messages from the publish queue to MQTT broker
void publish_from_queue()
{
    char *ack_message;
    while (xQueueReceive(publish_queue, &ack_message, 0) == pdPASS)
    {
        if (mqtt_publish(ack_message, publish_topic) != SUCCESS)
        {
            ESP_LOGE(LTE_TAG, "Failed to publish ACK message to MQTT broker.\n");
            free(ack_message);
        }
        else
        {
            free(ack_message);
        }
        ESP_LOGW(LTE_TAG, "Current publish queue count : %d | Heap : %" PRIu32 " bytes", uxQueueMessagesWaiting(publish_queue), esp_get_minimum_free_heap_size());
    }
}

/**
 * @brief Function that adds acks into publish queue
 * @param ack
 */
void enqueue_for_publish(char *ack)
{
    if (xQueueSend(publish_queue, &ack, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(LTE_TAG, "Publish Queue Full. Failed to enqueue ACK");
    }
    ESP_LOGW(LTE_TAG, "Current publish queue count : %d | Heap : %" PRIu32 " bytes", uxQueueMessagesWaiting(publish_queue), esp_get_minimum_free_heap_size());
}

/**
 * @brief Function that takes care of generating node manual ac contro ack
 *
 */
void generate_node_manual_ac_control_ack(manual_control *node_ac_manual_control_t)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(LTE_TAG, "%s() - Memory allocation failed", __func__);
        return;
    }
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, JSON_PACKET_ID_KEY, NODE_MANUAL_AC_CONTROL_ACK);
    jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
    jwObj_string(&jwc, NODE_SER_NO_KEY, node_ac_manual_control_t->deviceName);
    jwObj_int(&jwc, ELEMENT_ADDR_KEY, node_ac_manual_control_t->elemAddr);
    jwObj_int(&jwc, POWER_KEY, node_ac_manual_control_t->power_value);
    jwObj_int(&jwc, DETECTED_TEMPERATURE_KEY, node_ac_manual_control_t->temperature_value);
    jwObj_int(&jwc, FAN_SPEED_KEY, node_ac_manual_control_t->fanspeed_value);
    jwObj_string(&jwc, MODE_KEY, node_ac_manual_control_t->mode);
    jwObj_int(&jwc, POWER_ERROR_KEY, node_ac_manual_control_t->power_err);
    jwObj_int(&jwc, TEMPERATURE_ERROR_KEY, node_ac_manual_control_t->temperature_err);
    jwObj_int(&jwc, FAN_SPEED_KEY, node_ac_manual_control_t->fanspeed_err);
    jwObj_int(&jwc, MODE_ERROR_KEY, node_ac_manual_control_t->mode_err);
    jwEnd(&jwc);
    jwClose(&jwc);
    enqueue_for_publish(buffer);
}

/**
 * @brief Function that takes care of generating node teaching mode ack
 */
void generate_node_teaching_mode_ack(teaching_mode *node_teaching_mode_t)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(LTE_TAG, "%s() - Memory allocation failed", __func__);
        return;
    }
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, JSON_PACKET_ID_KEY, NODE_TEACHING_MODE);
    jwObj_int(&jwc, MSG_SEQ_NO_KEY, node_teaching_mode_t->msgseqno);
    jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
    jwObj_string(&jwc, NODE_SER_NO_KEY, node_teaching_mode_t->deviceName);
    jwObj_int(&jwc, ELEMENT_ADDR_KEY, node_teaching_mode_t->elemAddr);
    jwObj_int(&jwc, ERROR_CODE_KEY, node_teaching_mode_t->errorCode);
    jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(node_teaching_mode_t->errorCode));
    jwObj_int(&jwc, BLE_ERROR_CODE_KEY, node_teaching_mode_t->bleErrorCode);
    jwObj_int(&jwc, REMAINING_CMD_KEY, node_teaching_mode_t->remainingCommands);
    jwObj_string(&jwc, LAST_CMD_KEY, node_teaching_mode_t->lastCommand);
    if (node_teaching_mode_t->errorCheckEnabled)
        jwObj_string(&jwc, NEXT_CMD_KEY, node_teaching_mode_t->nextCommand);
    jwEnd(&jwc);
    jwClose(&jwc);
    enqueue_for_publish(buffer);
}

/**
 * @brief Function that publishes the Gwy debug info ack incase of a device reset/restart in pending
 * @param ack
 */
void generate_and_publish_debug_info_ack(CommandStruct *ack)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(LTE_TAG, "%s() - Memory allocation failed", __func__);
        return;
    }
    char version[20], uptime[10];
    sprintf(version, "%d.%d.%d", MAJ_VERSION, MIN_VERSION, PATCH_VERSION);
    sprintf(uptime, "%0.2f", ((xTaskGetTickCount() * portTICK_PERIOD_MS) / 3600000.00));

    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);
    jwObj_int(&jwc, JSON_PACKET_ID_KEY, ack->packetid);
    jwObj_int(&jwc, MSG_SEQ_NO_KEY, ack->msgseqno);
    jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
    jwObj_int(&jwc, ERROR_CODE_KEY, ack->errorcode);
    jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(ack->errorcode));
    jwObj_string(&jwc, FIRMWARE_VERSION_KEY, version);
    jwObj_int(&jwc, REGISTERED_KEY, registered);
    jwObj_string(&jwc, PROTOCOL_SEL_NUM_KEY, ir_protocol);
    jwObj_int(&jwc, PUBLISH_PERIOD_KEY, publishPeriod);
    jwObj_string(&jwc, DEVICE_UPTIME_KEY, uptime);
    jwEnd(&jwc);
    jwClose(&jwc);

    char MQTT_PUBLISH_MESG_CMD[MQTT_CMD_RESP_LEN];
    sprintf(MQTT_PUBLISH_MESG_CMD, "AT+QMTPUBEX=2,2,2,0,\"%s\",%d\r\n", publish_topic, strlen(buffer));
    if (send_cmd_and_check_response(LOG_DATA, MQTT_PUBLISH_MESG_CMD, "PUBLISH_TO_MQTT", ">", 1000) == SUCCESS)
    {
        if (uart_write_bytes(UART_NUM_1, ack, strlen(buffer)) == SUCCESS)
            ;
        else
        {
            ESP_LOGE(LTE_TAG, "Failed to write publish message to LTE UART");
            led_set_state(LED_STATE_INVALID_OPERATION);
        }
        free(buffer);
    }
    else
    {
        ESP_LOGE(LTE_TAG, "Publish Prompt not received");
        led_set_state(LED_STATE_INVALID_OPERATION);
        free(buffer);
    }
}

/**
 * @brief General ack structure constructor function for both Gwy and Node
 */
void construct_general_ack(error_codes err)
{
    CommandStruct ack;
#if (IS_GWY)
    ack.packetid = GWY_GENERAL_PACKET;
#else
    ack.packetid = NODE_GENERAL_PACKET;
    ack.elemaddr = last_command.elemaddr;
    strcpy(ack.deviceName, serialNoStr);
#endif
    ack.errorcode = err;

#if (IS_GWY)
    generate_ack(ack.packetid, &ack);
#else
    send_ack_to_provisioner(ack.packetid, &ack);
#endif
}

/**
 * @brief Function that generates the ack to be sent to cloud
 * @param cmd_struct
 */
void generate_ack(mqtt_packets packetid, CommandStruct *cmd_struct)
{
    char *buffer = (char *)malloc(sizeof(char) * MQTT_ACK_BUFFER_LEN);
    if (!buffer)
    {
        ESP_LOGE(LTE_TAG, "%s() - Memory allocation failed", __func__);
        return;
    }
    jWriteControl_t jwc;

    jwOpen(&jwc, buffer, MQTT_ACK_BUFFER_LEN, JW_OBJECT, 1);

    switch (packetid)
    {
    case GWY_REG_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case GWY_UNREG_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_PROV_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_UNPROV_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        break;

    case GWY_CONF_ACK:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NVS_IR_PROTOCOL_KEY, ir_protocol);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_CONF_ACK:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_string(&jwc, NVS_IR_PROTOCOL_KEY, (char *)get_protocol_string(cmd_struct->irProtocolNum));
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case GWY_RECONF_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_RECONF_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        break;

    case GWY_AC_CONTROL_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_AC_CONTROL_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        break;

    case GWY_HEARTBEAT_ACK:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, POWER_KEY, last_command.power);
        jwObj_string(&jwc, MODE_KEY, last_command.mode_str);
        jwObj_int(&jwc, FAN_SPEED_KEY, last_command.fanspeed);
        jwObj_int(&jwc, TEMPERATURE_KEY, last_command.temperature);
        last_command.ambientTemperatureAnalog = read_analog_temperature_sensor();
        last_command.ambientTemperatureDigital = read_digital_temperature_sensor();
        jwObj_int(&jwc, AMBIENT_TEMPERATURE_DIGITAL_DATA_KEY, last_command.ambientTemperatureDigital);
        jwObj_int(&jwc, AMBIENT_TEMPERATURE_ANALOG_DATA_KEY, last_command.ambientTemperatureAnalog);
        jwObj_int(&jwc, SWING_H_KEY, last_command.swingh);
        jwObj_int(&jwc, SWING_V_KEY, last_command.swingv);
        jwObj_int(&jwc, ONTIMER_KEY, last_command.ontimer);
        jwObj_int(&jwc, OFFTIMER_KEY, last_command.offtimer);
        jwObj_int(&jwc, AC_LOCKING_KEY, last_command.locking);
        jwObj_int(&jwc, UPPER_TEMPERATURE_LIMIT_KEY, last_command.upperTemperatureLimit);
        jwObj_int(&jwc, LOWER_TEMPERATURE_LIMIT_KEY, last_command.lowerTemperatureLimit);
        break;

    case NODE_HEARTBEAT_ACK:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, POWER_KEY, cmd_struct->power);
        jwObj_string(&jwc, MODE_KEY, cmd_struct->mode_str);
        jwObj_int(&jwc, FAN_SPEED_KEY, cmd_struct->fanspeed);
        jwObj_int(&jwc, TEMPERATURE_KEY, cmd_struct->temperature);
        last_command.ambientTemperatureAnalog = read_analog_temperature_sensor();
        last_command.ambientTemperatureDigital = read_digital_temperature_sensor();
        jwObj_int(&jwc, AMBIENT_TEMPERATURE_DIGITAL_DATA_KEY, cmd_struct->ambientTemperatureDigital);
        jwObj_int(&jwc, AMBIENT_TEMPERATURE_ANALOG_DATA_KEY, cmd_struct->ambientTemperatureAnalog);
        jwObj_int(&jwc, SWING_H_KEY, cmd_struct->swingh);
        jwObj_int(&jwc, SWING_V_KEY, cmd_struct->swingv);
        jwObj_int(&jwc, ONTIMER_KEY, cmd_struct->ontimer);
        jwObj_int(&jwc, OFFTIMER_KEY, cmd_struct->offtimer);
        jwObj_int(&jwc, AC_LOCKING_KEY, cmd_struct->locking);
        jwObj_int(&jwc, UPPER_TEMPERATURE_LIMIT_KEY, cmd_struct->upperTemperatureLimit);
        jwObj_int(&jwc, LOWER_TEMPERATURE_LIMIT_KEY, cmd_struct->lowerTemperatureLimit);
        break;

    case GWY_HEARTBEAT_PUB_CONF_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_HEARTBEAT_PUB_CONF_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        break;

    case GWY_TEACHING_MODE_CMD_SELECTION_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_TEACHING_MODE_CMD_SELECTION_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        break;

    case GWY_TEACHING_MODE:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, teaching_mode_t.msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, teaching_mode_t.errorCode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(teaching_mode_t.errorCode));
        jwObj_int(&jwc, REMAINING_CMD_KEY, teaching_mode_t.remainingCommands);
        jwObj_string(&jwc, LAST_CMD_KEY, teaching_mode_t.lastCommand);
        if (teaching_mode_t.errorCheckEnabled)
            jwObj_string(&jwc, NEXT_CMD_KEY, teaching_mode_t.nextCommand);
        break;

    case NODE_TEACHING_MODE: // Case where Node Teaching Mode got timed out or was not sent at all
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, "");
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        break;

    case GWY_MANUAL_AC_CONTROL_ACK:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, POWER_KEY, ac_manual_control_t.power_value);
        jwObj_int(&jwc, DETECTED_TEMPERATURE_KEY, ac_manual_control_t.temperature_value);
        jwObj_int(&jwc, TEMPERATURE_KEY, last_command.temperature);
        jwObj_int(&jwc, FAN_SPEED_KEY, ac_manual_control_t.fanspeed_value);
        jwObj_string(&jwc, MODE_KEY, ac_manual_control_t.mode);
        jwObj_int(&jwc, POWER_ERROR_KEY, ac_manual_control_t.power_err);
        jwObj_int(&jwc, TEMPERATURE_ERROR_KEY, ac_manual_control_t.temperature_err);
        jwObj_int(&jwc, FAN_SPEED_KEY, ac_manual_control_t.fanspeed_err);
        jwObj_int(&jwc, MODE_ERROR_KEY, ac_manual_control_t.mode_err);
        break;

    case GWY_DEBUG_INFO_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        char version[20], uptime[10];
        sprintf(version, "%d.%d.%d", MAJ_VERSION, MIN_VERSION, PATCH_VERSION);
        sprintf(uptime, "%0.2f", ((xTaskGetTickCount() * portTICK_PERIOD_MS) / 3600000.00));
        jwObj_string(&jwc, FIRMWARE_VERSION_KEY, version);
        jwObj_int(&jwc, REGISTERED_KEY, registered);
        jwObj_string(&jwc, PROTOCOL_SEL_NUM_KEY, ir_protocol);
        jwObj_int(&jwc, PUBLISH_PERIOD_KEY, publishPeriod);
        jwObj_string(&jwc, DEVICE_UPTIME_KEY, uptime);
        break;

    case NODE_DEBUG_INFO_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, cmd_struct->packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        jwObj_int(&jwc, BLE_ERROR_CODE_KEY, cmd_struct->bleErrorCode);
        if (cmd_struct->errorcode == SUCCESS && cmd_struct->bleErrorCode == ESP_OK)
        {
            sprintf(version, "%d.%d.%d", cmd_struct->majversion, cmd_struct->minversion, cmd_struct->patchversion);
            sprintf(uptime, "%0.2f", cmd_struct->deviceUpTimeHrs);
            jwObj_string(&jwc, FIRMWARE_VERSION_KEY, version);
            jwObj_string(&jwc, PROTOCOL_SEL_NUM_KEY, (char *)get_protocol_string(cmd_struct->irProtocolNum));
            jwObj_int(&jwc, PUBLISH_PERIOD_KEY, cmd_struct->publishPeriodSec);
            jwObj_string(&jwc, DEVICE_UPTIME_KEY, uptime);
        }
        break;

    case GWY_GENERAL_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, cmd_struct->packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    case NODE_GENERAL_PACKET:
        jwObj_int(&jwc, JSON_PACKET_ID_KEY, cmd_struct->packetid);
        jwObj_int(&jwc, MSG_SEQ_NO_KEY, cmd_struct->msgseqno);
        jwObj_string(&jwc, GWY_SER_NO_KEY, serialNoStr);
        jwObj_string(&jwc, NODE_SER_NO_KEY, cmd_struct->deviceName);
        jwObj_int(&jwc, ELEMENT_ADDR_KEY, cmd_struct->elemaddr);
        jwObj_int(&jwc, RSSI_KEY, cmd_struct->rssi);
        jwObj_int(&jwc, ERROR_CODE_KEY, cmd_struct->errorcode);
        jwObj_string(&jwc, ERROR_MSG_KEY, get_error_code_name(cmd_struct->errorcode));
        break;

    default:
        ESP_LOGE(LTE_TAG, "Unknown Packet - %d in %s", cmd_struct->packetid, __func__);
        jwEnd(&jwc);
        jwClose(&jwc);
        return;
    }
    jwEnd(&jwc);
    jwClose(&jwc);
    enqueue_for_publish(buffer);
}

/**
 * @brief Get the error code name
 * @param code
 * @return const char*
 */
char *get_error_code_name(error_codes code)
{
    static char *error_code_names[] = {
        "FAILURE",
        "SUCCESS",
        "JSON_PACKET_INVALID",
        "MISSING_PACKET_ID",
        "PACKET_ID_EXCEEDING_RANGE",
        "MISSING_MSG_SEQ_NO",
        "MSG_SEQ_NO_EXCEEDING_RANGE",
        "MISSING_LOCATION",
        "LOCATION_EXCEEDING_RANGE",
        "NODE_COMM_TIMEOUT",
        "GWY_NOT_REG",
        "GWY_ALREADY_REG",
        "MISSING_MACID",
        "INVALID_MACID",
        "MISSING_ELEMENT_ADDR",
        "ELEMENT_ADDR_EXCEEDING_RANGE",
        "DEVICE_NOT_CONFIGURED_WITH_AC_REMOTE",
        "MISSING_POWER",
        "POWER_EXCEEDING_RANGE",
        "MISSING_MODE",
        "MODE_EXCEEDING_RANGE",
        "MISSING_FAN_SPEED",
        "FAN_SPEED_EXCEEDING_RANGE",
        "MISSING_TEMPERATURE",
        "TEMPERATURE_EXCEEDING_RANGE",
        "MISSING_SWINGH",
        "SWINGH_EXCEEDING_RANGE",
        "MISSING_SWINGV",
        "SWINGV_EXCEEDING_RANGE",
        "MISSING_ONTIMER",
        "ONTIMER_EXCEEDING_RANGE",
        "MISSING_OFFTIMER",
        "OFFTIMER_EXCEEDING_RANGE",
        "MISSING_LOCKING",
        "LOCKING_EXCEEDING_RANGE",
        "MISSING_TEMPERATURE_UPPER_LIMIT",
        "TEMPERATURE_UPPER_LIMIT_EXCEEDING_RANGE",
        "MISSING_TEMPERATURE_LOWER_LIMIT",
        "TEMPERATURE_LOWER_LIMIT_EXCEEDING_RANGE",
        "INVALID_TEMPERATURE_LOCKING_LIMITS",
        "MISSING_PUBLISH_PERIOD",
        "PUBLISH_PERIOD_EXCEEDING_RANGE",
        "MISSING_RESET_DEVICE",
        "RESET_DEVICE_EXCEEDING_RANGE",
        "MISSING_RESTART_DEVICE",
        "RESTART_DEVICE_EXCEEDING_RANGE",
        "IR_PROTOCOL_DECODEABLE_ONLY",
        "IR_PROTOCOL_FULLY_UNSUPPORTED",
        "MISSING_ERROR_CHECK",
        "ERROR_CHECK_EXCEEDING_RANGE",
        "MISSING_TEACHING_START",
        "TEACHING_START_EXCEEDING_RANGE",
        "MISSING_STARTING_TEMPERATURE",
        "STARTING_TEMPERATURE_EXCEEDING_RANGE",
        "MISSING_ENDING_TEMPERATURE",
        "ENDING_TEMPERATURE_EXCEEDING_RANGE",
        "STARTING_TEMPERATURE_LESS_THAN_ENDING_TEMPERATURE",
        "ENTERED_TEACHING_MODE",
        "EXITED_TEACHING_MODE",
        "DEVICE_ALREADY_IN_TEACHING_MODE",
        "DEVICE_NOT_IN_TEACHING_MODE",
        "POWER_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING",
        "MODE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING",
        "FANSPEED_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING",
        "TEMPERATURE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING",
        "IR_CMD_NOT_AVAILABLE_IN_FLASH",
        "NODE_NOT_FOUND_IN_PROVISIONER_DATABASE",
        "NODE_NOT_PROVISIONED",
        "IR_TASK_CREATION_FAILED",
        "ENQUEUING_INTO_COMMAND_QUEUE_FAILED",
        "DEVICE_DATA_ERASURE_SUCCESSFUL",
        "DEVICE_DATA_ERASURE_FAILED",
        "IR_PARTITION_INIT_FAILED",
        "POWER_INVALID_FORMAT",
        "TEMPERATURE_INVALID_FORMAT",
        "FAN_SPEED_INVALID_FORMAT",
        "MODE_INVALID_FORMAT",
        "SWINGH_INVALID_FORMAT",
        "SWINGV_INVALID_FORMAT",
        "LOCKING_INVALID_FORMAT",
        "ONTIMER_INVALID_FORMAT",
        "OFFTIMER_INVALID_FORMAT",
        "TEMPERATURE_UPPER_LIMIT_INVALID_FORMAT",
        "TEMPERATURE_LOWER_LIMIT_INVALID_FORMAT",
        "PACKETID_INVALID_FORMAT",
        "MSG_SEQ_NO_INVALID_FORMAT",
        "ELEMENT_ADDR_INVALID_FORMAT",
        "LOCATION_INVALID_FORMAT",
        "PUBLISH_PERIOD_INVALID_FORMAT",
        "MACID_INVALID_FORMAT",
        "TEACHING_START_INVALID_FORMAT",
        "STARTING_TEMPERATURE_INVALID_FORMAT",
        "ENDING_TEMPERATURE_INVALID_FORMAT",
        "RESTART_INVALID_FORMAT",
        "RESET_INVALID_FORMAT",
    };

    int index = code + 1; // Adjust index for negative `FAILURE` as -1

    if (index >= 0 && index < sizeof(error_code_names) / sizeof(error_code_names[0]))
    {
        return error_code_names[index];
    }
    return "UNKNOWN_ERROR";
}

/**
 * @brief Function that validates the MacID
 * @param macid
 * @return true
 * @return false
 */
bool isValidMacId(char *macid)
{
    uint8_t d = 0, s = 0;
    if (strlen(macid) != 17)
        return false;
    for (uint8_t i = 0; i < 17; i++)
    {
        if ((macid[i] >= '0' && macid[i] <= '9') ||
            (macid[i] >= 'a' && macid[i] <= 'f') ||
            (macid[i] >= 'A' && macid[i] <= 'F'))
            d++;
        else if ((i == 2 || i == 5 || i == 8 || i == 11 || i == 14) && macid[i] == ':')
            s++;
    }
    if (d == 12 && s == 5)
        return true;
    else
        return false;
}

/**
 * @brief Function that validates the command received from MQTT
 * @param json_obj Parsed MQTT command json object
 * @param cmd_struct Strucutre to which the information will get stored
 * @return int
 */
void error_check_json(cJSON *json_obj, CommandStruct *cmd_struct)
{
    cJSON *cJSONTestValue = NULL;
    cmd_struct->errorcode = SUCCESS;

    // PacketId will be present in all packets, so let's check that first
    cJSONTestValue = cJSON_GetObjectItem(json_obj, JSON_PACKET_ID_KEY);
    if (cJSONTestValue == NULL)
    {
        cmd_struct->errorcode = MISSING_PACKET_ID;
        return;
    }
    else if (!cJSON_IsNumber(cJSONTestValue))
    {
        cmd_struct->errorcode = PACKETID_INVALID_FORMAT;
        return;
    }
    mqtt_packets packetid = cJSONTestValue->valueint;

    if (
        !(packetid >= 0 && packetid < MAX_GWY_PACKET_ID) &&
        !(packetid >= 100 && packetid < MAX_NODE_PACKET_ID) &&
        !(packetid == TEST_PACKET))
    {
        cmd_struct->errorcode = PACKET_ID_EXCEEDING_RANGE;
        return;
    }
    cmd_struct->packetid = packetid;

    // If it's a test packet, then need not worry about error handling.
    if (packetid == TEST_PACKET)
    {
        ESP_LOGI(LTE_TAG, "Received TEST PACKET");
        return;
    }

    // MsgSeqNo will be present in all packets, so let's check that first
    cJSONTestValue = cJSON_GetObjectItem(json_obj, MSG_SEQ_NO_KEY);
    if (cJSONTestValue)
    {
        if (!cJSON_IsNumber(cJSONTestValue))
        {
            cmd_struct->errorcode = MSG_SEQ_NO_INVALID_FORMAT;
            return;
        }
        int32_t msg_seq_no = cJSONTestValue->valueint;
        if (msg_seq_no < 0 && msg_seq_no > 65535)
        {
            cmd_struct->errorcode = MSG_SEQ_NO_EXCEEDING_RANGE;
            return;
        }
        cmd_struct->msgseqno = msg_seq_no;
    }
    else
    {
        cmd_struct->errorcode = MISSING_MSG_SEQ_NO;
        return;
    }

    // If it's a Node packet (but not prov packet), let's check for element addr
    cJSONTestValue = cJSON_GetObjectItem(json_obj, ELEMENT_ADDR_KEY);
    if ((cmd_struct->packetid >= 100 && cmd_struct->packetid < MAX_NODE_PACKET_ID) &&
        cmd_struct->packetid != NODE_PROV_PACKET)
    {
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_ELEMENT_ADDR;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = ELEMENT_ADDR_INVALID_FORMAT;
            return;
        }
        else
        {
            int elemaddr = cJSONTestValue->valueint;
            if (elemaddr <= 2 || elemaddr >= 65535)
            {
                cmd_struct->errorcode = ELEMENT_ADDR_EXCEEDING_RANGE;
                return;
            }
            else
            {
                cmd_struct->elemaddr = elemaddr;
            }
        }
    }

    // We need to check registration packet before checking other packets
    cJSONTestValue = cJSON_GetObjectItem(json_obj, LOCATION_KEY);
    if (cmd_struct->packetid == GWY_REG_PACKET)
    {
        if (registered)
            cmd_struct->errorcode = GWY_ALREADY_REG;
        else
        {
            int length = 0;
            if (cJSONTestValue == NULL)
            {
                cmd_struct->errorcode = MISSING_LOCATION;
                return;
            }
            else if (!cJSON_IsString(cJSONTestValue))
            {
                cmd_struct->errorcode = LOCATION_INVALID_FORMAT;
                return;
            }
            else
            {
                length = strlen(cJSONTestValue->valuestring);
                if (length == 0 || length > LOCATION_STR_LEN)
                {
                    cmd_struct->errorcode = LOCATION_EXCEEDING_RANGE;
                    return;
                }
                strcpy(device_location_str, cJSONTestValue->valuestring);
            }
        }
        return;
    }

    // Now before answering any other packet types, Gwy should have been registered
    if (!registered)
    {
        cmd_struct->errorcode = GWY_NOT_REG;
        return;
    }

    // At this point, the following Node Packets must already be valid
    if (cmd_struct->packetid == NODE_RECONF_PACKET || cmd_struct->packetid == NODE_UNPROV_PACKET)
        return;

    if (cmd_struct->packetid == GWY_RECONF_PACKET && !configured)
    {
        cmd_struct->errorcode = DEVICE_NOT_CONFIGURED_WITH_AC_REMOTE;
        return;
    }

    if (cmd_struct->packetid == GWY_AC_CONTROL_PACKET || cmd_struct->packetid == NODE_AC_CONTROL_PACKET)
    {
        if (cmd_struct->packetid == GWY_AC_CONTROL_PACKET && !configured)
        {
            cmd_struct->errorcode = DEVICE_NOT_CONFIGURED_WITH_AC_REMOTE;
            return;
        }

        int power, temperature, fanspeed, swingh, swingv, locking, ontimer, offtimer, upperTemperatureLimit, lowerTemperatureLimit;
        char mode[6] = "";

        // First make sure the required Keys are available.
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, POWER_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_POWER;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = POWER_INVALID_FORMAT;
            return;
        }
        else
            power = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, TEMPERATURE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_TEMPERATURE;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = TEMPERATURE_INVALID_FORMAT;
            return;
        }
        else
            temperature = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, FAN_SPEED_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_FAN_SPEED;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = FAN_SPEED_INVALID_FORMAT;
            return;
        }
        else
            fanspeed = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, MODE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_MODE;
            return;
        }
        else if(!cJSON_IsString(cJSONTestValue)) {
            cmd_struct->errorcode = MODE_INVALID_FORMAT;
            return;
        }
        else
            strcpy(mode, cJSONTestValue->valuestring);
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, SWING_H_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_SWINGH;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = SWINGH_INVALID_FORMAT;
            return;
        }
        else
            swingh = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, SWING_V_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_SWINGV;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = SWINGV_INVALID_FORMAT;
            return;
        }
        else
            swingv = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, AC_LOCKING_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_LOCKING;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = LOCKING_INVALID_FORMAT;
            return;
        }
        else
            locking = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, ONTIMER_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_ONTIMER;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = ONTIMER_INVALID_FORMAT;
            return;
        }
        else
            ontimer = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, OFFTIMER_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_OFFTIMER;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = OFFTIMER_INVALID_FORMAT;
            return;
        }
        else
            offtimer = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, UPPER_TEMPERATURE_LIMIT_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_TEMPERATURE_UPPER_LIMIT;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = TEMPERATURE_UPPER_LIMIT_INVALID_FORMAT;
            return;
        }
        else
            upperTemperatureLimit = cJSONTestValue->valueint;
        //========================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, LOWER_TEMPERATURE_LIMIT_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_TEMPERATURE_LOWER_LIMIT;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = TEMPERATURE_LOWER_LIMIT_INVALID_FORMAT;
            return;
        }
        else
            lowerTemperatureLimit = cJSONTestValue->valueint;
        //========================================================//
        // Now make sure all values are within range
        if (power != 0 && power != 1)
        {
            cmd_struct->errorcode = POWER_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->power = power;

        // Check Temperature exceding range
        if (temperature < 18 || temperature > 32)
        {
            cmd_struct->errorcode = TEMPERATURE_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->temperature = temperature;

        // If Device is using teaching mode, then let's check if the temperature is within range w.r.t saved values
        if (strcmp(ir_protocol, RAW_IR_PROTOCOL) == 0)
        {
            if (!(cmd_struct->temperature >= teaching_mode_t.startingTemperature &&
                  cmd_struct->temperature <= teaching_mode_t.endingTemperature))
            {
                cmd_struct->errorcode = IR_CMD_NOT_AVAILABLE_IN_FLASH;
                return;
            }
        }

        /* Setting fan speed */ 
        if (!(fanspeed >= 0 && fanspeed <= 5))
        {
            cmd_struct->errorcode = FAN_SPEED_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->fanspeed = fanspeed;
        // if (strcasecmp(mode, "Cool") == 0 ||
        //     strcasecmp(mode, "Hot") == 0 ||
        //     strcasecmp(mode, "Auto") == 0 ||
        //     strcasecmp(mode, "Dry") == 0 ||
        //     strcasecmp(mode, "Fan") == 0)
        //     ;
        // else
        // {
        //     cmd_struct->errorcode = MODE_EXCEEDING_RANGE;
        //     return;
        // }

        /* Setting mode_num */ 
        strcpy(cmd_struct->mode_str, mode);
        if (strcasecmp(mode, "Auto") == 0) cmd_struct->mode_num = 0;  /* stdAc::opmode_t::kAuto */
        else if (strcasecmp(mode, "Cool") == 0) cmd_struct->mode_num = 1;  /* stdAc::opmode_t::kCool */
        else if (strcasecmp(mode, "Hot")  == 0) cmd_struct->mode_num = 2;  /* stdAc::opmode_t::kHeat */
        else if (strcasecmp(mode, "Dry")  == 0) cmd_struct->mode_num = 3;  /* stdAc::opmode_t::kDry  */
        else if (strcasecmp(mode, "Fan")  == 0) cmd_struct->mode_num = 4;  /* stdAc::opmode_t::kFan  */

        /* Setting Swing Horizontal  */ 
        if (swingh != 0 && swingh != 1)
        {
            cmd_struct->errorcode = SWINGH_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->swingh = swingh;

        /* Setting Swing Vertical */  
        if (swingv != 0 && swingv != 1)
        {
            cmd_struct->errorcode = SWINGV_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->swingv = swingv;

        /* Locking Check */ 
        if (locking != 0 && locking != 1)
        {
            cmd_struct->errorcode = LOCKING_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->locking = locking;

        /* On Timer Check */ 
        if (!(ontimer >= 0 && ontimer <= 12))
        {
            cmd_struct->errorcode = ONTIMER_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->ontimer = ontimer;

        /* Off timer check */ 
        if (!(offtimer >= 0 && offtimer <= 12))
        {
            cmd_struct->errorcode = OFFTIMER_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->offtimer = offtimer;

        /* Upper Temperature Limit check  */ 
        if (upperTemperatureLimit < 18 || upperTemperatureLimit > 32)
        {
            cmd_struct->errorcode = TEMPERATURE_UPPER_LIMIT_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->upperTemperatureLimit = upperTemperatureLimit;

        /* Lower Temperature Limit check */  
        if (lowerTemperatureLimit < 18 || lowerTemperatureLimit > 32)
        {
            cmd_struct->errorcode = TEMPERATURE_LOWER_LIMIT_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->lowerTemperatureLimit = lowerTemperatureLimit;

        if (upperTemperatureLimit < lowerTemperatureLimit)
        {
            ESP_LOGD(LTE_TAG, "UL : %d | LL : %d", upperTemperatureLimit, lowerTemperatureLimit);
            cmd_struct->errorcode = INVALID_TEMPERATURE_LOCKING_LIMITS;
            return;
        }
        return;
    }

    if (cmd_struct->packetid == GWY_HEARTBEAT_PUB_CONF_PACKET || cmd_struct->packetid == NODE_HEARTBEAT_PUB_CONF_PACKET)
    {
        cJSONTestValue = cJSON_GetObjectItem(json_obj, PUBLISH_PERIOD_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_PUBLISH_PERIOD;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = PUBLISH_PERIOD_INVALID_FORMAT;
            return;
        }
        else
        {
            int publishperiod = cJSONTestValue->valueint;
            if(!(publishperiod>= DEFAULT_PUBLISH_PERIOD_SEC && publishperiod <= 65535)) cmd_struct->errorcode = PUBLISH_PERIOD_EXCEEDING_RANGE;
            else cmd_struct->publishPeriodSec = publishperiod;
            // cmd_struct->publishPeriodSec = publishperiod;
            return;
        }
    }

    if (cmd_struct->packetid == NODE_PROV_PACKET)
    {
        cJSONTestValue = cJSON_GetObjectItem(json_obj, MAC_ID_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_MACID;
            return;
        }
        else if(!cJSON_IsString(cJSONTestValue)) {
            cmd_struct->errorcode = MACID_INVALID_FORMAT;
            return;
        }
        else
        {
            char macid[20];
            strcpy(macid, cJSONTestValue->valuestring);
            if (!isValidMacId(macid))
                cmd_struct->errorcode = INVALID_MACID;
            else
                strcpy(node_macid, macid);
            return;
        }
    }

    if (cmd_struct->packetid == GWY_TEACHING_MODE || cmd_struct->packetid == NODE_TEACHING_MODE)
    {
        int teaching_start, startingTemp, endingTemp;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, TEACHING_START_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_TEACHING_START;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = TEACHING_START_INVALID_FORMAT;
            return;
        }
        teaching_start = cJSONTestValue->valueint;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, STARTING_TEMPERATURE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_STARTING_TEMPERATURE;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = STARTING_TEMPERATURE_INVALID_FORMAT;
            return;
        }
        startingTemp = cJSONTestValue->valueint;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, ENDING_TEMPERATURE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_ENDING_TEMPERATURE;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = ENDING_TEMPERATURE_INVALID_FORMAT;
            return;
        }
        endingTemp = cJSONTestValue->valueint;
        //=================================================================//
        
        if (teaching_start != 0 && teaching_start != 1)
        {
            cmd_struct->errorcode = TEACHING_START_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->teachingStart = teaching_start;

        if (cmd_struct->packetid == GWY_TEACHING_MODE)
        {
            if (teaching_start && teaching_in_progress)
            {
                cmd_struct->errorcode = DEVICE_ALREADY_IN_TEACHING_MODE;
                return;
            }

            if (!teaching_start && !teaching_in_progress)
            {
                cmd_struct->errorcode = DEVICE_NOT_IN_TEACHING_MODE;
                return;
            }
        }

        if (!(startingTemp >= MAX_LOW_TEMP && startingTemp <= MAX_HIGH_TEMP))
        {
            cmd_struct->errorcode = STARTING_TEMPERATURE_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->startingTemperature = startingTemp;

        if (!(endingTemp >= MAX_LOW_TEMP && endingTemp <= MAX_HIGH_TEMP))
        {
            cmd_struct->errorcode = ENDING_TEMPERATURE_EXCEEDING_RANGE;
            return;
        }
        else
            cmd_struct->endingTemperature = endingTemp;

        if (startingTemp > endingTemp)
        {
            cmd_struct->errorcode = STARTING_TEMPERATURE_LESS_THAN_ENDING_TEMPERATURE;
        }
        return;
    }

    if (cmd_struct->packetid == GWY_TEACHING_MODE_CMD_SELECTION_PACKET || cmd_struct->packetid == NODE_TEACHING_MODE_CMD_SELECTION_PACKET)
    {
        int power, temperature;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, POWER_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_POWER;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = POWER_INVALID_FORMAT;
            return;
        }
        else power = cJSONTestValue->valueint;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, TEMPERATURE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_TEMPERATURE;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = TEMPERATURE_INVALID_FORMAT;
            return;
        }
        temperature = cJSONTestValue->valueint;
        //=================================================================//
        if (power != 0 && power != 1)
        {
            cmd_struct->errorcode = POWER_EXCEEDING_RANGE;
            return;
        }
        else
        {
            if (cmd_struct->packetid == GWY_TEACHING_MODE_CMD_SELECTION_PACKET)
                teaching_mode_t.power = power;
            else
                cmd_struct->power = power;
        }

        if (!(temperature >= teaching_mode_t.startingTemperature && temperature <= teaching_mode_t.endingTemperature) &&
            cmd_struct->packetid == GWY_TEACHING_MODE_CMD_SELECTION_PACKET)
        {
            cmd_struct->errorcode = TEMPERATURE_EXCEEDING_RANGE;
            return;
        }
        else
        {
            if (cmd_struct->packetid == GWY_TEACHING_MODE_CMD_SELECTION_PACKET)
                teaching_mode_t.temperature = temperature;
            else
                cmd_struct->temperature = temperature;
        }
        return;
    }

    if (cmd_struct->packetid == GWY_DEBUG_INFO_PACKET || cmd_struct->packetid == NODE_DEBUG_INFO_PACKET)
    {
        int restart, reset;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, RESTART_DEVICE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_RESTART_DEVICE;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = RESTART_INVALID_FORMAT;
            return;
        }
        else restart = cJSONTestValue->valueint;
        //=================================================================//
        cJSONTestValue = cJSON_GetObjectItem(json_obj, RESET_DEVICE_KEY);
        if (cJSONTestValue == NULL)
        {
            cmd_struct->errorcode = MISSING_RESET_DEVICE;
            return;
        }
        else if(!cJSON_IsNumber(cJSONTestValue)) {
            cmd_struct->errorcode = RESET_INVALID_FORMAT;
            return;
        }
        else reset = cJSONTestValue->valueint;
        //=================================================================//
        
        if (restart != 0 && restart != 1)
        {
            cmd_struct->errorcode = RESTART_DEVICE_EXCEEDING_RANGE;
            return;
        }
        if (reset != 0 && reset != 1)
        {
            cmd_struct->errorcode = RESET_DEVICE_EXCEEDING_RANGE;
            return;
        }
        cmd_struct->restartDevice = restart;
        cmd_struct->resetDevice = reset;
        return;
    }
}

/**
 * @brief Function that takes care of registering Gwy
 */
void register_gwy()
{
    registered = 1;
    update_led_status();
    set_str_in_nvs_flash(GENERAL_HANDLE, NVS_DEVICE_LOCATION_KEY, device_location_str);
    set_number_in_nvs_flash(GENERAL_HANDLE, NVS_REGISTERED_KEY, 1, UINT8_SIZE);
    hb_timer_start();
    ble_init();
    ble_initialized = true;
    if (xTaskCreate(ir_recv_task, "IR Recv Task", IR_THREAD_STACK_SIZE, NULL, 2, &ir_recv_task_handle) != pdPASS)
    {
        ESP_LOGE(LTE_TAG, "IR Recv Task Creation Failed");
        construct_general_ack(IR_TASK_CREATION_FAILED);
    }
}

/**
 * @brief Function that unregisters, factory resets and restart the device
 */
void unregister(uint16_t msgseqno)
{
    registered = 0;
    update_led_status();
    CommandStruct ack;
    ack.packetid = GWY_UNREG_PACKET;
    ack.msgseqno = msgseqno;
    ack.errorcode = SUCCESS;
    generate_ack(ack.packetid, &ack);
    vTaskDelete(ir_recv_task_handle);
    factory_reset_device();
    hb_timer_stop();
    powerDownFlag = true;
}

void parse_json()
{
    led_set_state(LED_STATE_CMD_RECVD);
    CommandStruct cmd_struct;

    cJSON *json_obj = cJSON_Parse(LTE_UART_data);
    if (json_obj == NULL)
    {
        ESP_LOGE(LTE_TAG, "MQTT command parsing failed");
        return;
    }

    error_check_json(json_obj, &cmd_struct);
    ESP_LOGE(LTE_TAG, "Error Code : %s - %d", get_error_code_name(cmd_struct.errorcode), cmd_struct.errorcode);

    if (cmd_struct.errorcode == SUCCESS)
    {
        switch (cmd_struct.packetid)
        {
        case GWY_REG_PACKET:
            register_gwy();
            break;

        case GWY_UNREG_PACKET:
            unregister(cmd_struct.msgseqno);
            return;

        case GWY_AC_CONTROL_PACKET:
            handle_ac_control(&cmd_struct);
            break;

        case GWY_RECONF_PACKET:
            handle_reconfiguration(&cmd_struct);
            break;

        case GWY_HEARTBEAT_PUB_CONF_PACKET:
            handle_setting_hb_publish_configuration(cmd_struct.publishPeriodSec);
            break;

        case GWY_DEBUG_INFO_PACKET:
            if (!cmd_struct.resetDevice && !cmd_struct.restartDevice)
                generate_ack(cmd_struct.packetid, &cmd_struct);
            if (cmd_struct.resetDevice)
            {
                cmd_struct.errorcode = factory_reset_device();
                generate_and_publish_debug_info_ack(&cmd_struct);
                powerCycleDevice(DUE_TO_MQTT_CMD);
            }
            if (cmd_struct.restartDevice)
            {
                generate_and_publish_debug_info_ack(&cmd_struct);
                powerCycleDevice(DUE_TO_MQTT_CMD);
            }
            return;

        case GWY_TEACHING_MODE:
            handle_configuring_teaching_mode(DUE_TO_MQTT_CMD, &cmd_struct);
            return;

        case GWY_TEACHING_MODE_CMD_SELECTION_PACKET:
            break;

        case NODE_UNPROV_PACKET:
        case NODE_AC_CONTROL_PACKET:
        case NODE_DEBUG_INFO_PACKET:
        case NODE_RECONF_PACKET:
        case NODE_HEARTBEAT_PUB_CONF_PACKET:
        case NODE_TEACHING_MODE:
        case NODE_TEACHING_MODE_CMD_SELECTION_PACKET:
            send_cmd_to_node(&cmd_struct);
            ESP_LOGW(LTE_TAG, "Currnet command queue count : %d | Heap : %" PRIu32 " bytes", uxQueueMessagesWaiting(command_queue), esp_get_minimum_free_heap_size());
            return;

        default:
            break;
        }
        generate_ack(cmd_struct.packetid, &cmd_struct);
    }
    else
    {
        led_set_state(LED_STATE_INVALID_OPERATION);

        if (cmd_struct.packetid == GWY_TEACHING_MODE)
        {
            teaching_mode_t.errorCode = cmd_struct.errorcode;
            generate_ack(GWY_TEACHING_MODE, NULL);
            return;
        }
        else if (cmd_struct.packetid == NODE_TEACHING_MODE)
        {
            teaching_mode_t.errorCode = cmd_struct.errorcode;
            generate_ack(NODE_TEACHING_MODE, NULL);
            return;
        }
        strcpy(cmd_struct.deviceName, "");
        generate_ack(cmd_struct.packetid, &cmd_struct);
    }
}

error_codes check_response(char *uart_data, const char *check_string)
{
    if (strstr(uart_data, QMTOPEN_2_ERROR))
        return QMTOPEN_2_ERRORCODE;
    else if (strstr(uart_data, QMTOPEN_3_ERROR))
        return QMTOPEN_3_ERRORCODE;
    else if (strstr(uart_data, QMTSTAT_1_ERROR))
        return QMTSTAT_1_ERRORCODE;
    else if (strstr(uart_data, check_string))
        return SUCCESS;
    else
        return FAILURE;
}

/**
 * @brief Function that fetches data from LTE and checks if it is valid or not
 * @param cmd_id ID that is used to know what cmd was sent. This is being used instead of string comparisons which are expensive
 * @param timeout_ms How long should LTE wait for a response for the sent command.
 * @param check_string String that is used to check against the response to know if it is valid or not
 * @return int8_t
 */
error_codes fetch_and_check_data(bool logging, uint16_t timeout_ms, const char *check_string, const char *cmd_name)
{
    static uint8_t fail_counter = 0;
    error_codes rc = FAILURE;
    bzero(LTE_UART_data, UART_BUFFER_LEN);
    uart_flush(UART_NUM_1);
    int length = uart_read_bytes(UART_NUM_1, LTE_UART_data, UART_BUFFER_LEN, pdMS_TO_TICKS(timeout_ms));
    if (length > 0)
    {
        fail_counter = 0;
        if (logging)
            ESP_LOGI(LTE_TAG, "Received : %s", LTE_UART_data);
        if ((rc = check_response(LTE_UART_data, check_string)) == SUCCESS)
        {
            if (strstr(LTE_UART_data, "{"))
            {
                strcpy(LTE_UART_data, strstr(LTE_UART_data, "{"));
                ESP_LOGW(LTE_TAG, "%s", LTE_UART_data);
                parse_json();
            }
        }
    }
    else
    {
        ESP_LOGE(LTE_TAG, "No Data | data_len : %d", strlen(LTE_UART_data));
        if (fail_counter++ > 25)
            esp_restart();
    }
    return rc;
}

/**
 * @brief Function that takes care of sending an AT command to LTE, await response, check that response
 * is valid or not and perform necessary operation.
 * @param logging Flag that enables/disables logging of LTE-MQTT communication data
 * @param cmd The AT command that needs to be sent
 * @param requestString A string denoting what command is being sent. Gets logged if logging is enabled.
 * @param check_string String that needs to be compared against the response that we get from the LTE chip
 * @param response_wait_time_ms How long should the LTE chip wait for a response (in milliseconds)
 * @return 0=Success, -1=Failure
 */
error_codes send_cmd_and_check_response(bool logging, const char *cmd,
                                        const char *cmdName, const char *check_string, uint32_t timeout_ms)
{
    error_codes rc = FAILURE;
    uart_flush_input(UART_NUM_1);
    if (uart_write_bytes(UART_NUM_1, cmd, strlen(cmd)) != FAILURE)
    {
        if (logging)
        {
            ESP_LOGI(LTE_TAG, "Command sent : %s", cmd);
            ESP_LOGI(LTE_TAG, "WaitTime : %ldms", timeout_ms);
        }
        rc = fetch_and_check_data(logging, timeout_ms, check_string, cmdName);
    }
    else
    {
        ESP_LOGE(LTE_TAG, "Error in sending AT command to LTE!!!");
    }
    return rc;
}

/**
 * @brief Function that removes an item from the queue based on msgseqno
 * @param queue the queue from which the item needs to be removed from
 * @param msgseqno the msgseq no of the item to be removed
 * @retval true If item was removed
 * @retval false If item was not removed
 */
bool removeQueueItemByMsgSeqNo(QueueHandle_t queue, uint16_t msgseqno)
{
    CommandStruct temp_item;
    bool item_found = false;
    if (uxQueueMessagesWaiting(queue) == 0)
        return false;
    QueueHandle_t temp_queue = xQueueCreate(uxQueueMessagesWaiting(queue), sizeof(CommandStruct));

    if (temp_queue == NULL)
    {
        ESP_LOGE(LTE_TAG, "Temporary queue creation failed!");
        return false;
    }

    // Transfer items to the temporary queue, skipping the matched one
    while (uxQueueMessagesWaiting(queue) > 0)
    {
        if (xQueueReceive(queue, &temp_item, 0) == pdPASS)
        {
            if (temp_item.msgseqno == msgseqno)
            {
                ESP_LOGI(LTE_TAG, "Removing item with msgseqno=%d", msgseqno);
                item_found = true; // Mark as found
            }
            else
            {
                xQueueSend(temp_queue, &temp_item, portMAX_DELAY);
            }
        }
    }

    // Restore remaining items back to the original queue
    while (uxQueueMessagesWaiting(temp_queue) > 0)
    {
        if (xQueueReceive(temp_queue, &temp_item, 0) == pdPASS)
        {
            xQueueSend(queue, &temp_item, portMAX_DELAY);
        }
    }

    vQueueDelete(temp_queue);
    ESP_LOGW(LTE_TAG, "Current command queue count : %d | Heap : %" PRIu32 " bytes", uxQueueMessagesWaiting(command_queue), esp_get_minimum_free_heap_size());
    return item_found;
}

/**
 * @brief Function which removes stale items from the command queue
 * items in the command queue are considered stale if they have been in the queue
 * for more than the BLE_NODE_COMM_TIMEOUT duration
 */
void maintainCommandQueue()
{
    CommandStruct cmd_item;
    UBaseType_t queue_length = uxQueueMessagesWaiting(command_queue);
    esp_err_t err;
    for (UBaseType_t i = 0; i < queue_length; i++)
    {
        // Peek at the front item without removing it
        if (xQueuePeek(command_queue, &cmd_item, 0) == pdPASS)
        {
            if (cmd_item.requestSentToNode && (((xTaskGetTickCount() - cmd_item.reqSentToNodeTicks) * portTICK_PERIOD_MS) > BLE_NODE_COMM_TIMEOUT_MS))
            {
                ESP_LOGE(LTE_TAG, "MsgSeqNo : %d | currentTickCount : %ld | ReqSentTickCount : %ld", cmd_item.msgseqno, xTaskGetTickCount(), cmd_item.reqSentToNodeTicks);
                // Remove the stale item
                if ((err = xQueueReceive(command_queue, &cmd_item, 0)) == pdPASS)
                {
                    cmd_item.errorcode = NODE_COMM_TIMEOUT;
                    strcpy(cmd_item.deviceName, "");
                    cmd_item.bleErrorCode = SUCCESS;
                    generate_ack(cmd_item.packetid, &cmd_item);
                    ESP_LOGW(LTE_TAG, "Removed stale command from queue: msgseqno=%d", cmd_item.msgseqno);
                    ESP_LOGW(LTE_TAG, "Current command queue count : %d | Heap : %" PRIu32 " bytes", uxQueueMessagesWaiting(command_queue), esp_get_minimum_free_heap_size());
                }
                else
                    ESP_LOGE(LTE_TAG, "Failed to remove stale element : %s", esp_err_to_name(err));
            }
            else
            {
                // No more stale items, break early
                break;
            }
        }
    }
}

/**
 * Function that takes care of maintaining the MQTT communication with the broker
 * and also publishing ACK message from pubmessage queue to the Cloud
 * @param none
 * @retval none
 */
void maintainMQTTConnection()
{
    error_codes rc;
    static uint8_t ping_fail_counter = 0;
    send_cmd_and_check_response(LOG_DATA, BAND_CHECK_CMD, "BAND_CHECK_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    if (need_to_activate_pdp)
    {
        send_cmd_and_check_response(LOG_DATA, TCP_CONFIG_JIO_CMD, "TCP_CONFIG_JIO_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
        send_cmd_and_check_response(LOG_DATA, APN_CONFIG_CMD, "APN_CONFIG_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
        send_cmd_and_check_response(LOG_DATA, "AT+CGDCONT=2,\"IP\",\"\"\r\n", "APN_CONFIG_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
        send_cmd_and_check_response(LOG_DATA, "AT+CGDCONT=3,\"IP\",\"\"\r\n", "APN_CONFIG_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
        send_cmd_and_check_response(LOG_DATA, "AT+CGDCONT?\r\n", "CGDCONT_CHECK_CMD", OK_RESP, 1000);
        send_cmd_and_check_response(LOG_DATA, "AT+QIDEACT=1\r\n", "DEACT_CMD", OK_RESP, 1000);
        if (send_cmd_and_check_response(LOG_DATA, PDP_CONTXT_ACT_CMD, "PDP_CONTXT_ACT_CMD", OK_RESP, 5000) != SUCCESS){
            return;
        }
        else{
            send_cmd_and_check_response(LOG_DATA, "AT+QIACT?\r\n", "ACT_CHECK_CMD", OK_RESP, 1000);
            need_to_activate_pdp=false;
        }
    }
    if ((rc = send_cmd_and_check_response(LOG_DATA, MQTT_NETWORK_OPEN_CMD, "MQTT_NETWORK_OPEN_CMD", MQTT_NETWORK_OPEN_RESP, MIN_LTE_RESP_WAIT_MS * 5)) == SUCCESS)
        ;
    else
    {
        switch (rc)
        {
        case QMTOPEN_2_ERRORCODE:
            ESP_LOGE(LTE_TAG, "QMTOPEN_2_ERRORCODE");
            send_cmd_and_check_response(LOG_DATA, MQTT_NETWORK_CLOSE_CMD, "MQTT_NETWORK_CLOSE_CMD", MQTT_NETWORK_CLOSE_RESP, MIN_LTE_RESP_WAIT_MS * 5);
            return;

        case QMTOPEN_3_ERRORCODE:
            ESP_LOGE(LTE_TAG, "QMTOPEN_3_ERRORCODE");
            need_to_activate_pdp = true;
            return;

        default:
            break;
        }
    }
    if (send_cmd_and_check_response(LOG_DATA, MQTT_CLIENT_CONN_CMD, "MQTT_CLIENT_CONN_CMD", MQTT_CLIENT_CONN_RESP, MIN_LTE_RESP_WAIT_MS * 5) == SUCCESS)
        ;
    else
    {
        ;
    }

    if (send_cmd_and_check_response(LOG_DATA, MQTT_SUB_CMD, "MQTT_SUB_CMD", MQTT_SUB_RESP, MIN_LTE_RESP_WAIT_MS * 5) != SUCCESS)
        return;
    ESP_LOGI(LTE_TAG, "Resumed MQTT Connection");
    if (registered && !ble_initialized)
    {
        ble_initialized = true;
        ble_init();
    }
    need_to_activate_pdp = false;
    mqtt_connected = true;
    update_led_status();
    LOG_DATA = false;
    if (registered)
        hb_timer_restart();
    while (!powerDownFlag)
    {
        if (send_cmd_and_check_response(LOG_DATA, MQTT_READ_MSG_CMD, "MQTT_READ_MSG_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS * 2) == SUCCESS)
        {
            ;
        }
        if (xTaskGetTickCount() - lastNetworkCheckedTime > NETWORK_CHECK_INTERVAL_TICKS)
        {
            lastNetworkCheckedTime = xTaskGetTickCount();
            if (send_cmd_and_check_response(LOG_DATA, MQTT_NETWORK_CHECK_CMD, "MQTT_NETWORK_CHECK_CMD", MQTT_NETWORK_CHECK_RESP, MIN_LTE_RESP_WAIT_MS * 3) != SUCCESS)
            {
                ESP_LOGE(LTE_TAG, "Lost MQTT connection");
                return;
            }
            else
            {
                if (mqtt_publish(alive_msg, alive_topic) == SUCCESS)
                {
                    ;
                }
                else
                    ESP_LOGE(LTE_TAG, "Alive message publish failed");
            }
        }
        if (send_cmd_and_check_response(LOG_DATA, PING_CMD, "PING_CMD", OK_RESP, 50) != SUCCESS)
        {
            if (ping_fail_counter++ > RETRY_COUNT)
            {
                ESP_LOGE(LTE_TAG, "Lost MQTT connection");
                ping_fail_counter = 0;
                mqtt_connected = false;
                update_led_status();
                if (registered)
                    hb_timer_stop();
                LOG_DATA = true;
                break;
            }
            continue;
        }
        ping_fail_counter = 0;
        maintainCommandQueue();
        publish_from_queue();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Function that takes care of configuring the MQTT communication
 * @param none
 * @retval none
 */
void MQTT_config()
{
    send_cmd_and_check_response(LOG_DATA, CLEAN_SESSION_CMD, "CLEAN_SESSION_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, KEEP_ALIVE_CMD, "KEEP_ALIVE_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, WILL_CMD, "WILL_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS * 5);
}

/**
 * @brief Function that performs basic AT cmd checks at startup
 * @param none
 * @retval none
 */
void execute_general_AT_cmds()
{
    /*Configure Bands to Be Searched*/
    send_cmd_and_check_response(LOG_DATA, BAND_CHECK_CMD, "BAND_CHECK_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    send_cmd_and_check_response(LOG_DATA, BAND_CONFIG_CLR_CMD, "BAND_CONFIG_CLR_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    send_cmd_and_check_response(LOG_DATA, BAND_CONFIG_CMD, "BAND_CONFIG_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    send_cmd_and_check_response(LOG_DATA, BAND_CHECK_CMD, "BAND_CHECK_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    send_cmd_and_check_response(LOG_DATA, REBOOT_CMD, "REBOOT_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    vTaskDelay(pdMS_TO_TICKS(500));
    send_cmd_and_check_response(LOG_DATA, BAND_CHECK_CMD, "BAND_CHECK_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS*10);
    send_cmd_and_check_response(LOG_DATA, "AT&V\r", "DISPLAY_CURRENT_CONFIGURATION", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, "ATE0\r", "TURN_OFF_ECHO_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, CHECK_FIRMWARE_CMD, "CHECK_FIRMWARE_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, CHECK_OPERATOR_SELECTION_CMD, "CHECK_OP_SEL_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, CHECK_DOMAIN_REG_CMD, "CHECK_DOMAIN_REG_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
    send_cmd_and_check_response(LOG_DATA, ENABLE_SIM_HOTSWAP_CMD, "ENABLE_SIM_HOTSWAP_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS);
}

/**
 * @brief Function that takes care of initializing certain
 * string variables that we use across the LTE communication
 * like Subscribe topic, publish topic, etc.,
 * @param none
 * @retval none
 */
void initialize_mqtt_cmd_strings()
{
    sprintf(KEEP_ALIVE_CMD, "AT+QMTCFG=\"keepalive\",%d,%d\r", MQTT_CLIENT_INDEX, publishPeriod + 5);
    sprintf(SET_BAUD_RATE_CMD, "AT+IPR=%d\r", BAUD_RATE);
    sprintf(MQTT_NETWORK_OPEN_CMD, "AT+QMTOPEN=%d,\"%s\",%d\r", MQTT_CLIENT_INDEX, MQTT_SERVER_IP, MQTT_PORT);
    sprintf(MQTT_NETWORK_OPEN_RESP, "+QMTOPEN: %d,0", MQTT_CLIENT_INDEX);
    sprintf(MQTT_NETWORK_CHECK_RESP, "+QMTOPEN: %d,\"%s\",%d", MQTT_CLIENT_INDEX, MQTT_SERVER_IP, MQTT_PORT);
    sprintf(MQTT_CLIENT_CONN_CMD, "AT+QMTCONN=%d,\"%s_abcd\",\"%s\",\"%s\"\r", MQTT_CLIENT_INDEX, serialNoStr, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    sprintf(MQTT_SUB_CMD, "AT+QMTSUB=%d,%d,\"%s\",%d\r", MQTT_CLIENT_INDEX, MQTT_MSG_ID, subscribe_topic, MQTT_QOS);
    sprintf(WILL_CMD, "AT+QMTCFG=\"will\",%d,%d,%d,%d,\"%s\",\"%s\"\r",
            MQTT_CLIENT_INDEX,
            MQTT_WILL_FLAG,
            MQTT_WILL_QOS,
            MQTT_WILL_RETAIN,
            will_topic,
            will_msg);
}

/**
 * @brief Function that configures the POWER and RESET pins of LTE as Output
 * @param none
 * @retval none
 */
void lte_gpio_configuration()
{
    gpio_set_direction(LTE_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LTE_POWER_PIN, GPIO_MODE_OUTPUT);
}

/**
 * @brief Funcion that performs the Powering down sequence
 *
 */
void powerCycleDevice(action_type_t type)
{
    led_set_state(LED_STATE_POWERING_DOWN);
    powerDownFlag = true;

    // Close the NVS handles
    nvs_close(ir_nvs_handle);
    nvs_close(general_nvs_handle);

#if (IS_GWY)
    while (send_cmd_and_check_response(true, POWER_DOWN_CMD, "POWER_DOWN_CMD", OK_RESP, MIN_LTE_RESP_WAIT_MS) != SUCCESS)
        vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 30; i > 0; i--)
    {
        ESP_LOGI(LTE_TAG, "Restarting in %d seconds ... ", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    for (int i = 5; i > 0; i--)
    {
        ESP_LOGI(LTE_TAG, "Restarting in %d seconds ... ", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
    esp_restart();
}

/**
 * @brief Function that performs the power up sequence of LTE.
 * @param none
 * @retval none
 * @warning Logic is Inverted
 */
void powerUpLTE()
{
    gpio_set_level(LTE_POWER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LTE_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2500));
    gpio_set_level(LTE_POWER_PIN, 0);
    ESP_LOGI(LTE_TAG, " LTE Power-Up Sequence Done");
}

/**
 * @brief configure esp32 uart
 * @param None
 * @retval None
 */
void lte_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // uart_driver_install(UART_NUM_1, UART_BUFFER_LEN, UART_BUFFER_LEN, 10, &uart_event_queue, 0);
    uart_driver_install(UART_NUM_1, UART_BUFFER_LEN, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief Thread that takes care of Establishing and Maintaining the LTE communication
 * @param args
 */
void lte_task(void *args)
{
    publish_queue = xQueueCreate(PUBLISH_QUEUE_SIZE, sizeof(char *));
    command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(CommandStruct));

    initialize_mqtt_cmd_strings();
    lte_gpio_configuration();
    powerUpLTE();
    lte_uart_init();
    execute_general_AT_cmds();
    MQTT_config();
    while (1)
    {
        maintainMQTTConnection();
        if (powerDownFlag)
            powerCycleDevice(DUE_TO_BUTTON_PRESS);
        powerUpLTE();
        MQTT_config();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Function that takes care of handling the sending of AC commands and
 * storing the sent command to flash. Common function for both provisioner and node
 * @param cmd_struct
 */
void handle_ac_control(CommandStruct *cmd)
{
    last_command = *cmd;
    ir_transmit();
    set_last_ac_cmd_in_nvs_flash();
}