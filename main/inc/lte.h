#ifndef LTE_H
#define LTE_H

#include <stdint.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define PUBLISH_QUEUE_SIZE 10
#define COMMAND_QUEUE_SIZE 10
#define BLE_RESP_QUEUE_SIZE 50

#define BAUD_RATE 115200

#define LTE_TAG "LTE"

#define LTE_RESET_PIN 46
#define LTE_POWER_PIN 9
#define TXD_PIN 17
#define RXD_PIN 18
#define CTS_PIN 11
#define RTS_PIN 10

#define MIN_LTE_RESP_WAIT_MS 100

#define RETRY_COUNT 5
#define NETWORK_CHECK_INTERVAL_TICKS pdMS_TO_TICKS(60000) //60s once

/*MQTT Configuration parameters*/
#define MQTT_CLIENT_INDEX 2
#define WILL_MSG_LEN 100
#define MQTT_MSG_ID 2
#define MQTT_QOS 2
#define MQTT_WILL_QOS 2
#define MQTT_WILL_RETAIN 1
#define MQTT_WILL_FLAG 1
#define MQTT_KEEP_ALIVE_S 10

#define DEFAULT_PUBLISH_PERIOD_SEC 300

/*Sizes and Lengths*/
#define SERIAL_NO_LEN 16
#define ALIVE_MSG_LEN 30
#define IR_PROTOCOL_NAME_LEN 16
#define LOCATION_STR_LEN 20
#define MODE_STR_LEN 8
#define MQTT_TOPIC_NAME_LEN 50
#define MQTT_CMD_RESP_LEN 200
#define UART_BUFFER_LEN 800
#define MQTT_ACK_BUFFER_LEN 1024

#define DEFAULT_DEVICE_SER_NO "DEV99999"
#define DEFAULT_DEVICE_LOCATION_STR "EARTH :)"

#define BUTTON_PRESS_MSGSEQNO 9999

extern char will_msg[WILL_MSG_LEN];

extern const char *JSON_PACKET_ID_KEY;
extern const char *JSON_ACK_NAME_KEY;
extern const char *MSG_SEQ_NO_KEY;
extern const char *GWY_SER_NO_KEY;
extern const char *NODE_SER_NO_KEY;
extern const char *LOCATION_KEY;
extern const char *APP_KEY_INDEX;
extern const char *APP_KEY;
extern const char *NET_KEY_INDEX;
extern const char *NET_KEY;
extern const char *ELEMENT_ADDR_KEY;
extern const char *MAC_ID_KEY;
extern const char *MODE_KEY;
extern const char *POWER_KEY;
extern const char *FAN_SPEED_KEY;
extern const char *TEMPERATURE_KEY;
extern const char *SWING_H_KEY;
extern const char *SWING_V_KEY;
extern const char *ONTIMER_KEY;
extern const char *OFFTIMER_KEY;
extern const char *AC_LOCKING_KEY;
extern const char *UPPER_TEMPERATURE_LIMIT_KEY;
extern const char *LOWER_TEMPERATURE_LIMIT_KEY;
extern const char *ERROR_CODE_KEY;
extern const char *ERROR_MSG_KEY;
extern const char *AMBIENT_TEMPERATURE_DIGITAL_DATA_KEY;
extern const char *AMBIENT_TEMPERATURE_ANALOG_DATA_KEY;
extern const char *PUBLISH_PERIOD_KEY;
extern const char *FIRMWARE_VERSION_KEY;
extern const char *REGISTERED_KEY;
extern const char *PROTOCOL_SEL_NUM_KEY;
extern const char *PUBLISH_MESG_QUEUE_COUNT_KEY;
extern const char *PROV_QUEUE_COUNT_KEY;
extern const char *UNPROV_QUEUE_COUNT_KEY;
extern const char *AC_CONTROL_QUEUE_COUNT_KEY;
extern const char *RECONF_QUEUE_COUNT_KEY;
extern const char *PUB_CONF_QUEUE_COUNT_KEY;
extern const char *TEACHING_MODE_QUEUE_COUNT_KEY;
extern const char *DEBUG_INFO_QUEUE_COUNT_KEY;
extern const char *DEVICE_UPTIME_KEY;
extern const char *RESTART_DEVICE_KEY;
extern const char *RESET_DEVICE_KEY;
extern const char *LINK_KEY;
extern const char *TEACHING_START_KEY;
extern const char *STARTING_TEMPERATURE_KEY;
extern const char *ENDING_TEMPERATURE_KEY;
extern const char *POWER_ERROR_KEY;
extern const char *TEMPERATURE_ERROR_KEY;
extern const char *FANSPEED_ERROR_KEY;
extern const char *MODE_ERROR_KEY;
extern const char *LAST_CMD_KEY;
extern const char *NEXT_CMD_KEY;
extern const char *REMAINING_CMD_KEY; 
extern const char *DETECTED_TEMPERATURE_KEY;
extern const char *BLE_ERROR_CODE_KEY;
extern const char *MESSAGE_KEY;
extern const char *RSSI_KEY;

extern char subscribe_topic[MQTT_TOPIC_NAME_LEN];
extern char publish_topic[MQTT_TOPIC_NAME_LEN];
extern char alive_topic[MQTT_TOPIC_NAME_LEN];
extern char will_topic[MQTT_TOPIC_NAME_LEN];

typedef enum
{
    INT8_SIZE,
    UINT8_SIZE,
    INT16_SIZE,
    UINT16_SIZE,
    INT32_SIZE,
    UINT32_SIZE,
    INT64_SIZE,
    UINT64_SIZE
}sizes_t;

typedef enum 
{
	// Basic
    QMTSTAT_1_ERRORCODE=-4,
    QMTOPEN_2_ERRORCODE=-3,
    QMTOPEN_3_ERRORCODE=-2,
	FAILURE = -1,
	SUCCESS,
    JSON_PACKET_INVALID,
	MISSING_PACKET_ID,
	PACKET_ID_EXCEEDING_RANGE,
	MISSING_MSG_SEQ_NO,
    MSG_SEQ_NO_EXCEEDING_RANGE,
	MISSING_LOCATION,
    LOCATION_EXCEEDING_RANGE,
    NODE_COMM_TIMEOUT,
    GWY_NOT_REG,
    GWY_ALREADY_REG,
    MISSING_MACID,
    INVALID_MACID,
    MISSING_ELEMENT_ADDR,
    ELEMENT_ADDR_EXCEEDING_RANGE,
    DEVICE_NOT_CONFIGURED_WITH_AC_REMOTE,
    MISSING_POWER,
    POWER_EXCEEDING_RANGE,
    MISSING_MODE,
    MODE_EXCEEDING_RANGE,
    MISSING_FAN_SPEED,
    FAN_SPEED_EXCEEDING_RANGE,
    MISSING_TEMPERATURE,
    TEMPERATURE_EXCEEDING_RANGE,
    MISSING_SWINGH,
    SWINGH_EXCEEDING_RANGE,
    MISSING_SWINGV,
    SWINGV_EXCEEDING_RANGE,
    MISSING_ONTIMER,
    ONTIMER_EXCEEDING_RANGE,
    MISSING_OFFTIMER,
    OFFTIMER_EXCEEDING_RANGE,
    MISSING_LOCKING,
    LOCKING_EXCEEDING_RANGE,
    MISSING_TEMPERATURE_UPPER_LIMIT,
    TEMPERATURE_UPPER_LIMIT_EXCEEDING_RANGE,
    MISSING_TEMPERATURE_LOWER_LIMIT,
    TEMPERATURE_LOWER_LIMIT_EXCEEDING_RANGE,
    INVALID_TEMPERATURE_LOCKING_LIMITS,
    MISSING_PUBLISH_PERIOD,
    PUBLISH_PERIOD_EXCEEDING_RANGE,
    MISSING_RESET_DEVICE,
    RESET_DEVICE_EXCEEDING_RANGE,
    MISSING_RESTART_DEVICE,
    RESTART_DEVICE_EXCEEDING_RANGE,
    IR_PROTOCOL_DECODEABLE_ONLY,
    IR_PROTOCOL_FULLY_UNSUPPORTED,
    MISSING_ERROR_CHECK,
    ERROR_CHECK_EXCEEDING_RANGE,
    MISSING_TEACHING_START,
    TEACHING_START_EXCEEDING_RANGE,
    MISSING_STARTING_TEMPERATURE,
    STARTING_TEMPERATURE_EXCEEDING_RANGE,
    MISSING_ENDING_TEMPERATURE,
    ENDING_TEMPERATURE_EXCEEDING_RANGE,
    STARTING_TEMPERATURE_LESS_THAN_ENDING_TEMPERATURE,
    ENTERED_TEACHING_MODE,
    EXITED_TEACHING_MODE,
    DEVICE_ALREADY_IN_TEACHING_MODE,
    DEVICE_NOT_IN_TEACHING_MODE,
    POWER_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING,
    MODE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING,
    FANSPEED_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING,
    TEMPERATURE_NOT_AVAILABLE_IN_IR_SIGNAL_DECODED_STRING,
    IR_CMD_NOT_AVAILABLE_IN_FLASH,
    NODE_NOT_FOUND_IN_PROVISIONER_DATABASE,
    NODE_NOT_PROVISIONED,
    IR_TASK_CREATION_FAILED,
    ENQUEUING_INTO_COMMAND_QUEUE_FAILED,
    DEVICE_DATA_ERASURE_SUCCESSFUL,
    DEVICE_DATA_ERASURE_FAILED,
    IR_PARTITION_INIT_FAILED,
    POWER_INVALID_FORMAT,
    TEMPERATURE_INVALID_FORMAT,
    FAN_SPEED_INVALID_FORMAT,
    MODE_INVALID_FORMAT,
    SWINGH_INVALID_FORMAT,
    SWINGV_INVALID_FORMAT,
    LOCKING_INVALID_FORMAT,
    ONTIMER_INVALID_FORMAT,
    OFFTIMER_INVALID_FORMAT,
    TEMPERATURE_UPPER_LIMIT_INVALID_FORMAT,
    TEMPERATURE_LOWER_LIMIT_INVALID_FORMAT,
    PACKETID_INVALID_FORMAT,
    MSG_SEQ_NO_INVALID_FORMAT,
    ELEMENT_ADDR_INVALID_FORMAT,
    LOCATION_INVALID_FORMAT,
    PUBLISH_PERIOD_INVALID_FORMAT,
    MACID_INVALID_FORMAT,
    TEACHING_START_INVALID_FORMAT,
    STARTING_TEMPERATURE_INVALID_FORMAT,
    ENDING_TEMPERATURE_INVALID_FORMAT,
    RESTART_INVALID_FORMAT,
    RESET_INVALID_FORMAT,
}error_codes;

typedef enum
{
	/* GWY PACKETS */
	GWY_REG_PACKET,
	GWY_CONF_ACK,
	GWY_UNREG_PACKET,
	GWY_AC_CONTROL_PACKET,
	GWY_MANUAL_AC_CONTROL_ACK,
	GWY_RECONF_PACKET,
	GWY_HEARTBEAT_ACK,
	GWY_HEARTBEAT_PUB_CONF_PACKET,
	GWY_TEACHING_MODE,
    GWY_TEACHING_MODE_CMD_SELECTION_PACKET,
	GWY_DEBUG_INFO_PACKET,
	MAX_GWY_PACKET_ID,
    GWY_GENERAL_PACKET, 
    GWY_AC_CONTROL_ACK,        // 13 — ACK sent to cloud after GWY AC control

	/* NODE PACKETS */
	NODE_PROV_PACKET = 100,
	NODE_CONF_ACK,
	NODE_UNPROV_PACKET,
	NODE_AC_CONTROL_PACKET,
	NODE_MANUAL_AC_CONTROL_ACK,
	NODE_RECONF_PACKET,
	NODE_HEARTBEAT_ACK,
	NODE_HEARTBEAT_PUB_CONF_PACKET,
	NODE_TEACHING_MODE,
    NODE_TEACHING_MODE_CMD_SELECTION_PACKET,
	NODE_DEBUG_INFO_PACKET,
	MAX_NODE_PACKET_ID,
    NODE_GENERAL_PACKET,
    NODE_AC_CONTROL_ACK,       // 113 — ACK sent to cloud after NODE AC control
    
    /*MISC*/
    TEST_PACKET = 999
}mqtt_packets;

typedef enum
{
    DUE_TO_BUTTON_PRESS,
    DUE_TO_MQTT_CMD,
}action_type_t;

typedef struct
{
    TickType_t reqSentToNodeTicks;     // 4 bytes
    esp_err_t bleErrorCode;            // 4 bytes
    float deviceUpTimeHrs;             // 4 bytes
    uint16_t packetid;                 // 2 bytes
    int16_t irProtocolNum;             // 2 bytes
    uint16_t msgseqno;                 // 2 bytes
    uint16_t elemaddr;                 // 2 bytes
    uint16_t ontimer;                  // 2 bytes
    uint16_t offtimer;                 // 2 bytes
    uint16_t publishPeriodSec;         // 2 bytes
    int16_t ambientTemperatureDigital; // 2 bytes
    int16_t ambientTemperatureAnalog;  // 2 bytes
    error_codes errorcode;             // 1 byte
    uint8_t power;                     // 1 byte
    uint8_t temperature;               // 1 byte
    uint8_t fanspeed;                  // 1 byte
    uint8_t mode_num;                  // 1 byte
    uint8_t swingh;                    // 1 byte
    uint8_t swingv;                    // 1 byte
    uint8_t locking;                   // 1 byte
    uint8_t upperTemperatureLimit;     // 1 byte
    uint8_t lowerTemperatureLimit;     // 1 byte
    uint8_t resetDevice;               // 1 byte
    uint8_t restartDevice;             // 1 byte
    uint8_t logging;                   // 1 byte
    uint8_t teachingStart;             // 1 byte
    uint8_t startingTemperature;       // 1 byte
    uint8_t endingTemperature;         // 1 byte
    uint8_t majversion;                // 1 byte
    uint8_t minversion;                // 1 byte
    uint8_t patchversion;              // 1 byte
    uint8_t provisioned;               // 1 byte
    uint8_t errorCheckEnabled;         // 1 Byte
    int8_t rssi;
    bool requestSentToNode;            // 1 byte
    bool configured;                   // 1 byte
    char mode_str[MODE_STR_LEN];       // Variable size, but align at the end
    char deviceName[SERIAL_NO_LEN];    // 16 bytes
} CommandStruct;


typedef struct
{
    uint16_t packetid;                // 2 bytes
    uint16_t elemAddr;                // 2 bytes
    error_codes power_err;            // 1 byte
    error_codes fanspeed_err;         // 1 byte
    error_codes mode_err;             // 1 byte
    error_codes temperature_err;      // 1 byte
    int8_t power_value;               // 1 byte
    int8_t fanspeed_value;            // 1 byte
    int8_t temperature_value;         // 1 byte
    char deviceName[SERIAL_NO_LEN];   // 16 bytes
    char mode[8];                     // 8 bytes
    char temperature[4];              // 4 bytes
    char power[4];                    // 4 bytes
    char fan[2];                      // 2 bytes
    // Added for currACState support
    int8_t rssi;                      // 1 byte
    int8_t swingh;                    // 1 byte
    int8_t swingv;                    // 1 byte
    int8_t ontimer;                   // 1 byte
    int8_t offtimer;                  // 1 byte
    int8_t locking;                   // 1 byte
    int8_t upperTemperatureLimit;     // 1 byte
    int8_t lowerTemperatureLimit;     // 1 byte
    int16_t ambientTemperatureDigital;// 2 bytes
    int16_t ambientTemperatureAnalog; // 2 bytes
} manual_control;


typedef struct
{
    uint16_t packetid;              // 2 bytes
    uint16_t elemAddr;              // 2 bytes
    uint16_t msgseqno;              // 2 bytes
    esp_err_t bleErrorCode;         // 4 bytes
    char lastCommand[36];           // 36 bytes
    char nextCommand[36];           // 36 bytes
    char deviceName[SERIAL_NO_LEN]; // 16 bytes
    error_codes errorCode;          // 1 byte
    uint8_t teachingStart;          // 1 byte
    uint8_t power;                  // 1 byte
    uint8_t temperature;            // 1 byte
    uint8_t startingTemperature;    // 1 byte
    uint8_t endingTemperature;      // 1 byte
    uint8_t expectedTemperature;    // 1 byte
    uint8_t commandsReceived;       // 1 byte
    uint8_t commandIndex;           // 1 byte
    uint8_t remainingCommands;      // 1 byte
    uint8_t errorCheckEnabled;      // 1 byte
} teaching_mode;

typedef struct {
    uint16_t packetid;
    char *msg;
}general_purpose_t;

extern CommandStruct last_command;
extern manual_control ac_manual_control_t;
extern teaching_mode teaching_mode_t;
extern QueueHandle_t command_queue;
extern char node_macid[18];

/*Global Variables*/
extern TaskHandle_t lte_task_handle;
extern QueueHandle_t publish_queue;
extern QueueHandle_t command_queue;
/*Global Variables*/
extern char serialNoStr[SERIAL_NO_LEN];
extern char alive_msg[ALIVE_MSG_LEN];
extern bool mqtt_connected;
extern uint8_t registered;
extern uint8_t provisioned;
extern uint8_t configured;
extern bool sending_ir_command;
extern bool teaching_in_progress;
extern char ir_protocol[IR_PROTOCOL_NAME_LEN];
extern uint16_t publishPeriod;
extern char device_location_str[LOCATION_STR_LEN];
extern uint16_t teaching_mode_raw_len;
extern int16_t ir_protocol_num;
extern uint8_t newDevice;
extern bool powerDownFlag;

/*Function Declarations*/
void lte_task(void *args);
void uart_event_task(void *pvParameters);
error_codes check_response(char *uart_data, const char *check_string);
error_codes fetch_and_check_data(bool logging, uint16_t timeout_ms, const char *check_string, const char *cmd_name);
error_codes send_cmd_and_check_response(bool logging, const char *cmd, const char *cmdName, const char *check_string, uint32_t timeout_ms);
void powerUpLTE();
void powerCycleDevice(action_type_t type);
bool removeQueueItemByMsgSeqNo(QueueHandle_t queue, uint16_t msgseqno);
char* get_error_code_name(error_codes code);
void handle_ac_control(CommandStruct *cmd_struct);
void register_gwy();
void unregister(uint16_t msgseqno);
void generate_and_publish_debug_info_ack(CommandStruct *ack);

#ifdef __cplusplus
extern "C" {
#endif

void generate_ack(mqtt_packets packetid, CommandStruct *cmd_struct);
void construct_general_ack(error_codes err);
void generate_node_teaching_mode_ack(teaching_mode *node_teaching_mode_t);
void generate_node_manual_ac_control_ack(manual_control *node_ac_manual_control_t);
void enqueue_for_publish(char *ack_json);

#ifdef __cplusplus
}
#endif

#endif //LTE_H