/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <main.h>
#include <general.h>
#include <button.h>
#include <led.h>
#include <lte.h>
#include <heartbeat.h>
#include <flash.h>
#include <ir.h>
#include <ble_new.h>
#include <temperature_sensor.h>
#include "esp_timer.h"  

#define TAG "MAIN"

TaskHandle_t button_task_handle = NULL;
TaskHandle_t lte_task_handle = NULL;
TaskHandle_t ir_recv_task_handle = NULL;
TaskHandle_t health_monitor_task_handle = NULL;

/*Global Flags/Variables Initialization*/
uint8_t newDevice = 1;
char serialNoStr[SERIAL_NO_LEN];
char alive_msg[ALIVE_MSG_LEN];
bool mqtt_connected = 0;
uint8_t registered = 0;
uint8_t provisioned = 0;
uint8_t configured = 0;
bool sending_ir_command = false;
bool teaching_in_progress = false;
int16_t ir_protocol_num = -1;
char ir_protocol[IR_PROTOCOL_NAME_LEN];
uint16_t publishPeriod = DEFAULT_PUBLISH_PERIOD_SEC;
CommandStruct last_command;
char device_location_str[LOCATION_STR_LEN];
uint16_t teaching_mode_raw_len = 0;

uint16_t teachingModeIrCmds[MAX_CMDS_IN_TEACHING_MODE][TEACHING_MODE_CDM_LEN];

/**
 * @brief Function that prints basic information about the device after fetching info from nvs flash
 */
void print_basic_info()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGW(TAG, "=-=-=-=-=-=-=-=-=-=- BOOT SUCCESSFUL - %d.%d.%d -=-=-=-=-=-=-=-=-=-=", MAJ_VERSION, MIN_VERSION, PATCH_VERSION);
    ESP_LOGI(TAG, "%s : %s - %s", "Compiled Date and Time", __DATE__, __TIME__);
    ESP_LOGI(TAG, "%s : %d", NVS_NEW_DEVICE_KEY, newDevice);
    ESP_LOGI(TAG, "%s : %s", NVS_DEVICE_LOCATION_KEY, device_location_str);
    ESP_LOGI(TAG, "%s : %s", NVS_SERIAL_NO_KEY, serialNoStr);
#if (IS_GWY)
    ESP_LOGI(TAG, "%s : %d", NVS_REGISTERED_KEY, registered);
    ESP_LOGI(TAG, "%s : %s", "MQTT Publish Topic", publish_topic);
    ESP_LOGI(TAG, "%s : %s", "MQTT Subscribe Topic", subscribe_topic);
    ESP_LOGI(TAG, "%s : %s", "MQTT Alive Topic", alive_topic);
    ESP_LOGI(TAG, "%s : %s", "MQTT Will Topic", will_topic);
#else
    ESP_LOGI(TAG, "%s : %d", NVS_PROVISIONED_KEY, provisioned);
#endif
    ESP_LOGI(TAG, "%s : %d", NVS_CONFIGURED_KEY, configured);
    ESP_LOGI(TAG, "%s : %ds", "Heartbeat Interval", publishPeriod);
    ESP_LOGI(TAG, "%s : %s", NVS_IR_PROTOCOL_KEY, ir_protocol);
    if (strcmp(ir_protocol, RAW_IR_PROTOCOL) == 0)
    {
        ESP_LOGI(TAG, "%s : %d", NVS_RAWLEN_KEY, teaching_mode_raw_len);
    }
    ESP_LOGW(TAG, "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
    ESP_LOGW(TAG, "LAST AC-SETTINGS:");
    ESP_LOGI(TAG, "%s : %d", NVS_POWER_KEY, last_command.power);
    ESP_LOGI(TAG, "%s : %d", NVS_TEMPERATURE_KEY, last_command.temperature);
    ESP_LOGI(TAG, "%s : %d", NVS_FANSPEED_KEY, last_command.fanspeed);
    ESP_LOGI(TAG, "%s : %d", NVS_SWINGH_KEY, last_command.swingh);
    ESP_LOGI(TAG, "%s : %d", NVS_SWINGV_KEY, last_command.swingv);
    ESP_LOGI(TAG, "%s : %s", NVS_MODE_KEY, last_command.mode_str);
    ESP_LOGI(TAG, "%s : %d", NVS_ONTIMER_KEY, last_command.ontimer);
    ESP_LOGI(TAG, "%s : %d", NVS_OFFTIMER_KEY, last_command.offtimer);
    ESP_LOGI(TAG, "%s : %d", NVS_LOCKING_KEY, last_command.locking);
    ESP_LOGI(TAG, "%s : %d", NVS_UPPER_TEMPERATURE_LIMIT_KEY, last_command.upperTemperatureLimit);
    ESP_LOGI(TAG, "%s : %d", NVS_LOWER_TEMPERATURE_LIMIT_KEY, last_command.lowerTemperatureLimit);
    ESP_LOGW(TAG, "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
    ESP_LOGW(TAG, "%s : %d", "Command Struct size", sizeof(CommandStruct));
    ESP_LOGW(TAG, "%s : %d", "Teaching Mode Struct size", sizeof(teaching_mode));
    ESP_LOGW(TAG, "%s : %d", "Manual Control Struct size", sizeof(manual_control));
}

/**
 * @brief Function that constructs the MQTT Topic strings and Messages
 */
void construct_topics_and_msgs()
{
    sprintf(will_msg, "%s disconnected unexpectedly", serialNoStr);
    sprintf(alive_msg, "%s is alive", serialNoStr);
    sprintf(subscribe_topic, "%s/command", serialNoStr);
    sprintf(publish_topic, "%s/message", serialNoStr);
    sprintf(alive_topic, "%s/alive", serialNoStr);
    sprintf(will_topic, "GWYS/will");
}

#if (IS_GWY)
/**
 * @brief Periodic health monitor task — logs heap, queue and task stack
 *        high water marks every 5 minutes to help diagnose memory leaks
 *        and task stack overflows (e.g. the 40-hour freeze issue)
 */
#define HEALTH_MONITOR_INTERVAL_MS  (5 * 60 * 1000)  // 5 minutes
#define HEALTH_TAG "HEALTH"

void health_monitor_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(HEALTH_MONITOR_INTERVAL_MS));

        ESP_LOGW(HEALTH_TAG, "========== HEALTH CHECK ==========");

        // Heap status
        ESP_LOGW(HEALTH_TAG, "Free heap        : %" PRIu32 " bytes", esp_get_free_heap_size());
        ESP_LOGW(HEALTH_TAG, "Min free heap    : %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

        // Queue states
        ESP_LOGW(HEALTH_TAG, "Publish queue    : %d / %d items", 
            uxQueueMessagesWaiting(publish_queue), PUBLISH_QUEUE_SIZE);
        ESP_LOGW(HEALTH_TAG, "Command queue    : %d / %d items",
            uxQueueMessagesWaiting(command_queue), COMMAND_QUEUE_SIZE);

        // Task stack high water marks (minimum free stack words remaining)
        // If this approaches 0, stack overflow is imminent
        ESP_LOGW(HEALTH_TAG, "LTE task HWM     : %d words", 
            uxTaskGetStackHighWaterMark(lte_task_handle));
        ESP_LOGW(HEALTH_TAG, "IR task HWM      : %d words",
            uxTaskGetStackHighWaterMark(ir_recv_task_handle));
        ESP_LOGW(HEALTH_TAG, "Button task HWM  : %d words",
            uxTaskGetStackHighWaterMark(button_task_handle));
        ESP_LOGW(HEALTH_TAG, "Health task HWM  : %d words",
            uxTaskGetStackHighWaterMark(health_monitor_task_handle));

        // MQTT connection state
        ESP_LOGW(HEALTH_TAG, "MQTT connected   : %d", mqtt_connected);
        ESP_LOGW(HEALTH_TAG, "Uptime           : %" PRIu32 " seconds", 
            (uint32_t)(esp_timer_get_time() / 1000000));

        ESP_LOGW(HEALTH_TAG, "==================================");
    }
}
#endif

/**
 * @brief Entry point for the entire application
 */
void app_main(void)
{
    nvs_init();

#if (IS_GWY)
    construct_topics_and_msgs();
#else
    ble_init();
#endif

    print_basic_info();
    temperature_sensor_init();
    hb_init();
    led_init();
    gpio_install_isr_service(0);
    button_intr_init();
    ir_tran_setup();

#if (IS_GWY)
    xTaskCreate(lte_task, "LTE Task", LTE_THREAD_STACK_SIZE, NULL, 2, &lte_task_handle);
    xTaskCreate(health_monitor_task, "Health Monitor", 4096, NULL, 1, &health_monitor_task_handle);
#endif
    if(registered || provisioned) xTaskCreate(ir_recv_task, "IR Recv Task", IR_THREAD_STACK_SIZE, NULL, 2, &ir_recv_task_handle);
}
