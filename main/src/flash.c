#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"


#include <main.h>
#include <lte.h>
#include <flash.h>
#include <ir.h>
#include <led.h>
#include <ble_new.h>
#include <group_table.h>

#define NVS_TAG "NVS"

nvs_handle_t ir_nvs_handle;
nvs_handle_t general_nvs_handle;
nvs_handle_t serial_num_nvs_handle;
nvs_handle_t group_nvs_handle; 

/**
 * @warning The keys below are used by nvs. They should not be more than 15 chars
 */
const char *NVS_NEW_DEVICE_KEY = "NewDevice";
const char *NVS_SERIAL_NO_KEY = "SerialNo";
const char *NVS_DEVICE_LOCATION_KEY = "Location";
const char *NVS_REGISTERED_KEY = "Registered";
const char *NVS_CONFIGURED_KEY = "Configured";
const char *NVS_PROVISIONED_KEY = "Provisioned";
const char *NVS_RAWLEN_KEY = "RawLen";
const char *NVS_IR_PROTOCOL_KEY = "IrProtocol";
const char *NVS_POWER_KEY = "Power";
const char *NVS_TEMPERATURE_KEY = "Temperature";
const char *NVS_FANSPEED_KEY = "FanSpeed";
const char *NVS_MODE_KEY = "Mode";
const char *NVS_SWINGH_KEY = "SwingH";
const char *NVS_SWINGV_KEY = "SwingV";
const char *NVS_LOCKING_KEY = "Locking";
const char *NVS_ONTIMER_KEY = "OnTimer";
const char *NVS_OFFTIMER_KEY = "OffTimer";
const char *NVS_UPPER_TEMPERATURE_LIMIT_KEY = "UTL";
const char *NVS_LOWER_TEMPERATURE_LIMIT_KEY = "LTL";
const char *NVS_PUBPERIOD_KEY = "PubPeriod";
const char *NVS_LAST_COMMAND_KEY = "LastCommand";
const char *NVS_TEACHING_MODE_STARTING_TEMPERATURE_KEY = "StartTemp";
const char *NVS_TEACHING_MODE_ENDING_TEMPERATURE_KEY = "EndTemp";

const char *NVS_TEACHING_MODE_CMD_KEYS[] = {
    "CMD1",
    "CMD2",
    "CMD3",
    "CMD4",
    "CMD5",
    "CMD6",
    "CMD7",
    "CMD8",
    "CMD9",
    "CMD10",
    "CMD11",
    "CMD12",
    "CMD13",
    "CMD14",
    "CMD15",
    "CMD16"};

/**
 * @brief Function that resets the device as a new device
 */
error_codes factory_reset_device()
{
    esp_err_t err = ESP_OK;
    err = nvs_flash_erase_partition(GENERAL_NVS_PARTITION_NAME);
    if (err)
    {
        ESP_LOGE(NVS_TAG, "Erasing General NVS partition failed : %s", esp_err_to_name(err));
        goto here;
    }
    err = nvs_flash_erase_partition(IR_NVS_PARTITION_NAME);
    if (err)
    {
        ESP_LOGE(NVS_TAG, "Erasing IR NVS partition failed : %s", esp_err_to_name(err));
        goto here;
    }
    err = nvs_flash_erase_partition(GROUP_NVS_PARTITION_NAME);
    if (err)
    {
        ESP_LOGE(NVS_TAG, "Erasing Group NVS partition failed : %s", esp_err_to_name(err));
        goto here;
    }

    err = nvs_flash_erase(); // Clears off the BLE Partition
    if (err)
    {
        ESP_LOGE(NVS_TAG, "Erasing BLE NVS partition failed : %s", esp_err_to_name(err));
        goto here;
    }
    else ESP_LOGW(NVS_TAG, "Flash data erased succuessfully");


    here:
    if(err) {
        err = DEVICE_DATA_ERASURE_FAILED;
        led_set_state(LED_STATE_INVALID_OPERATION);
    }
    return err; 
}

/**
 * @brief Get the last ac cmd from nvs flash
 */
void get_last_ac_cmd_in_nvs_flash()
{
    size_t size = sizeof(CommandStruct);
    esp_err_t err = nvs_get_blob(ir_nvs_handle, NVS_LAST_COMMAND_KEY, &last_command, &size);
    if(err) ESP_LOGE(NVS_TAG, "Failed to get %s from nvs flash : %s", NVS_LAST_COMMAND_KEY, esp_err_to_name(err));
}

/**
 * @brief Function that saves a blob to nvs flash
 *
 * @param nvshandle
 * @param key
 */
void set_last_ac_cmd_in_nvs_flash()
{
    esp_err_t err;
    err = nvs_set_blob(ir_nvs_handle, NVS_LAST_COMMAND_KEY, &last_command, sizeof(CommandStruct));
    if(err) ESP_LOGE(NVS_TAG, "Failed to set %s in nvs flash : %s", NVS_LAST_COMMAND_KEY, esp_err_to_name(err));
    err = nvs_commit(ir_nvs_handle);
    if(err) ESP_LOGE(NVS_TAG, "Failed to Commit - %s : %s", __func__, esp_err_to_name(err));
}

/**
 * @brief Function that saves a number to nvs flash
 * @param handle
 * @param key
 * @param value
 */
void set_number_in_nvs_flash(handle_enum_t nvshandle, const char *key, int value, sizes_t size)
{
    esp_err_t err;
    nvs_handle_t handle;
    switch (nvshandle)
    {
    case IR_HANDLE:
        handle = ir_nvs_handle;
        break;
    case GENERAL_HANDLE:
        handle = general_nvs_handle;
        break;
    case GROUP_HANDLE:
        handle = group_nvs_handle;
        break;
    default:
        return;
    }

    switch (size)
    {
    case INT8_SIZE:
        err = nvs_set_i8(handle, key, (int8_t)value);
        if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d in nvs flash : %s", key, value, esp_err_to_name(err));
        break;

    case UINT8_SIZE:
        err = nvs_set_u8(handle, key, (uint8_t)value);
        if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d in nvs flash : %s", key, value, esp_err_to_name(err));
        break;

    case INT16_SIZE:
        err = nvs_set_i16(handle, key, (int16_t)value);
        if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d in nvs flash : %s", key, value, esp_err_to_name(err));
        break;

    case UINT16_SIZE:
        err = nvs_set_u16(handle, key, (uint16_t)value);
        if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d in nvs flash : %s", key, value, esp_err_to_name(err));
        break;

    case INT32_SIZE:
        err = nvs_set_i32(handle, key, (int32_t)value);
        if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d in nvs flash : %s", key, value, esp_err_to_name(err));
        break;

    case UINT32_SIZE:
        err = nvs_set_u32(handle, key, (uint32_t)value);
        if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d in nvs flash : %s", key, value, esp_err_to_name(err));
        break;

    case INT64_SIZE:
        break;

    case UINT64_SIZE:
        break;

    default:
        break;
    }

    err = nvs_commit(handle);
    if(err) ESP_LOGE(NVS_TAG, "Failed to Commit - %s : %s", __func__, esp_err_to_name(err));
}

/**
 * @brief Function that saves a string to nvs flash
 * @param handle
 * @param key
 * @param value
 */
void set_str_in_nvs_flash(handle_enum_t nvshandle, const char *key, char *value)
{
    esp_err_t err;
    nvs_handle_t handle;
    switch (nvshandle)
    {
    case IR_HANDLE:
        handle = ir_nvs_handle;
        break;
    case GENERAL_HANDLE:
        handle = general_nvs_handle;
        break;
    case GROUP_HANDLE:
        handle = group_nvs_handle;
        break;
    default:
        return;
    }

    err = nvs_set_str(handle, key, value);
    if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %s in nvs flash : %s", key, value, esp_err_to_name(err));

    err = nvs_commit(handle);
    if(err) ESP_LOGE(NVS_TAG, "Failed to Commit - %s : %s", __func__, esp_err_to_name(err));
}

/**
 * @brief Function that writes a blob of values to nvs flash
 * @param nvshandle
 * @param key
 * @param value
 */
void set_blob_in_nvs_flash(handle_enum_t nvshandle, const char *key, const void *value, size_t length)
{
    nvs_handle_t handle;
    switch (nvshandle)
    {
    case IR_HANDLE:
        handle = ir_nvs_handle;
        break;
    case GENERAL_HANDLE:
        handle = general_nvs_handle;
        break;
    case GROUP_HANDLE:
        handle = group_nvs_handle;
        break;
    default:
        return;
    }
    nvs_set_blob(handle, key, value, length);
    esp_err_t err = nvs_commit(handle);
    if(err) ESP_LOGE(NVS_TAG, "Failed to Commit - %s : %s", __func__, esp_err_to_name(err));
}

/**
 * @brief Function that gets a number from the nvs flash
 * @param handle
 * @param key
 * @return int32_t
 */
int32_t get_number_from_nvs_flash(handle_enum_t nvshandle, const char *key, sizes_t size)
{
    switch (size)
    {
    case INT8_SIZE:
        break;

    case UINT8_SIZE:
        break;

    case INT16_SIZE:
        break;

    case UINT16_SIZE:
        break;

    case INT32_SIZE:
        break;

    case UINT32_SIZE:
        break;

    case INT64_SIZE:
        break;

    case UINT64_SIZE:
        break;

    default:
        break;
    }
    return 0;
}

/**
 * @brief Function that gets a string from the nvs flash
 * @param handle
 * @param key
 * @return char*
 */
char *get_str_from_nvs_flash(handle_enum_t nvshandle, const char *key)
{
    return "";
}

/**
 * @brief Funtion that gets a blob of values from the nvs flash
 * @param nvshandle
 * @param key
 * @param out_value
 * @param length
 */
void get_blob_from_nvs_flash(handle_enum_t nvshandle, const char *key, void *out_value, size_t *length)
{
    nvs_handle_t handle;
    switch (nvshandle)
    {
    case IR_HANDLE:
        handle = ir_nvs_handle;
        break;
    case GENERAL_HANDLE:
        handle = general_nvs_handle;
        break;
    case GROUP_HANDLE:
        handle = group_nvs_handle;
        break;
    default:
        return;
    }
    esp_err_t err = nvs_get_blob(handle, key, out_value, length);
    if(err) ESP_LOGE(NVS_TAG, "Failed to get %s from nvs flash - %s", key, esp_err_to_name(err));
}

/**
 * @brief Function that initializes the nvs flash with defaults
 * @return esp_err_t
 */
esp_err_t init_data_in_nvs(void)
{
    esp_err_t err;
    
    err = nvs_set_u8(general_nvs_handle, NVS_NEW_DEVICE_KEY, 0);
    if(err) ESP_LOGE(NVS_TAG, "Failed to set %s as %d : %s", NVS_NEW_DEVICE_KEY, 0, esp_err_to_name(err));

    err = nvs_commit(ir_nvs_handle);
    if(err) ESP_LOGE(NVS_TAG, "Failed to Commit %s - %s : %s", IR_NVS_NAMESPACE, __func__, esp_err_to_name(err));
    err = nvs_commit(general_nvs_handle);
    if(err) ESP_LOGE(NVS_TAG, "Failed to Commit %s - %s : %s", GENERAL_NVS_NAMESPACE, __func__, esp_err_to_name(err));

    return ESP_OK;
}

/**
 * @brief Function that pulls the IR command data from flash and fills it in ram for access
 *
 */
void pull_ir_cmd_data()
{
    size_t size = (teaching_mode_raw_len)*sizeof(uint16_t);
    for (uint8_t i = 0; i < MAX_CMDS_IN_TEACHING_MODE; i++)
    {
        nvs_get_blob(ir_nvs_handle, NVS_TEACHING_MODE_CMD_KEYS[i], teachingModeIrCmds[i], &size);
    }
}

void print_ir_cmds()
{
    for (uint8_t i = 0; i < MAX_CMDS_IN_TEACHING_MODE; i++)
    {
        if (teaching_mode_raw_len == 0)
        {
            ESP_LOGE(NVS_TAG, "Raw len is zero");
            return;
        }
        ESP_LOGW(NVS_TAG, "IR Cmd #%d - %s", i, TEACHING_MODE_SEQUENCE[i]);
        for (uint16_t j = 0; j < teaching_mode_raw_len; j++)
        {
            printf("%d ", teachingModeIrCmds[i][j]);
        }
        printf("\n");
    }
}

/**
 * @brief Function that pulls data from NVS flash. If no data present,
 * meaning its a fresh device, then initializes the defaults to NVS
 * @return esp_err_t
 */
esp_err_t pull_data_from_nvs(void)
{
    nvs_get_u8(ir_nvs_handle, NVS_TEACHING_MODE_STARTING_TEMPERATURE_KEY, &teaching_mode_t.startingTemperature);
    nvs_get_u8(ir_nvs_handle, NVS_TEACHING_MODE_ENDING_TEMPERATURE_KEY, &teaching_mode_t.endingTemperature);

    nvs_get_i16(ir_nvs_handle, NVS_IR_PROTOCOL_KEY, &ir_protocol_num);
    strcpy(ir_protocol, get_protocol_string(ir_protocol_num));

    if (strcmp(ir_protocol, RAW_IR_PROTOCOL) == 0)
    {
        nvs_get_u16(ir_nvs_handle, NVS_RAWLEN_KEY, &teaching_mode_raw_len);
        pull_ir_cmd_data();
        print_ir_cmds();
    }

    size_t size = sizeof(serialNoStr);
    nvs_get_str(serial_num_nvs_handle, NVS_SERIAL_NO_KEY, serialNoStr, &size);
    nvs_get_u8(general_nvs_handle, NVS_REGISTERED_KEY, &registered);
    nvs_get_u8(general_nvs_handle, NVS_PROVISIONED_KEY, &provisioned);
    nvs_get_u8(general_nvs_handle, NVS_CONFIGURED_KEY, &configured);
    nvs_get_u16(general_nvs_handle, NVS_PUBPERIOD_KEY, &publishPeriod);

    if (configured)
        get_last_ac_cmd_in_nvs_flash();

    size_t locationReqSize = LOCATION_STR_LEN;
    nvs_get_str(general_nvs_handle, NVS_DEVICE_LOCATION_KEY, device_location_str, &locationReqSize);

    return ESP_OK;
}

/**
 * @brief Function that checks if a device is brand new
 * @return true
 * @return false
 */
bool isNewDevice()
{
    esp_err_t err = nvs_get_u8(general_nvs_handle, NVS_NEW_DEVICE_KEY, &newDevice);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(NVS_TAG, "NVS_NEW_DEVICE_KEY not found, assuming new device.");
        return true;
    }
    else if (err == ESP_OK)
    {
        return newDevice;
    }
    else
    {
        ESP_LOGE(NVS_TAG, "Error reading NVS_NEW_DEVICE_KEY: %s", esp_err_to_name(err));
        return false;
    }
}

/**
 * @brief Function that initializes nvs flash partitions.
 *
 */
void nvs_init()
{
    esp_err_t err;

    err = nvs_flash_init_partition(IR_NVS_PARTITION_NAME);
    if(err) ESP_LOGE(NVS_TAG, "Failed to init %s partition - %s", IR_NVS_PARTITION_NAME, esp_err_to_name(err));
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND || err == ESP_ERR_NVS_NOT_ENOUGH_SPACE)
    {
        ESP_ERROR_CHECK(nvs_flash_erase_partition(IR_NVS_PARTITION_NAME));
        err = nvs_flash_init_partition(IR_NVS_PARTITION_NAME);
    }

    err = nvs_flash_init_partition(GENERAL_NVS_PARTITION_NAME);
    if(err) ESP_LOGE(NVS_TAG, "Failed to init %s partition - %s", GENERAL_NVS_PARTITION_NAME, esp_err_to_name(err));
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND || err == ESP_ERR_NVS_NOT_ENOUGH_SPACE)
    {
        ESP_ERROR_CHECK(nvs_flash_erase_partition(GENERAL_NVS_PARTITION_NAME));
        err = nvs_flash_init_partition(GENERAL_NVS_PARTITION_NAME);
    }

    err = nvs_flash_init_partition(SERIALNUM_NVS_PARTITION_NAME);
    if(err) ESP_LOGE(NVS_TAG, "Failed to init %s partition - %s", SERIALNUM_NVS_PARTITION_NAME, esp_err_to_name(err));
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND || err == ESP_ERR_NVS_NOT_ENOUGH_SPACE)
    {
        ESP_ERROR_CHECK(nvs_flash_erase_partition(SERIALNUM_NVS_PARTITION_NAME));
        err = nvs_flash_init_partition(SERIALNUM_NVS_PARTITION_NAME);
    }

    // Group subscription partition — gateway only
    err = nvs_flash_init_partition(GROUP_NVS_PARTITION_NAME);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase_partition(GROUP_NVS_PARTITION_NAME));
        err = nvs_flash_init_partition(GROUP_NVS_PARTITION_NAME);
    }
    if (err) ESP_LOGE(NVS_TAG, "Failed to init %s partition - %s",
        GROUP_NVS_PARTITION_NAME, esp_err_to_name(err));

    err = nvs_open_from_partition(GROUP_NVS_PARTITION_NAME, GROUP_NVS_NAMESPACE,
        NVS_READWRITE, &group_nvs_handle);
    if (err) ESP_LOGE(NVS_TAG, "Opening %s Partition failed : %s",
        GROUP_NVS_PARTITION_NAME, esp_err_to_name(err));

    //Open the partitions to get their handles
    err = nvs_open_from_partition(IR_NVS_PARTITION_NAME, IR_NVS_NAMESPACE, NVS_READWRITE, &ir_nvs_handle);
    if(err) {
        ESP_LOGE(NVS_TAG, "Opening %s Partition failed : %s",IR_NVS_PARTITION_NAME,esp_err_to_name(err));
        return;
    }
    err = nvs_open_from_partition(GENERAL_NVS_PARTITION_NAME, GENERAL_NVS_NAMESPACE, NVS_READWRITE, &general_nvs_handle);
    if(err){
        ESP_LOGE(NVS_TAG, "Opening %s Partition failed : %s",GENERAL_NVS_PARTITION_NAME,esp_err_to_name(err));
        return;
    }
    err = nvs_open_from_partition(SERIALNUM_NVS_PARTITION_NAME, SERIALNUM_NVS_NAMESPACE, NVS_READONLY, &serial_num_nvs_handle);
    if(err){
        ESP_LOGE(NVS_TAG, "Opening %s Partition failed : %s",SERIALNUM_NVS_PARTITION_NAME,esp_err_to_name(err));
        return;
    }

    if (isNewDevice())
        ESP_LOGW(NVS_TAG, "Defaulting data in NVS flash : %s", esp_err_to_name(init_data_in_nvs()));
    pull_data_from_nvs();

    group_table_load();

    // Example of nvs_get_stats() to get overview of actual statistics of data entries :
    nvs_stats_t nvs_stats;

    nvs_get_stats(GENERAL_NVS_PARTITION_NAME, &nvs_stats);
    ESP_LOGI(NVS_TAG, "=-=-=-=-=-=-=-=-=-=- GENERAL_PARTITION_STATS -=-=-=-=-=-=-=-=-=-=");
    ESP_LOGI(NVS_TAG, "UsedEntries : (%d)", nvs_stats.used_entries);
    ESP_LOGI(NVS_TAG, "FreeEntries : (%d)", nvs_stats.free_entries);
    // ESP_LOGI(NVS_TAG, "AvailableEntries : (%lu)", nvs_stats.available_entries);
    ESP_LOGI(NVS_TAG, "AllEntries : (%d)", nvs_stats.total_entries);

    nvs_get_stats(IR_NVS_PARTITION_NAME, &nvs_stats);
    ESP_LOGI(NVS_TAG, "=-=-=-=-=-=-=-=-=-=- IR_PARTITION_STATS -=-=-=-=-=-=-=-=-=-=");
    ESP_LOGI(NVS_TAG, "UsedEntries : (%d)", nvs_stats.used_entries);
    ESP_LOGI(NVS_TAG, "FreeEntries : (%d)", nvs_stats.free_entries);
    // ESP_LOGI(NVS_TAG, "AvailableEntries : (%lu)", nvs_stats.available_entries);
    ESP_LOGI(NVS_TAG, "AllEntries : (%d)", nvs_stats.total_entries);

    nvs_get_stats(GROUP_NVS_PARTITION_NAME, &nvs_stats);
    ESP_LOGI(NVS_TAG, "=-=-=-=-=-=-=-=-=-=- GROUP_PARTITION_STATS -=-=-=-=-=-=-=-=-=-=");
    ESP_LOGI(NVS_TAG, "UsedEntries : (%d)", nvs_stats.used_entries);
    ESP_LOGI(NVS_TAG, "FreeEntries : (%d)", nvs_stats.free_entries);
    ESP_LOGI(NVS_TAG, "AllEntries : (%d)", nvs_stats.total_entries);
}