// #ifndef FLASH_H
// #define FLASH_H

#include "nvs_flash.h"
#include "nvs.h"
#include <lte.h>

#define IR_NVS_PARTITION_NAME "ir"
#define GENERAL_NVS_PARTITION_NAME "general"
#define SERIALNUM_NVS_PARTITION_NAME "serialnum"

#define IR_NVS_NAMESPACE "IR"
#define GENERAL_NVS_NAMESPACE "GENERAL"
#define SERIALNUM_NVS_NAMESPACE "serial"

#define GROUP_NVS_PARTITION_NAME  "group"
#define GROUP_NVS_NAMESPACE       "GROUP"

/*Global Variables*/
extern nvs_handle_t ir_nvs_handle;
extern nvs_handle_t general_nvs_handle;
extern const char *NVS_TEACHING_MODE_CMD_KEYS[];

extern const char *NVS_NEW_DEVICE_KEY;
extern const char *NVS_SERIAL_NO_KEY;
extern const char *NVS_DEVICE_LOCATION_KEY;
extern const char *NVS_REGISTERED_KEY;
extern const char *NVS_CONFIGURED_KEY;
extern const char *NVS_PROVISIONED_KEY;
extern const char *NVS_RAWLEN_KEY;
extern const char *NVS_IR_PROTOCOL_KEY;
extern const char *NVS_POWER_KEY;
extern const char *NVS_TEMPERATURE_KEY;
extern const char *NVS_FANSPEED_KEY;
extern const char *NVS_MODE_KEY;
extern const char *NVS_SWINGH_KEY;
extern const char *NVS_SWINGV_KEY;
extern const char *NVS_LOCKING_KEY;
extern const char *NVS_ONTIMER_KEY;
extern const char *NVS_OFFTIMER_KEY;
extern const char *NVS_UPPER_TEMPERATURE_LIMIT_KEY;
extern const char *NVS_LOWER_TEMPERATURE_LIMIT_KEY;
extern const char *NVS_PUBPERIOD_KEY;
extern const char *NVS_LAST_COMMAND_KEY;
extern const char *NVS_TEACHING_MODE_STARTING_TEMPERATURE_KEY;
extern const char *NVS_TEACHING_MODE_ENDING_TEMPERATURE_KEY;

typedef enum
{
    IR_HANDLE,
    GENERAL_HANDLE,
    GROUP_HANDLE,
}handle_enum_t;

/*Function declarations*/
void nvs_init();
int32_t get_number_from_nvs_flash(handle_enum_t nvshandle, const char *key, sizes_t size);
char *get_str_from_nvs_flash(handle_enum_t nvshandle, const char *key);
void set_last_ac_cmd_in_nvs_flash();
error_codes factory_reset_device();
#ifdef __cplusplus
extern "C" {
#endif

void set_number_in_nvs_flash(handle_enum_t nvshandle, const char *key, int value, sizes_t size);
void set_str_in_nvs_flash(handle_enum_t nvshandle, const char *key, char *value);
void set_blob_in_nvs_flash(handle_enum_t nvshandle, const char *key, const void *value, size_t length);
void get_blob_from_nvs_flash(handle_enum_t nvshandle, const char *key, void *value, size_t *length);
void pull_ir_cmd_data();

#ifdef __cplusplus
}
#endif

// #endif