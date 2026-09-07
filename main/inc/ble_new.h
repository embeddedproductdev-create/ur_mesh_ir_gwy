#ifndef BLE_NEW_H
#define BLE_NEW_H

#include "esp_ble_mesh_defs.h"

#include <main.h>
#include <lte.h>

#define BLE_NODE_COMM_TIMEOUT_MS 20000
#define BLE_RESP_BUFFER_LEN 200

#define PROV_OWN_ADDR           0x0001
#define GROUP_ACK_TIMEOUT_MS    10000
#define MAX_PENDING_GROUP_CMDS  5

/*Global Variables*/
extern bool ble_initialized;

/*Function Declarations*/
void ble_init();
void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);
esp_err_t bluetooth_init(void);
void send_cmd_to_node(CommandStruct *cmd);

void handle_cmds_from_provisioner(CommandStruct *cmd);
void provision_success_cb();
void unprovision_success_cb();
void attach_elemAddr_to_structures(uint16_t elemAddr);
void handle_ble_incoming(esp_ble_mesh_model_cb_param_t *param);
#ifdef __cplusplus
extern "C" {
#endif
void send_ack_to_provisioner(uint16_t packetid, CommandStruct *ack);
void send_teaching_mode_ack_to_provisioner();
void send_manual_control_ack_to_provisioner();

void ble_send_group_ac_control(CommandStruct *cmd);
void handle_gwy_group_subscribe(CommandStruct *cmd);
void handle_gwy_group_unsubscribe(CommandStruct *cmd);
void handle_group_ac_control(CommandStruct *cmd);

/**
 * @brief Generic BLE Mesh sender for RS485 relay structs (MeterConfigCmd,
 *        GpioConfigCmd, GpioSetCmd, SumanAcControlCmd). Unlike
 *        send_cmd_to_node(), this does not go through command_queue —
 *        RS485 relay commands are fire-and-forget from the gateway's
 *        perspective; the node's ACK (a RS485CmdStruct echoing the same
 *        packetid) is matched to the original cloud MsgSeqNo by
 *        rs485_relay.c's pending-command tracker, not by command_queue.
 * @param elemaddr     Target node's BLE Mesh unicast address
 * @param payload      Pointer to the struct to send (packetid must be its first field)
 * @param payload_len  sizeof(*payload)
 */
void send_rs485_struct_to_node(uint16_t elemaddr, const void *payload, size_t payload_len);

/*===========================================================
+ * Node type table — elemaddr -> node type (NODE_TYPE_IR_AC /
+ * NODE_TYPE_RS485), read from dev_uuid[8] at provisioning time.
+ * Table itself stays private to ble_prov_new.c — reachable only
+ * through these three functions.
+ *===========================================================*/
typedef struct
{
    uint16_t elemaddr;
    uint8_t  node_type;   /* NODE_TYPE_IR_AC / NODE_TYPE_RS485 */
    bool     valid;
} node_type_entry_t;

/** @brief Record a node's type at provisioning time — called from prov_complete(). */
void node_type_table_set(uint16_t elemaddr, uint8_t node_type);

/**
+ * @brief Node type for a provisioned node's element address, recorded from
+ *        dev_uuid[8] at provisioning time (see prov_complete() in
+ *        ble_prov_new.c). Returns 0 ("unknown") if elemaddr was never
+ *        provisioned or has since been unprovisioned.
+ */
uint8_t get_node_type_for_elemaddr(uint16_t elemaddr);

/** @brief Drop the node-type entry for elemaddr — called on NODE_UNPROV_PACKET. */
void node_type_table_clear(uint16_t elemaddr);

#ifdef __cplusplus
}
#endif

#endif