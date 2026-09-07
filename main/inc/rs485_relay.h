#ifndef RS485_RELAY_H
#define RS485_RELAY_H
 
#include <cJSON.h>
#include "lte.h"          // mqtt_packets, error_codes
#include "rs485_types.h"  // RS485CmdStruct, MeterConfigCmd, ModbusReadingPkt, ...
 
/**
 * @brief Validate+relay a cloud->gateway->node RS485 command.
 *        packetid must be one of: NODE_METER_CONFIG_PACKET (151),
 *        NODE_GPIO_CONFIG_PACKET (154), NODE_GPIO_SET_PACKET (155),
 *        NODE_SUMAN_AC_CONTROL_PACKET (158).
 *
 *        On successful validation, relays the parsed struct to the
 *        target node over BLE Mesh and returns — the ACK to cloud is
 *        sent later, asynchronously, when the node's reply arrives via
 *        rs485_relay_handle_node_ack().
 *
 *        On validation failure, builds and enqueues an error ACK to
 *        cloud immediately (tagged "ErrorSource":"GATEWAY" so the cloud
 *        can tell it apart from a node-side rejection, since gateway
 *        and node error codes are different, unrelated numbering
 *        spaces) — the command is never sent to the node in this case.
 *
 * @param json_obj  Parsed MQTT command JSON (JsonPacketID already confirmed
 *                  to be one of the four values above by the caller)
 * @param packetid  Which of the four command types this is
 */
void rs485_relay_handle_cloud_command(cJSON *json_obj, mqtt_packets packetid);
 
/**
 * @brief Handle a generic RS485CmdStruct ACK arriving from a node over BLE
 *        Mesh (packetid is one of the four command types above, echoed
 *        back with errorcode set). Builds the cloud-facing ACK JSON
 *        ("ErrorSource":"NODE") and enqueues it for MQTT publish.
 * @param ack  Cast of the raw BLE Mesh payload — rssi must already be set
 *             by the caller (handle_ble_incoming), the rest comes from the node.
 */
void rs485_relay_handle_node_ack(RS485CmdStruct *ack);
 
/** @brief Publish a node's meter reading (152=3-phase, 153=1-phase) to MQTT as-is. */
void rs485_relay_publish_meter_reading(ModbusReadingPkt *pkt);
 
/** @brief Publish a node's GPIO status (156) to MQTT as-is. */
void rs485_relay_publish_gpio_status(GpioStatusPkt *pkt);
 
/** @brief Publish a node's Suman AC controller status (157) to MQTT as-is. */
void rs485_relay_publish_suman_ac_status(SumanAcStatusPkt *pkt);
 
/**
 * @brief Scan in-flight RS485 command trackers for timeout. Call once per
 *        maintainMQTTConnection() loop iteration, same cadence as the
 *        existing group_tracker_check_timeouts(). Any tracker older than
 *        BLE_NODE_COMM_TIMEOUT_MS is freed and a NODE_COMM_TIMEOUT ACK
 *        (ErrorSource: GATEWAY) is sent to cloud in its place.
 */
void rs485_relay_check_timeouts(void);
 
#endif /* RS485_RELAY_H */