#ifndef GROUP_TABLE_H
#define GROUP_TABLE_H

#include <stdint.h>
#include <esp_err.h>

// ============================================================================
// Group Subscription Table
// Maintains a local record of which BLE Mesh nodes are subscribed to which
// group addresses. Used to track membership for group control commands.
//
// The ESP-IDF BLE Mesh API does not provide a query function for group
// subscriptions — this table is the gateway's own record, kept in sync
// with the actual node subscription tables via subscribe/unsubscribe ACKs.
//
// Persisted in NVS flash so group memberships survive reboots.
// ============================================================================

#define MAX_GROUPS          16    // maximum number of distinct group addresses
#define MAX_NODES_PER_GROUP 24    // maximum nodes per group (matches node limit)

#define NVS_GROUP_TABLE_KEY "GroupTable"

// BLE Mesh group address valid range
#define BLE_MESH_GROUP_ADDR_MIN  0xC000
#define BLE_MESH_GROUP_ADDR_MAX  0xFFFE

typedef struct
{
    uint16_t group_addr;                        // BLE Mesh group address e.g. 0xC001
    uint16_t node_addrs[MAX_NODES_PER_GROUP];   // unicast addresses of subscribed nodes
    uint8_t  node_count;                        // number of nodes currently in this group
} GroupEntry_t;

typedef struct
{
    GroupEntry_t groups[MAX_GROUPS];
    uint8_t group_count;                        // number of distinct groups
} GroupTable_t;

// Global group table instance — defined in group_table.c
extern GroupTable_t group_table;

// ============================================================================
// NVS persistence
// ============================================================================

/**
 * @brief Save the full group table to NVS flash
 */
void group_table_save(void);

/**
 * @brief Load the group table from NVS flash on boot
 *        Call this in init_flash() after NVS is initialized
 */
void group_table_load(void);

// ============================================================================
// Group membership management
// ============================================================================

/**
 * @brief Add a node to a group
 * @param group_addr  BLE Mesh group address (0xC000–0xFFFE)
 * @param node_addr   Node unicast address
 * @return ESP_OK on success
 *         ESP_ERR_NO_MEM if group table or group is full
 */
esp_err_t group_table_subscribe(uint16_t group_addr, uint16_t node_addr);

/**
 * @brief Remove a node from a group
 * @param group_addr  BLE Mesh group address
 * @param node_addr   Node unicast address
 * @return ESP_OK on success
 *         ESP_ERR_NOT_FOUND if node or group not found
 */
esp_err_t group_table_unsubscribe(uint16_t group_addr, uint16_t node_addr);

// ============================================================================
// Group table queries
// ============================================================================

/**
 * @brief Get the number of nodes subscribed to a group
 * @param group_addr  BLE Mesh group address
 * @return Node count, or 0 if group not found
 */
uint8_t group_table_get_count(uint16_t group_addr);

/**
 * @brief Get a pointer to the group entry for a given group address
 * @param group_addr  BLE Mesh group address
 * @return Pointer to GroupEntry_t, or NULL if not found
 */
GroupEntry_t *group_table_get_entry(uint16_t group_addr);

/**
 * @brief Check if a node is subscribed to a group
 * @param group_addr  BLE Mesh group address
 * @param node_addr   Node unicast address
 * @return true if subscribed, false otherwise
 */
bool group_table_is_subscribed(uint16_t group_addr, uint16_t node_addr);

/**
 * @brief Print the full group table to serial log (for debugging)
 */
void group_table_print(void);

#endif // GROUP_TABLE_H