#ifndef GROUP_TABLE_H
#define GROUP_TABLE_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================================
// Group Subscription Table
// Maintains a local record of which BLE Mesh nodes (and gateway itself) are
// subscribed to which group addresses.
//
// The ESP-IDF BLE Mesh API does not provide a query function for group
// subscriptions — this table is the gateway's own record, kept in sync
// with actual node subscription tables via subscribe/unsubscribe ACKs.
//
// Persisted in NVS flash so group memberships survive reboots.
// ============================================================================

// ── Limits ──────────────────────────────────────────────────────────────────
#define MAX_GROUPS              16    // maximum distinct group addresses
#define MAX_NODES_PER_GROUP     24    // maximum nodes per group (matches node limit)

// ── BLE Mesh address constants ───────────────────────────────────────────────
#define BLE_MESH_GROUP_ADDR_MIN  0xC000
#define BLE_MESH_GROUP_ADDR_MAX  0xFFFE
#define PROV_OWN_ADDR            0x0001   // gateway's own BLE Mesh unicast address

// ── Timing ──────────────────────────────────────────────────────────────────
#define GROUP_ACK_TIMEOUT_MS    10000   // wait up to 10s for all node ACKs

// ── NVS ─────────────────────────────────────────────────────────────────────
#define NVS_GROUP_TABLE_KEY     "GroupTable"

// ============================================================================
// Group membership table
// ============================================================================

typedef struct
{
    uint16_t group_addr;                        // BLE Mesh group address e.g. 0xC001
    uint16_t node_addrs[MAX_NODES_PER_GROUP];   // unicast addresses of subscribed members
    uint8_t  node_count;                        // number of members in this group
} GroupEntry_t;

typedef struct
{
    GroupEntry_t groups[MAX_GROUPS];
    uint8_t      group_count;                   // number of distinct groups
} GroupTable_t;

// ============================================================================
// Group AC control ACK tracker
// One tracker per in-flight group AC control command.
// Gateway waits for ACKs from all subscribed nodes before sending summary ACK.
// ============================================================================

typedef struct GroupAckTracker_tag
{
    uint16_t    group_addr;                         // group this tracker belongs to
    uint16_t    seq;                                // unique sequence number per command
    uint8_t     expected;                           // number of node ACKs expected
    uint8_t     received;                           // number of node ACKs received so far
    uint8_t     gwy_subscribed;                     // 1 if GWY itself was in the group
    uint16_t    failed_addrs[MAX_NODES_PER_GROUP];  // nodes that haven't ACKed yet
    uint8_t     failed_count;                       // number of nodes that failed/timed out
    TickType_t  start_tick;                         // when group command was sent
    CommandStruct cmd;                              // original command for summary ACK
    struct GroupAckTracker_tag *next;               // linked list pointer
} GroupAckTracker_t;



// ============================================================================
// Globals — defined in group_table.c
// ============================================================================
extern GroupTable_t      group_table;
extern GroupAckTracker_t *group_tracker_head;   // head of linked list — NULL when empty
extern uint16_t           group_cmd_seq_counter;

// ============================================================================
// NVS persistence
// ============================================================================
/**
 * @brief Save group table to NVS flash
 */
void group_table_save(void);

/**
 * @brief Load group table from NVS flash — call in nvs_init() after handles open
 */
void group_table_load(void);

// ============================================================================
// Group membership management
// ============================================================================
/**
 * @brief Add a node/gateway to a group
 * @param group_addr  BLE Mesh group address (0xC000–0xFFFE)
 * @param node_addr   Unicast address of node (or PROV_OWN_ADDR for gateway)
 * @return ESP_OK, ESP_ERR_NO_MEM if full
 */
esp_err_t group_table_subscribe(uint16_t group_addr, uint16_t node_addr);

/**
 * @brief Remove a node/gateway from a group
 * @return ESP_OK, ESP_ERR_NOT_FOUND if not found
 */
esp_err_t group_table_unsubscribe(uint16_t group_addr, uint16_t node_addr);

// ============================================================================
// Group table queries
// ============================================================================
/**
 * @brief Get total member count for a group (nodes + gateway if subscribed)
 */
uint8_t group_table_get_count(uint16_t group_addr);

/**
 * @brief Get pointer to GroupEntry_t for a group address, or NULL
 */
GroupEntry_t *group_table_get_entry(uint16_t group_addr);

/**
 * @brief Check if a specific address is subscribed to a group
 */
bool group_table_is_subscribed(uint16_t group_addr, uint16_t node_addr);

/**
 * @brief Print full group table to serial log
 */
void group_table_print(void);

/**
 * @brief Build group table JSON response into a pre-allocated buffer
 *        Used to respond to GWY_GROUP_TABLE_PACKET (ID:18) from cloud
 *        and to auto-publish group state on MQTT reconnect
 */
void group_table_build_json(char      *buffer,
                             size_t     buf_len,
                             int        msgseqno,
                             const char *serialNo);


// ============================================================================
// ACK tracker — linked list, one node per in-flight group AC command
// ============================================================================
GroupAckTracker_t *group_tracker_alloc(GroupEntry_t *entry,
                                        uint16_t      group_addr,
                                        uint8_t       gwy_sub);        // malloc new tracker, insert at head
GroupAckTracker_t *group_tracker_find_by_seq(uint16_t seq);            // find tracker by sequence number
bool               group_tracker_record_ack(GroupAckTracker_t *tracker,
                                             uint16_t           node_addr,
                                             int                errorcode); // record node ACK, returns true when all done
void               group_tracker_free(GroupAckTracker_t *tracker);     // remove from list and free memory
void               group_tracker_check_timeouts(void);                 // call from LTE loop every 10ms

#endif // GROUP_TABLE_H