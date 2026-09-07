#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "flash.h"
#include "lte.h"
#include "group_table.h"

#include "json_maker.h"   // for jWrite APIs

#define GROUP_TAG "GROUP_TABLE"

// ============================================================================
// Globals
// ============================================================================

GroupTable_t      group_table          = {0};
GroupAckTracker_t *group_tracker_head  = NULL;
uint16_t           group_cmd_seq_counter = 0;

// ============================================================================
// NVS persistence
// ============================================================================
/**
 * @brief Save the full group table to NVS flash
 *        Called automatically after every subscribe/unsubscribe operation
 */
void group_table_save(void)
{
    set_blob_in_nvs_flash(GROUP_HANDLE, NVS_GROUP_TABLE_KEY,
        &group_table, sizeof(GroupTable_t));
    ESP_LOGI(GROUP_TAG, "Group table saved (%d groups)", group_table.group_count);
}

/**
 * @brief Load the group table from NVS flash
 *        Call this once in nvs_init() after all NVS handles are opened
 *        Prints the loaded table to serial log via group_table_print()
 */
void group_table_load(void)
{
    size_t size = sizeof(GroupTable_t);
    get_blob_from_nvs_flash(GROUP_HANDLE, NVS_GROUP_TABLE_KEY,
        &group_table, &size);
    ESP_LOGI(GROUP_TAG, "Group table loaded (%d groups)", group_table.group_count);
    group_table_print();
}

// ============================================================================
// Group membership management
// ============================================================================
/**
 * @brief Add a node or gateway to a group
 *        If the group does not exist, creates a new entry
 *        If the node is already subscribed, returns ESP_OK with no change
 *        Saves group table to NVS on success
 * @param group_addr  BLE Mesh group address (0xC000–0xFFFE)
 * @param node_addr   Unicast address of the node, or PROV_OWN_ADDR (0x0001) for gateway
 * @return ESP_OK on success
 *         ESP_ERR_NO_MEM if group table is full or group has reached MAX_NODES_PER_GROUP
 */
esp_err_t group_table_subscribe(uint16_t group_addr, uint16_t node_addr)
{
    // Search for existing group entry
    for (int i = 0; i < group_table.group_count; i++)
    {
        if (group_table.groups[i].group_addr == group_addr)
        {
            // Check if already subscribed
            for (int j = 0; j < group_table.groups[i].node_count; j++)
            {
                if (group_table.groups[i].node_addrs[j] == node_addr)
                {
                    ESP_LOGW(GROUP_TAG, "0x%04x already in group 0x%04x — no change",
                        node_addr, group_addr);
                    return ESP_OK;
                }
            }
            // Group found — add member
            if (group_table.groups[i].node_count >= MAX_NODES_PER_GROUP)
            {
                ESP_LOGE(GROUP_TAG, "Group 0x%04x full (%d/%d)",
                    group_addr, group_table.groups[i].node_count, MAX_NODES_PER_GROUP);
                return ESP_ERR_NO_MEM;
            }
            group_table.groups[i].node_addrs[group_table.groups[i].node_count++] = node_addr;
            group_table_save();
            ESP_LOGI(GROUP_TAG, "0x%04x added to group 0x%04x (%d members)",
                node_addr, group_addr, group_table.groups[i].node_count);
            return ESP_OK;
        }
    }
    // Group not found — create new entry
    if (group_table.group_count >= MAX_GROUPS)
    {
        ESP_LOGE(GROUP_TAG, "Group table full (%d/%d groups)",
            group_table.group_count, MAX_GROUPS);
        return ESP_ERR_NO_MEM;
    }
    int idx = group_table.group_count++;
    group_table.groups[idx].group_addr    = group_addr;
    group_table.groups[idx].node_addrs[0] = node_addr;
    group_table.groups[idx].node_count    = 1;
    group_table_save();
    ESP_LOGI(GROUP_TAG, "New group 0x%04x created with 0x%04x (%d groups total)",
        group_addr, node_addr, group_table.group_count);
    return ESP_OK;
}

/**
 * @brief Remove a node or gateway from a group
 *        Shifts remaining members left to fill the gap
 *        Saves group table to NVS on success
 * @param group_addr  BLE Mesh group address
 * @param node_addr   Unicast address of the node, or PROV_OWN_ADDR for gateway
 * @return ESP_OK on success
 *         ESP_ERR_NOT_FOUND if node or group not found
 */
esp_err_t group_table_unsubscribe(uint16_t group_addr, uint16_t node_addr)
{
    for (int i = 0; i < group_table.group_count; i++)
    {
        if (group_table.groups[i].group_addr == group_addr)
        {
            for (int j = 0; j < group_table.groups[i].node_count; j++)
            {
                if (group_table.groups[i].node_addrs[j] == node_addr)
                {
                    // Shift remaining left
                    for (int k = j; k < group_table.groups[i].node_count - 1; k++)
                        group_table.groups[i].node_addrs[k] = group_table.groups[i].node_addrs[k + 1];
                    group_table.groups[i].node_count--;
                    group_table_save();
                    ESP_LOGI(GROUP_TAG, "0x%04x removed from group 0x%04x (%d remaining)",
                        node_addr, group_addr, group_table.groups[i].node_count);
                    return ESP_OK;
                }
            }
            ESP_LOGW(GROUP_TAG, "0x%04x not in group 0x%04x", node_addr, group_addr);
            return ESP_ERR_NOT_FOUND;
        }
    }
    ESP_LOGW(GROUP_TAG, "Group 0x%04x not found", group_addr);
    return ESP_ERR_NOT_FOUND;
}

// ============================================================================
// Group table queries
// ============================================================================

/**
 * @brief Get total member count for a group including gateway if subscribed
 * @param group_addr  BLE Mesh group address
 * @return Member count, or 0 if group not found
 */
uint8_t group_table_get_count(uint16_t group_addr)
{
    for (int i = 0; i < group_table.group_count; i++)
        if (group_table.groups[i].group_addr == group_addr)
            return group_table.groups[i].node_count;
    return 0;
}

/**
 * @brief Get a pointer to the GroupEntry_t for a given group address
 *        Use this to iterate over member addresses for group AC control
 * @param group_addr  BLE Mesh group address
 * @return Pointer to GroupEntry_t, or NULL if group not found
 */
GroupEntry_t *group_table_get_entry(uint16_t group_addr)
{
    for (int i = 0; i < group_table.group_count; i++)
        if (group_table.groups[i].group_addr == group_addr)
            return &group_table.groups[i];
    return NULL;
}

/**
 * @brief Check if a specific node or gateway is subscribed to a group
 * @param group_addr  BLE Mesh group address
 * @param node_addr   Unicast address to check, or PROV_OWN_ADDR for gateway
 * @return true if subscribed, false if not found
 */
bool group_table_is_subscribed(uint16_t group_addr, uint16_t node_addr)
{
    GroupEntry_t *entry = group_table_get_entry(group_addr);
    if (entry == NULL) return false;
    for (int i = 0; i < entry->node_count; i++)
        if (entry->node_addrs[i] == node_addr)
            return true;
    return false;
}

/**
 * @brief Print the full group table to serial log
 *        Shows each group address and all member unicast addresses
 *        Labels PROV_OWN_ADDR as (GWY) and all other addresses as (NODE)
 */
void group_table_print(void)
{
    if (group_table.group_count == 0)
    {
        ESP_LOGI(GROUP_TAG, "Group table is empty");
        return;
    }
    ESP_LOGI(GROUP_TAG, "=== Group Table (%d groups) ===", group_table.group_count);
    for (int i = 0; i < group_table.group_count; i++)
    {
        ESP_LOGI(GROUP_TAG, "Group 0x%04x (%d members):",
            group_table.groups[i].group_addr,
            group_table.groups[i].node_count);
        for (int j = 0; j < group_table.groups[i].node_count; j++)
        {
            if (group_table.groups[i].node_addrs[j] == PROV_OWN_ADDR)
                ESP_LOGI(GROUP_TAG, "  [%d] 0x%04x (GWY)", j,
                    group_table.groups[i].node_addrs[j]);
            else
                ESP_LOGI(GROUP_TAG, "  [%d] 0x%04x (NODE)", j,
                    group_table.groups[i].node_addrs[j]);
        }
    }
    ESP_LOGI(GROUP_TAG, "==============================");
}

/**
 * @brief Build group table JSON response into a pre-allocated buffer
 *        Used to respond to GWY_GROUP_TABLE_PACKET (ID:18) from cloud
 *        and to auto-publish group state on MQTT reconnect
 *
 *        Output JSON format:
 *        {
 *            "JsonPacketID": 18,
 *            "MsgSeqNo": <msgseqno>,
 *            "GwySerNo": <serialNo>,
 *            "GroupCount": <N>,
 *            "Groups": [
 *                {
 *                    "GroupAddr": <addr>,
 *                    "MemberCount": <N>,
 *                    "Members": [
 *                        {"ElementAddr": 1,  "Type": "GWY"},
 *                        {"ElementAddr": 5,  "Type": "NODE"},
 *                        ...
 *                    ]
 *                },
 *                ...
 *            ]
 *        }
 *
 * @param buffer    Pre-allocated buffer — caller mallocs, caller passes to enqueue_for_publish()
 * @param buf_len   Size of buffer — must be >= MQTT_ACK_BUFFER_LEN
 * @param msgseqno  Echoed from cloud request MsgSeqNo. Pass 0 for auto-publish on reconnect
 * @param serialNo  Gateway serial number string e.g. "GWY00013"
 */
void group_table_build_json(char      *buffer,
                             size_t     buf_len,
                             int        msgseqno,
                             const char *serialNo)
{
    jWriteControl_t jwc;
    jwOpen(&jwc, buffer, buf_len, JW_OBJECT, 1);

    jwObj_int(&jwc, "JsonPacketID", GWY_GROUP_TABLE_PACKET);     // Group Table packet
    jwObj_int(&jwc, "MsgSeqNo", msgseqno);
    jwObj_string(&jwc, "GwySerNo", (char *)serialNo);
    jwObj_int(&jwc, "GroupCount", group_table.group_count);

    jwObj_array(&jwc, "Groups");
    for (int i = 0; i < group_table.group_count; i++)
    {
        GroupEntry_t *entry = &group_table.groups[i];

        jwArr_object(&jwc);
            jwObj_int(&jwc, "GroupAddr", entry->group_addr);
            jwObj_int(&jwc, "MemberCount", entry->node_count);

            jwObj_array(&jwc, "Members");
            for (int j = 0; j < entry->node_count; j++)
            {
                jwArr_object(&jwc);
                    jwObj_int(&jwc, "ElementAddr", entry->node_addrs[j]);
                    jwObj_string(&jwc, "Type",
                        entry->node_addrs[j] == PROV_OWN_ADDR
                        ? (char *)"GWY"
                        : (char *)"NODE");
                jwEnd(&jwc);   // closes member object
            }
            jwEnd(&jwc);       // closes Members array

        jwEnd(&jwc);           // closes group object
    }
    jwEnd(&jwc);               // closes Groups array

    jwEnd(&jwc);               // closes root object
    jwClose(&jwc);

    ESP_LOGI(GROUP_TAG, "Group table JSON built: %d groups", group_table.group_count);
}

// ============================================================================
// ACK tracker management
// ============================================================================
/**
 * @brief Allocate and initialize a new tracker node, insert at head of list
 *        Assigns a unique sequence number from group_cmd_seq_counter
 *        Populates failed_addrs with all non-GWY members (cleared as ACKs arrive)
 *        Returns NULL if malloc fails — check heap before calling
 * @param entry       GroupEntry_t for the target group
 * @param group_addr  BLE Mesh group address
 * @param gwy_sub     1 if gateway itself is subscribed to this group, 0 otherwise
 * @return Pointer to new tracker node, or NULL on malloc failure
 */
GroupAckTracker_t *group_tracker_alloc(GroupEntry_t *entry,
                                        uint16_t      group_addr,
                                        uint8_t       gwy_sub)
{
    GroupAckTracker_t *tracker = (GroupAckTracker_t *)malloc(sizeof(GroupAckTracker_t));
    if (tracker == NULL)
    {
        ESP_LOGE(GROUP_TAG, "Tracker malloc failed — heap: %" PRIu32,
            esp_get_free_heap_size());
        return NULL;
    }

    memset(tracker, 0, sizeof(GroupAckTracker_t));
    group_cmd_seq_counter++;

    tracker->seq            = group_cmd_seq_counter;
    tracker->group_addr     = group_addr;
    tracker->gwy_subscribed = gwy_sub;
    tracker->start_tick     = xTaskGetTickCount();
    tracker->next           = NULL;

    // Populate expected nodes — all non-GWY members
    int fi = 0;
    for (int i = 0; i < entry->node_count; i++)
        if (entry->node_addrs[i] != PROV_OWN_ADDR)
            tracker->failed_addrs[fi++] = entry->node_addrs[i];
    tracker->expected     = fi;
    tracker->failed_count = fi;

    // Insert at head of linked list
    tracker->next      = group_tracker_head;
    group_tracker_head = tracker;

    ESP_LOGI(GROUP_TAG, "Tracker [seq=%d] allocated: group=0x%04x expected=%d gwy=%d heap=%" PRIu32,
        tracker->seq, group_addr, tracker->expected, gwy_sub, esp_get_free_heap_size());

    return tracker;
}

/**
 * @brief Find an active tracker in the linked list by sequence number
 *        Used by handle_ble_incoming() to match node ACKs to the correct command
 * @param seq  Sequence number assigned by group_tracker_alloc()
 * @return Pointer to matching tracker, or NULL if not found
 */
GroupAckTracker_t *group_tracker_find_by_seq(uint16_t seq)
{
    GroupAckTracker_t *curr = group_tracker_head;
    while (curr != NULL)
    {
        if (curr->seq == seq)
            return curr;
        curr = curr->next;
    }
    return NULL;
}

/**
 * @brief Remove a tracker from the linked list and free its memory
 *        Must be called after generate_ack(NODE_GROUP_AC_CONTROL_SUMMARY_ACK) returns
 *        so the tracker data is still valid when the ACK JSON is built
 *        Handles both head and non-head removal cases
 * @param tracker  Pointer to tracker to free — do not use after calling this
 */
void group_tracker_free(GroupAckTracker_t *tracker)
{
    // Remove from linked list
    if (group_tracker_head == tracker)
    {
        group_tracker_head = tracker->next;
    }
    else
    {
        GroupAckTracker_t *curr = group_tracker_head;
        while (curr != NULL && curr->next != tracker)
            curr = curr->next;
        if (curr != NULL)
            curr->next = tracker->next;
    }

    ESP_LOGI(GROUP_TAG, "Tracker [seq=%d] freed — heap: %" PRIu32,
        tracker->seq, esp_get_free_heap_size());

    free(tracker);
}

/**
 * @brief Record a node ACK against an in-flight tracker
 *        Increments received count
 *        If errorcode == SUCCESS, removes node_addr from failed_addrs list
 *        If errorcode != SUCCESS, node remains in failed_addrs for summary ACK
 * @param tracker    Active tracker returned by group_tracker_find_by_seq()
 * @param node_addr  Unicast address of the node that sent the ACK
 * @param errorcode  0 (SUCCESS) if node executed successfully, non-zero otherwise
 * @return true if all expected ACKs have been received — caller should then
 *         call generate_ack(NODE_GROUP_AC_CONTROL_SUMMARY_ACK) and group_tracker_free()
 */
bool group_tracker_record_ack(GroupAckTracker_t *tracker,
                               uint16_t           node_addr,
                               int                errorcode)
{
    tracker->received++;

    // Remove from failed list if ACK was successful
    if (errorcode == 0)  // SUCCESS
    {
        for (int i = 0; i < tracker->failed_count; i++)
        {
            if (tracker->failed_addrs[i] == node_addr)
            {
                for (int j = i; j < tracker->failed_count - 1; j++)
                    tracker->failed_addrs[j] = tracker->failed_addrs[j + 1];
                tracker->failed_count--;
                break;
            }
        }
    }

    ESP_LOGI(GROUP_TAG, "Tracker [seq=%d]: %d/%d ACKs (node 0x%04x err=%d)",
        tracker->seq, tracker->received, tracker->expected, node_addr, errorcode);

    // Return true if all expected ACKs received
    return (tracker->received >= tracker->expected);
}

/**
 * @brief Check all active trackers for timeout — call from maintainMQTTConnection() loop
 *        For each tracker where elapsed time >= GROUP_ACK_TIMEOUT_MS:
 *          - Logs timeout warning with received/expected/failed counts
 *          - Calls generate_ack(NODE_GROUP_AC_CONTROL_SUMMARY_ACK)
 *          - Calls group_tracker_free() to remove and release memory
 *        Saves next pointer before freeing to safely traverse the list
 */
void group_tracker_check_timeouts(void)
{
    GroupAckTracker_t *curr = group_tracker_head;
    while (curr != NULL)
    {
        GroupAckTracker_t *next = curr->next;  // save before potential free

        TickType_t elapsed_ms = (xTaskGetTickCount() - curr->start_tick)
                                * portTICK_PERIOD_MS;

        if (elapsed_ms >= GROUP_ACK_TIMEOUT_MS)
        {
            ESP_LOGW(GROUP_TAG, "Tracker [seq=%d] TIMEOUT: %d/%d ACKs %d failed",
                curr->seq, curr->received, curr->expected, curr->failed_count);
            generate_ack(NODE_GROUP_AC_CONTROL_SUMMARY_ACK, &curr->cmd);
            group_tracker_free(curr);
        }

        curr = next;
    }
}