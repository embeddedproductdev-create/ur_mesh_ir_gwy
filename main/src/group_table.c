#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "group_table.h"
#include "flash.h"

#define GROUP_TAG "GROUP_TABLE"

// ============================================================================
// Global group table instance
// ============================================================================
GroupTable_t group_table = {0};

// ============================================================================
// NVS persistence
// ============================================================================

void group_table_save(void)
{
    set_blob_in_nvs_flash(GROUP_HANDLE, NVS_GROUP_TABLE_KEY,
        &group_table, sizeof(GroupTable_t));
    ESP_LOGI(GROUP_TAG, "Group table saved (%d groups)", group_table.group_count);
}

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

esp_err_t group_table_subscribe(uint16_t group_addr, uint16_t node_addr)
{
    // Search for existing group entry
    for (int i = 0; i < group_table.group_count; i++)
    {
        if (group_table.groups[i].group_addr == group_addr)
        {
            // Check if node is already subscribed
            for (int j = 0; j < group_table.groups[i].node_count; j++)
            {
                if (group_table.groups[i].node_addrs[j] == node_addr)
                {
                    ESP_LOGW(GROUP_TAG, "Node 0x%04x already in group 0x%04x — no change",
                        node_addr, group_addr);
                    return ESP_OK;
                }
            }

            // Group found — add node
            if (group_table.groups[i].node_count >= MAX_NODES_PER_GROUP)
            {
                ESP_LOGE(GROUP_TAG, "Group 0x%04x is full (%d/%d nodes)",
                    group_addr, group_table.groups[i].node_count, MAX_NODES_PER_GROUP);
                return ESP_ERR_NO_MEM;
            }

            group_table.groups[i].node_addrs[group_table.groups[i].node_count++] = node_addr;
            group_table_save();
            ESP_LOGI(GROUP_TAG, "Node 0x%04x added to group 0x%04x (%d/%d nodes)",
                node_addr, group_addr,
                group_table.groups[i].node_count, MAX_NODES_PER_GROUP);
            return ESP_OK;
        }
    }

    // Group not found — create new entry
    if (group_table.group_count >= MAX_GROUPS)
    {
        ESP_LOGE(GROUP_TAG, "Group table full (%d/%d groups)", group_table.group_count, MAX_GROUPS);
        return ESP_ERR_NO_MEM;
    }

    int idx = group_table.group_count++;
    group_table.groups[idx].group_addr    = group_addr;
    group_table.groups[idx].node_addrs[0] = node_addr;
    group_table.groups[idx].node_count    = 1;
    group_table_save();
    ESP_LOGI(GROUP_TAG, "New group 0x%04x created with node 0x%04x (group %d/%d)",
        group_addr, node_addr, group_table.group_count, MAX_GROUPS);
    return ESP_OK;
}

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
                    // Shift remaining nodes left to fill the gap
                    for (int k = j; k < group_table.groups[i].node_count - 1; k++)
                        group_table.groups[i].node_addrs[k] = group_table.groups[i].node_addrs[k + 1];

                    group_table.groups[i].node_count--;
                    group_table_save();
                    ESP_LOGI(GROUP_TAG, "Node 0x%04x removed from group 0x%04x (%d nodes remaining)",
                        node_addr, group_addr, group_table.groups[i].node_count);
                    return ESP_OK;
                }
            }

            ESP_LOGW(GROUP_TAG, "Node 0x%04x not found in group 0x%04x", node_addr, group_addr);
            return ESP_ERR_NOT_FOUND;
        }
    }

    ESP_LOGW(GROUP_TAG, "Group 0x%04x not found in table", group_addr);
    return ESP_ERR_NOT_FOUND;
}

// ============================================================================
// Group table queries
// ============================================================================

uint8_t group_table_get_count(uint16_t group_addr)
{
    for (int i = 0; i < group_table.group_count; i++)
        if (group_table.groups[i].group_addr == group_addr)
            return group_table.groups[i].node_count;
    return 0;
}

GroupEntry_t *group_table_get_entry(uint16_t group_addr)
{
    for (int i = 0; i < group_table.group_count; i++)
        if (group_table.groups[i].group_addr == group_addr)
            return &group_table.groups[i];
    return NULL;
}

bool group_table_is_subscribed(uint16_t group_addr, uint16_t node_addr)
{
    GroupEntry_t *entry = group_table_get_entry(group_addr);
    if (entry == NULL)
        return false;
    for (int i = 0; i < entry->node_count; i++)
        if (entry->node_addrs[i] == node_addr)
            return true;
    return false;
}

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
        ESP_LOGI(GROUP_TAG, "Group 0x%04x (%d nodes):",
            group_table.groups[i].group_addr,
            group_table.groups[i].node_count);
        for (int j = 0; j < group_table.groups[i].node_count; j++)
            ESP_LOGI(GROUP_TAG, "  [%d] Node 0x%04x", j, group_table.groups[i].node_addrs[j]);
    }
    ESP_LOGI(GROUP_TAG, "==============================");
}