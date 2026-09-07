/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <main.h>

#if(IS_GWY)

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "ble_mesh_example_nvs.h"
#include "esp_ble_mesh_local_data_operation_api.h"   // ← ADD for model subscribe/unsubscribe

#include <flash.h>
#include <ble_new.h>
#include <lte.h>
#include <led.h>
#include "group_table.h"

bool ble_initialized = false;

#define BLE_TAG "BLE"

#define CID_ESP             0x02E5
#define MSG_SEND_TTL        3
#define MSG_SEND_REL        false
#define MSG_TIMEOUT         0
#define MSG_ROLE            ROLE_PROVISIONER

#define COMP_DATA_PAGE_0    0x00

#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12

#define COMP_DATA_1_OCTET(msg, offset)      (msg[offset])
#define COMP_DATA_2_OCTET(msg, offset)      (msg[offset + 1] << 8 | msg[offset])

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN];

static struct example_info_store {
    uint16_t server_addr;   /* Vendor server unicast address */
    uint16_t vnd_tid;       /* TID contained in the vendor message */
} store = {
    .server_addr = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .vnd_tid = 0,
};

static nvs_handle_t NVS_HANDLE;
static const char * NVS_KEY = "vendor_client";

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[ESP_BLE_MESH_OCTET16_LEN];
} prov_key;

static esp_ble_mesh_cfg_srv_t config_server = {
    .beacon = ESP_BLE_MESH_BEACON_DISABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_client_t config_client;

static const esp_ble_mesh_client_op_pair_t vnd_op_pair[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_SEND, ESP_BLE_MESH_VND_MODEL_OP_STATUS },
};

static esp_ble_mesh_client_t vendor_client = {
    .op_pair_size = ARRAY_SIZE(vnd_op_pair),
    .op_pair = vnd_op_pair,
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
    vnd_op, NULL, &vendor_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid          = dev_uuid,
    .prov_unicast_addr  = PROV_OWN_ADDR,
    .prov_start_address = 0x0005,
};

static void mesh_example_info_store(void)
{
    ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
}

static void mesh_example_info_restore(void)
{
    esp_err_t err = ESP_OK;
    bool exist = false;

    err = ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), &exist);
    if (err != ESP_OK) {
        return;
    }

    if (exist) {
        ESP_LOGI(BLE_TAG, "Restore, server_addr 0x%04x, vnd_tid 0x%04x", store.server_addr, store.vnd_tid);
    }
}

static void example_ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                            esp_ble_mesh_node_t *node,
                                            esp_ble_mesh_model_t *model, uint32_t opcode)
{
    common->opcode = opcode;
    common->model = model;
    common->ctx.net_idx = prov_key.net_idx;
    common->ctx.app_idx = prov_key.app_idx;
    common->ctx.addr = node->unicast_addr;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->ctx.send_rel = MSG_SEND_REL;
    common->msg_timeout = MSG_TIMEOUT;
    common->msg_role = MSG_ROLE;
}

static esp_err_t prov_complete(uint16_t node_index, const esp_ble_mesh_octet16_t uuid,
                               uint16_t primary_addr, uint8_t element_num, uint16_t net_idx)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get = {0};
    esp_ble_mesh_node_t *node = NULL;
    char name[10] = {'\0'};
    esp_err_t err;

    ESP_LOGI(BLE_TAG, "node_index %u, primary_addr 0x%04x, element_num %u, net_idx 0x%03x",
        node_index, primary_addr, element_num, net_idx);
    ESP_LOG_BUFFER_HEX("uuid", uuid, ESP_BLE_MESH_OCTET16_LEN);

    store.server_addr = primary_addr;
    mesh_example_info_store(); /* Store proper mesh example info */

    sprintf(name, "%s%02x", "NODE-", node_index);
    err = esp_ble_mesh_provisioner_set_node_name(node_index, name);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to set node name");
        return ESP_FAIL;
    }

    node = esp_ble_mesh_provisioner_get_node_with_addr(primary_addr);
    if (node == NULL) {
        ESP_LOGE(BLE_TAG, "Failed to get node 0x%04x info", primary_addr);
        return ESP_FAIL;
    }

    example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to send Config Composition Data Get");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    esp_err_t err;

    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner will only
     * use this callback to report the devices, whose device UUID starts with 0xdd & 0xdd,
     * to the application layer.
     */

    ESP_LOG_BUFFER_HEX("Device address", addr, BD_ADDR_LEN);
    ESP_LOGI(BLE_TAG, "Address type 0x%02x, adv type 0x%02x", addr_type, adv_type);
    ESP_LOG_BUFFER_HEX("Device UUID", dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    ESP_LOGI(BLE_TAG, "oob info 0x%04x, bearer %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (uint8_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (uint8_t)bearer;
    /* Note: If unprovisioned device adv packets have not been received, we should not add
             device with ADD_DEV_START_PROV_NOW_FLAG set. */
    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
            ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to start provisioning device");
    }
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        mesh_example_info_restore(); /* Restore proper mesh example info */
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT, bearer %s",
            param->provisioner_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT, bearer %s, reason 0x%02x",
            param->provisioner_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                      param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                      param->provisioner_prov_complete.netkey_idx);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code %d", param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d", param->provisioner_set_node_name_comp.err_code);
        if (param->provisioner_set_node_name_comp.err_code == 0) {
            const char *name = esp_ble_mesh_provisioner_get_node_name(param->provisioner_set_node_name_comp.node_index);
            if (name) {
                ESP_LOGI(BLE_TAG, "Node %d name %s", param->provisioner_set_node_name_comp.node_index, name);
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == 0) {
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            esp_err_t err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                    ESP_BLE_MESH_VND_MODEL_ID_CLIENT, CID_ESP);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to bind AppKey to vendor client");
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT:
        ESP_LOGI(BLE_TAG, "ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT, err_code %d", param->provisioner_store_node_comp_data_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_parse_node_comp_data(const uint8_t *data, uint16_t length)
{
    uint16_t cid, pid, vid, crpl, feat;
    uint16_t loc, model_id, company_id;
    uint8_t nums, numv;
    uint16_t offset;
    int i;

    cid = COMP_DATA_2_OCTET(data, 0);
    pid = COMP_DATA_2_OCTET(data, 2);
    vid = COMP_DATA_2_OCTET(data, 4);
    crpl = COMP_DATA_2_OCTET(data, 6);
    feat = COMP_DATA_2_OCTET(data, 8);
    offset = 10;

    ESP_LOGI(BLE_TAG, "********************** Composition Data Start **********************");
    ESP_LOGI(BLE_TAG, "* CID 0x%04x, PID 0x%04x, VID 0x%04x, CRPL 0x%04x, Features 0x%04x *", cid, pid, vid, crpl, feat);
    for (; offset < length; ) {
        loc = COMP_DATA_2_OCTET(data, offset);
        nums = COMP_DATA_1_OCTET(data, offset + 2);
        numv = COMP_DATA_1_OCTET(data, offset + 3);
        offset += 4;
        ESP_LOGI(BLE_TAG, "* Loc 0x%04x, NumS 0x%02x, NumV 0x%02x *", loc, nums, numv);
        for (i = 0; i < nums; i++) {
            model_id = COMP_DATA_2_OCTET(data, offset);
            ESP_LOGI(BLE_TAG, "* SIG Model ID 0x%04x *", model_id);
            offset += 2;
        }
        for (i = 0; i < numv; i++) {
            company_id = COMP_DATA_2_OCTET(data, offset);
            model_id = COMP_DATA_2_OCTET(data, offset + 2);
            ESP_LOGI(BLE_TAG, "* Vendor Model ID 0x%04x, Company ID 0x%04x *", model_id, company_id);
            offset += 4;
        }
    }
    ESP_LOGI(BLE_TAG, "*********************** Composition Data End ***********************");
}

static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                              esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    esp_ble_mesh_node_t *node = NULL;
    esp_err_t err;

    ESP_LOGI(BLE_TAG, "Config client, err_code %d, event %u, addr 0x%04x, opcode 0x%04" PRIx32,
        param->error_code, event, param->params->ctx.addr, param->params->opcode);

    if (param->error_code) {
        ESP_LOGE(BLE_TAG, "Send config client message failed, opcode 0x%04" PRIx32, param->params->opcode);
        return;
    }

    node = esp_ble_mesh_provisioner_get_node_with_addr(param->params->ctx.addr);
    if (!node) {
        ESP_LOGE(BLE_TAG, "Failed to get node 0x%04x info", param->params->ctx.addr);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET) {
            ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            example_ble_mesh_parse_node_comp_data(param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            err = esp_ble_mesh_provisioner_store_node_comp_data(param->params->ctx.addr,
                param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to store node composition data");
                break;
            }

            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to send Config AppKey Add");
            }
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = ESP_BLE_MESH_VND_MODEL_ID_SERVER;
            set.model_app_bind.company_id = CID_ESP;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to send Config Model App Bind");
            }
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            ESP_LOGW(BLE_TAG, "%s, Provision and config successfully", __func__);
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS) {
            ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
            esp_ble_mesh_cfg_client_get_state_t get = {0};
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to send Config Composition Data Get");
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to send Config AppKey Add");
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = ESP_BLE_MESH_VND_MODEL_ID_SERVER;
            set.model_app_bind.company_id = CID_ESP;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                ESP_LOGE(BLE_TAG, "Failed to send Config Model App Bind");
            }
            break;
        default:
            break;
        }
        break;
    default:
        ESP_LOGE(BLE_TAG, "Invalid config client event %u", event);
        break;
    }
}

void example_ble_mesh_send_vendor_message(bool resend)
{
    esp_ble_mesh_msg_ctx_t ctx = {0};
    uint32_t opcode;
    esp_err_t err;

    ctx.net_idx = prov_key.net_idx;
    ctx.app_idx = prov_key.app_idx;
    ctx.addr = store.server_addr;
    ctx.send_ttl = MSG_SEND_TTL;
    ctx.send_rel = MSG_SEND_REL;
    opcode = ESP_BLE_MESH_VND_MODEL_OP_SEND;

    if (resend == false) {
        store.vnd_tid++;
    }

    err = esp_ble_mesh_client_model_send_msg(vendor_client.model, &ctx, opcode,
            sizeof(store.vnd_tid), (uint8_t *)&store.vnd_tid, MSG_TIMEOUT, true, MSG_ROLE);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to send vendor message 0x%06" PRIx32, opcode);
        return;
    }

    mesh_example_info_store(); /* Store proper mesh example info */
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    static int64_t start_time;

    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_STATUS) {
            int64_t end_time = esp_timer_get_time();
            ESP_LOGI(BLE_TAG, "Recv 0x06%" PRIx32 ", tid 0x%04x, time %lldus",
            param->model_operation.opcode, store.vnd_tid, end_time - start_time);
            handle_ble_incoming(param);
        }
        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(BLE_TAG, "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);

            break;
        }
        start_time = esp_timer_get_time();
        ESP_LOGI(BLE_TAG, "Send 0x%06" PRIx32, param->model_send_comp.opcode);  
        break;

    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        // ESP_LOGI(BLE_TAG, "Receive publish message 0x%06" PRIx32, param->client_recv_publish_msg.opcode);
        handle_ble_incoming(param);
        break;

    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        ESP_LOGW(BLE_TAG, "Client message 0x%06" PRIx32 " timeout", param->client_send_timeout.opcode);
        example_ble_mesh_send_vendor_message(true);
        break;

    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = { 0x32, 0x10 };
    esp_err_t err;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_client_model_init(&vnd_models[0]);
    if (err) {
        ESP_LOGE(BLE_TAG, "Failed to initialize vendor client");
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to set matching device uuid");
        return err;
    }

    err = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to enable mesh provisioner");
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to add local AppKey");
        return err;
    }

    ESP_LOGI(BLE_TAG, "ESP BLE Mesh Provisioner initialized");

    return ESP_OK;
}

void ble_init(void)
{
    esp_err_t err;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Open nvs namespace for storing/restoring mesh example info */
    err = ble_mesh_nvs_open(&NVS_HANDLE);
    if (err) {
        return;
    }

    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}

/**
 * =========================================================================
 * @brief FUNCTIONS BELOW THIS ARE CUSTOM MADE - NOT THE PART OF BLE EXAMPLE
 * =========================================================================
 */

/**
 * @brief Function that handles the incoming ble messages from Nodes
 * @param param 
 */
void handle_ble_incoming(esp_ble_mesh_model_cb_param_t *param)
{
    int8_t rssi = param->model_operation.ctx->recv_rssi;
    ESP_LOGI(BLE_TAG, "Incoming BLE msg RSSI: %d dBm", rssi);
    teaching_mode *teach_ack = (teaching_mode *)param->model_operation.msg;
    if(teach_ack->packetid == NODE_TEACHING_MODE) {
        generate_node_teaching_mode_ack(teach_ack);
        if(teach_ack->msgseqno != BUTTON_PRESS_MSGSEQNO) removeQueueItemByMsgSeqNo(command_queue, teach_ack->msgseqno);
        return;
    }

    manual_control *manual_control_ack = (manual_control *)param->model_operation.msg;
    if(manual_control_ack->packetid == NODE_MANUAL_AC_CONTROL_ACK) {
        manual_control_ack->rssi = rssi;
        generate_node_manual_ac_control_ack(manual_control_ack);
        return;
    }

    CommandStruct *ack = (CommandStruct *)param->model_operation.msg;
    ack->rssi = rssi;
    ESP_LOGI(BLE_TAG, "Incoming BLE Msg: packetid=0x%04x, elemaddr=0x%04x, groupaddr=0x%04x, msgseqno=%d, errorcode=%d",
        ack->packetid, ack->elemaddr, ack->groupaddr, ack->msgseqno, ack->errorcode);
    if (ack->packetid == NODE_GROUP_SUB_ACK)
    {
        ESP_LOGI(BLE_TAG, "Node 0x%04x group subscribe ACK — group 0x%04x err=%d",
            ack->elemaddr, ack->groupaddr, ack->errorcode);
        if (ack->errorcode == SUCCESS)
        {
            group_table_subscribe(ack->groupaddr, ack->elemaddr);
            ESP_LOGI(BLE_TAG, "Group 0x%04x now has %d members",
                ack->groupaddr, group_table_get_count(ack->groupaddr));
        }
        generate_ack(NODE_GROUP_SUB_ACK, ack);
        if (ack->msgseqno != BUTTON_PRESS_MSGSEQNO)
            removeQueueItemByMsgSeqNo(command_queue, ack->msgseqno);
        return;
    }

    // ADD: Handle node group unsubscribe ACK
    if (ack->packetid == NODE_GROUP_UNSUB_ACK)
    {
        ESP_LOGI(BLE_TAG, "Node 0x%04x group unsubscribe ACK — group 0x%04x err=%d",
            ack->elemaddr, ack->groupaddr, ack->errorcode);
        if (ack->errorcode == SUCCESS)
        {
            group_table_unsubscribe(ack->groupaddr, ack->elemaddr);
            ESP_LOGI(BLE_TAG, "Group 0x%04x now has %d members",
                ack->groupaddr, group_table_get_count(ack->groupaddr));
        }
        generate_ack(NODE_GROUP_UNSUB_ACK, ack);
        if (ack->msgseqno != BUTTON_PRESS_MSGSEQNO)
            removeQueueItemByMsgSeqNo(command_queue, ack->msgseqno);
        return;
    }

    // ADD: Handle node group AC control ACK — update tracker
    if (ack->packetid == NODE_GROUP_AC_CONTROL_PACKET)
    {
        ESP_LOGI(BLE_TAG, "Node 0x%04x group AC ACK — seq=%d group=0x%04x err=%d",
            ack->elemaddr, ack->group_cmd_seq, ack->groupaddr, ack->errorcode);

        GroupAckTracker_t *tracker = group_tracker_find_by_seq(ack->group_cmd_seq);
        if (tracker == NULL)
        {
            ESP_LOGW(BLE_TAG, "No active tracker for seq=%d — stale ACK ignored",
                ack->group_cmd_seq);
            return;
        }

        bool all_done = group_tracker_record_ack(tracker, ack->elemaddr, ack->errorcode);
        if (all_done)
        {
            generate_ack(NODE_GROUP_AC_CONTROL_SUMMARY_ACK, &tracker->cmd);
            group_tracker_free(tracker);   // free after ACK enqueued
        }
        return;
    }

    if (ack->packetid == NODE_AC_CONTROL_PACKET)
        generate_ack(NODE_AC_CONTROL_ACK, ack);
    else
        generate_ack(ack->packetid, ack);
    if(ack->packetid == NODE_UNPROV_PACKET) {
        esp_err_t err = esp_ble_mesh_provisioner_delete_node_with_addr(ack->elemaddr);
        if(err) ESP_LOGE(BLE_TAG, "Failed to remove Node(elemAddr:%d) from database - %s",ack->elemaddr, esp_err_to_name(err));
    }
    if(ack->msgseqno != BUTTON_PRESS_MSGSEQNO) 
        removeQueueItemByMsgSeqNo(command_queue, ack->msgseqno);
}

void handle_gwy_group_subscribe(CommandStruct *cmd)
{
    ESP_LOGW(LTE_TAG, "GWY subscribing itself to group 0x%04x", cmd->groupaddr);

    // Gateway subscribes its own vendor CLIENT model to the group address
    esp_err_t err = esp_ble_mesh_model_subscribe_group_addr(
        PROV_OWN_ADDR,                      // gateway primary element address
        CID_ESP,
        ESP_BLE_MESH_VND_MODEL_ID_CLIENT,   // gateway uses vendor CLIENT model
        cmd->groupaddr
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(LTE_TAG, "GWY group subscribe failed: %s", esp_err_to_name(err));
        cmd->errorcode = FAILURE;
    }
    else
    {
        ESP_LOGI(LTE_TAG, "GWY subscribed to group 0x%04x", cmd->groupaddr);
        group_table_subscribe(cmd->groupaddr, PROV_OWN_ADDR);
        cmd->errorcode = SUCCESS;
    }
    generate_ack(GWY_GROUP_SUB_ACK, cmd);
}

void handle_gwy_group_unsubscribe(CommandStruct *cmd)
{
    ESP_LOGW(LTE_TAG, "GWY unsubscribing itself from group 0x%04x", cmd->groupaddr);

    esp_err_t err = esp_ble_mesh_model_unsubscribe_group_addr(
        PROV_OWN_ADDR,
        CID_ESP,
        ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
        cmd->groupaddr
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(LTE_TAG, "GWY group unsubscribe failed: %s", esp_err_to_name(err));
        cmd->errorcode = FAILURE;
    }
    else
    {
        ESP_LOGI(LTE_TAG, "GWY unsubscribed from group 0x%04x", cmd->groupaddr);
        group_table_unsubscribe(cmd->groupaddr, PROV_OWN_ADDR);
        cmd->errorcode = SUCCESS;
    }
    generate_ack(GWY_GROUP_UNSUB_ACK, cmd);
}

/**
 * @brief Send group AC control command to all nodes subscribed to group address
 *        Uses unacknowledged vendor model message to the group address
 *        All subscribed nodes receive and execute simultaneously
 * @param cmd  CommandStruct with groupaddr and AC control fields populated
 */
void ble_send_group_ac_control(CommandStruct *cmd)
{
    esp_ble_mesh_msg_ctx_t ctx = {
        .addr     = cmd->groupaddr,      // GROUP address — not unicast
        .app_idx  = prov_key.app_idx,
        .net_idx  = prov_key.net_idx,
        .send_ttl = MSG_SEND_TTL,
        .send_rel = MSG_SEND_REL,
    };

    cmd->packetid = NODE_GROUP_AC_CONTROL_PACKET;

    ESP_LOGW(BLE_TAG, "Sending group AC control to 0x%04x (%d nodes)",
        cmd->groupaddr, group_table_get_count(cmd->groupaddr));

    esp_err_t err = esp_ble_mesh_client_model_send_msg(
        &vnd_models[0], &ctx,
        ESP_BLE_MESH_VND_MODEL_OP_SEND,
        sizeof(CommandStruct), (uint8_t *)cmd,
        MSG_TIMEOUT, false,              // unacknowledged — group msgs cannot be acked in BLE Mesh spec
        ROLE_PROVISIONER);

    if (err != ESP_OK)
        ESP_LOGE(BLE_TAG, "Failed to send group AC control: %s", esp_err_to_name(err));
}

/**
 * @brief Function that takes are of sending commands to Nodes
 * @param cmd 
 */
void send_cmd_to_node(CommandStruct *cmd)
{
    esp_ble_mesh_msg_ctx_t ctx = {
        .addr = cmd->elemaddr,
        .app_idx = prov_key.app_idx,
        .net_idx = prov_key.net_idx,
        .send_ttl = 3,
        .send_rel = false,
    };
    switch(cmd->packetid)
    {
        case NODE_UNPROV_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Node Unprov packet");
            break;

        case NODE_AC_CONTROL_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Node AC Control packet");
            break;
        
        case NODE_RECONF_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Node Reconf packet");
            break;
        
        case NODE_HEARTBEAT_PUB_CONF_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Node HeartBeat Publish Configuration packet");
            break;

        case NODE_TEACHING_MODE:
            ESP_LOGI(BLE_TAG, "Sending Node Teaching Mode packet");
            break;
        
        case NODE_TEACHING_MODE_CMD_SELECTION_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Node Teaching Mode Cmd Selection packet");
            break;
        
        case NODE_DEBUG_INFO_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Node Debug Info packet");
            break;

        case NODE_GROUP_SUB_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Group Subscribe to node 0x%04x for group 0x%04x",
                cmd->elemaddr, cmd->groupaddr);
            break;

        case NODE_GROUP_UNSUB_PACKET:
            ESP_LOGI(BLE_TAG, "Sending Group Unsubscribe to node 0x%04x for group 0x%04x",
                cmd->elemaddr, cmd->groupaddr);
            break;

        default:
            ESP_LOGE(BLE_TAG, "Unknown cmd type : %d in %s", cmd->packetid, __func__);
            return;
    }

    cmd->reqSentToNodeTicks = xTaskGetTickCount();
    cmd->requestSentToNode = true;
    if (xQueueSend(command_queue, cmd, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(BLE_TAG, "Enqueing into Command Queue failed");
        cmd->errorcode = ENQUEUING_INTO_COMMAND_QUEUE_FAILED;
        led_set_state(LED_STATE_INVALID_OPERATION);
        generate_ack(cmd->packetid, cmd);
        return;
    }
    esp_ble_mesh_client_model_send_msg(&vnd_models[0], &ctx, ESP_BLE_MESH_VND_MODEL_OP_SEND, sizeof(CommandStruct), (uint8_t *)cmd, 10, true, ROLE_PROVISIONER);
}

#endif