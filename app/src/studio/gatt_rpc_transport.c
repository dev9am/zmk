/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <sys/types.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/ring_buffer.h>

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/studio/rpc.h>

#include "uuid.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk_studio, CONFIG_ZMK_STUDIO_LOG_LEVEL);

static bool handling_rx = false;

static atomic_t notify_size;

/* Track the BLE connection that is actively using Studio.
 * This allows Studio to work on a non-active-profile connection
 * (e.g. Web Bluetooth connecting independently from the HID host).
 *
 * Guarded by studio_conn_lock: the disconnected callback (BT RX thread) can
 * clear and unref the conn concurrently with readers on the system workqueue,
 * so readers must take their own reference under the lock. */
static struct bt_conn *studio_conn = NULL;
static struct k_spinlock studio_conn_lock;

static void studio_conn_set(struct bt_conn *conn) {
    struct bt_conn *prev;

    k_spinlock_key_t key = k_spin_lock(&studio_conn_lock);
    prev = studio_conn;
    studio_conn = conn ? bt_conn_ref(conn) : NULL;
    k_spin_unlock(&studio_conn_lock, key);

    if (prev) {
        bt_conn_unref(prev);
    }
}

/* Returns a referenced conn (or NULL); the caller must bt_conn_unref it. */
static struct bt_conn *studio_conn_get(void) {
    k_spinlock_key_t key = k_spin_lock(&studio_conn_lock);
    struct bt_conn *conn = studio_conn ? bt_conn_ref(studio_conn) : NULL;
    k_spin_unlock(&studio_conn_lock, key);

    return conn;
}

static void studio_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct bt_conn *prev = NULL;

    k_spinlock_key_t key = k_spin_lock(&studio_conn_lock);
    if (studio_conn == conn) {
        prev = studio_conn;
        studio_conn = NULL;
    }
    k_spin_unlock(&studio_conn_lock, key);

    if (prev) {
        LOG_INF("Studio connection disconnected");
        bt_conn_unref(prev);
    }
}

BT_CONN_CB_DEFINE(studio_conn_callbacks) = {
    .disconnected = studio_disconnected,
};

static void rpc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);

    bool notif_enabled = (value == BT_GATT_CCC_INDICATE);

    LOG_INF("RPC Notifications %s", notif_enabled ? "enabled" : "disabled");

#if CONFIG_ZMK_STUDIO_TRANSPORT_BLE_PREF_LATENCY < CONFIG_BT_PERIPHERAL_PREF_LATENCY
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (conn) {
        uint8_t latency = notif_enabled ? CONFIG_ZMK_STUDIO_TRANSPORT_BLE_PREF_LATENCY
                                        : CONFIG_BT_PERIPHERAL_PREF_LATENCY;

        int ret = bt_conn_le_param_update(
            conn,
            BT_LE_CONN_PARAM(CONFIG_BT_PERIPHERAL_PREF_MIN_INT, CONFIG_BT_PERIPHERAL_PREF_MAX_INT,
                             latency, CONFIG_BT_PERIPHERAL_PREF_TIMEOUT));
        if (ret < 0) {
            LOG_WRN("Failed to request lower latency while studio is active (%d)", ret);
        }

        bt_conn_unref(conn);
    }
#endif
}

static ssize_t read_rpc_resp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                             uint16_t len, uint16_t offset) {

    LOG_DBG("Read response for length %d at offset %d", len, offset);
    return 0;
}

static ssize_t write_rpc_req(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                             uint16_t len, uint16_t offset, uint8_t flags) {
    if (!handling_rx) {
        return len;
    }

    /* Remember which connection is talking to Studio */
    studio_conn_set(conn);

    uint32_t copied = 0;
    struct ring_buf *rpc_buf = zmk_rpc_get_rx_buf();
    while (copied < len) {
        uint8_t *buffer;
        uint32_t claim_len = ring_buf_put_claim(rpc_buf, &buffer, len - copied);

        if (claim_len > 0) {
            memcpy(buffer, ((uint8_t *)buf) + copied, claim_len);
            copied += claim_len;
        }

        ring_buf_put_finish(rpc_buf, claim_len);
    }

    zmk_rpc_rx_notify();

    return len;
}

BT_GATT_SERVICE_DEFINE(
    rpc_interface, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(ZMK_STUDIO_BT_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ZMK_STUDIO_BT_RPC_CHRC_UUID),
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_rpc_resp,
                           write_rpc_req, NULL),
    BT_GATT_CCC(rpc_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));

static uint16_t get_notify_size_for_conn(struct bt_conn *conn) {
    uint16_t notify_size = 23; // Default MTU size unless negotiated higher
    struct bt_conn_info conn_info;
    if (conn && bt_conn_get_info(conn, &conn_info) >= 0) {
        notify_size = conn_info.le.data_len->tx_max_len;
    }

    return notify_size;
}

static void refresh_notify_size(void) {
    struct bt_conn *conn = studio_conn_get();
    if (!conn) {
        conn = zmk_ble_active_profile_conn();
    }

    uint16_t ns = get_notify_size_for_conn(conn);
    if (conn) {
        bt_conn_unref(conn);
    }

    atomic_set(&notify_size, ns);
}

static int gatt_start_rx() {
    refresh_notify_size();
    handling_rx = true;
    return 0;
}

static int gatt_stop_rx(void) {
    handling_rx = false;
    return 0;
}

static struct bt_gatt_indicate_params rpc_indicate_params = {
    .attr = &rpc_interface.attrs[1],
};

static void notif_rpc_tx_cb(struct k_work *work) {
    /* Prefer the tracked Studio connection; fall back to active profile */
    struct bt_conn *conn = studio_conn_get();
    if (!conn) {
        conn = zmk_ble_active_profile_conn();
    }
    struct ring_buf *tx_buf = zmk_rpc_get_tx_buf();

    if (!conn) {
        LOG_WRN("No active connection for queued data, dropping");
        ring_buf_reset(tx_buf);
        return;
    }

    uint16_t notify_size = get_notify_size_for_conn(conn);
    uint8_t notify_bytes[notify_size];

    while (ring_buf_size_get(tx_buf) > 0) {
        uint16_t added = 0;
        while (added < notify_size && ring_buf_size_get(tx_buf) > 0) {
            uint8_t *buf;
            int len = ring_buf_get_claim(tx_buf, &buf, notify_size - added);

            memcpy(notify_bytes + added, buf, len);

            added += len;
            ring_buf_get_finish(tx_buf, len);
        }

        rpc_indicate_params.data = notify_bytes;
        rpc_indicate_params.len = added;

        int notify_attempts = 5;
        do {
            int err = bt_gatt_indicate(conn, &rpc_indicate_params);
            if (err >= 0) {
                break;
            }

            LOG_WRN("Failed to notify the response %d", err);
            k_sleep(K_MSEC(200));
        } while (notify_attempts-- > 0);
    }

    bt_conn_unref(conn);
}

static K_WORK_DEFINE(notify_tx_work, notif_rpc_tx_cb);

struct gatt_write_state {
    size_t pending_notify;
};

static void gatt_tx_notify(struct ring_buf *tx_buf, size_t added, bool msg_done, void *user_data) {
    struct gatt_write_state *state = (struct gatt_write_state *)user_data;

    state->pending_notify += added;

    atomic_t ns = atomic_get(&notify_size);

    if (msg_done || state->pending_notify > ns) {
        k_work_submit(&notify_tx_work);
        state->pending_notify = 0;
    }
}

static struct gatt_write_state tx_state = {};

static void *gatt_tx_user_data(void) {
    memset(&tx_state, sizeof(tx_state), 0);

    return &tx_state;
}

ZMK_RPC_TRANSPORT(gatt, ZMK_TRANSPORT_BLE, gatt_start_rx, gatt_stop_rx, gatt_tx_user_data,
                  gatt_tx_notify);

static int gatt_rpc_listener(const zmk_event_t *eh) {
    refresh_notify_size();

#if IS_ENABLED(CONFIG_ZMK_STUDIO_LOCK_ON_DISCONNECT)
    struct bt_conn *conn = zmk_ble_active_profile_conn();

    if (!conn) {
        zmk_studio_core_lock();
    } else {
        bt_conn_unref(conn);
    }
#endif

    return 0;
}

ZMK_LISTENER(gatt_rpc_listener, gatt_rpc_listener);
ZMK_SUBSCRIPTION(gatt_rpc_listener, zmk_ble_active_profile_changed);
