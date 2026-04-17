#include "ble.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>

/* ---------------- BLE STATE ---------------- */

static struct bt_conn *current_conn;
static bool notify_enabled;

/* ---------------- ADVERTISING DATA ---------------- */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ---------------- HELPER ---------------- */

static int start_advertising(void)
{
    int err;

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err == -EALREADY) {
        printk("Advertising already running\n");
        return 0;
    }

    if (err) {
        printk("Advertising failed: %d\n", err);
        return err;
    }

    printk("Advertising started\n");
    return 0;
}

/* ---------------- BLE CALLBACKS ---------------- */

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(eeg_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf1)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf2),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),

    BT_GATT_CCC(ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ---------------- CONNECTION CALLBACKS ---------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    if (current_conn) {
        bt_conn_unref(current_conn);
    }

    current_conn = bt_conn_ref(conn);
    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    printk("Disconnected (reason %u)\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    notify_enabled = false;

    /* Do NOT restart advertising here */
}

static void recycled(void)
{
    printk("Connection object recycled\n");
    start_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled,
};

/* ---------------- PUBLIC FUNCTIONS ---------------- */

int ble_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed: %d\n", err);
        return err;
    }

    printk("Bluetooth ready\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = start_advertising();
    if (err) {
        return err;
    }

    return 0;
}

bool ble_is_connected(void)
{
    return (current_conn != NULL);
}

bool ble_notifications_enabled(void)
{
    return notify_enabled;
}

int ble_send_ch34(int32_t ch3, int32_t ch4)
{
    uint8_t data[8];

    if (!current_conn) {
        return -ENOTCONN;
    }

    if (!bt_gatt_is_subscribed(current_conn, &eeg_svc.attrs[2], BT_GATT_CCC_NOTIFY)) {
        return -EACCES;
    }

    sys_put_be32((uint32_t)ch3, &data[0]);
    sys_put_be32((uint32_t)ch4, &data[4]);

    return bt_gatt_notify(current_conn, &eeg_svc.attrs[2], data, sizeof(data));
}