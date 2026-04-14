#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "ble.h"
#include "ads1299.h"

int main(void)
{
    int err;
    unsigned char ads_id = 0;

    printk("MAIN STARTED\n");

    err = ble_init();
    if (err) {
        printk("BLE init failed: %d\n", err);
        return 0;
    }

    err = ads1299_init();
    if (err) {
        printk("ADS1299 init failed: %d\n", err);
        return 0;
    }

    printk("ADS1299 init OK\n");
    printk("Before ads1299_read_id\n");

    err = ads1299_read_id(&ads_id);
    printk("After ads1299_read_id, err = %d\n", err);

    if (!err) {
        printk("ADS1299 ID = 0x%02X\n", ads_id);
    }

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}