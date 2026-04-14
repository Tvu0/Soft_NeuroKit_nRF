#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>

int ble_init(void);
bool ble_is_connected(void);
bool ble_notifications_enabled(void);
int ble_send_ch34(int32_t ch3, int32_t ch4);

#endif