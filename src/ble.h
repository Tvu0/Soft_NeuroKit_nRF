#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>

int ble_init(void);
bool ble_is_connected(void);
bool ble_notifications_enabled(void);
int ble_send_ch34(int32_t ch3, int32_t ch4);

/*
 * Notification state is used as the app's START/STOP command:
 *
 * Enable notifications  -> start_req = true
 * Disable notifications -> stop_req = true
 */
bool ble_start_requested(void);
bool ble_stop_requested(void);
void ble_clear_start_request(void);
void ble_clear_stop_request(void);

#endif