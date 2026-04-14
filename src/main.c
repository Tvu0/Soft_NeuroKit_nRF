#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

//#include "ads1299.h"
#include "ble.h"

int main(void)
{
    printk("MAIN STARTED\n");
    ble_init();
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
