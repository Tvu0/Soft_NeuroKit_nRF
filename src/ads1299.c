#include "ads1299.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define SPI1_NODE DT_NODELABEL(spi1)
#define USER_NODE DT_PATH(zephyr_user)

static const struct device *spi_dev = DEVICE_DT_GET(SPI1_NODE);

static const struct gpio_dt_spec pwdn =
    GPIO_DT_SPEC_GET(USER_NODE, pwdn_gpios);

static const struct gpio_dt_spec cs_gpio =
    GPIO_DT_SPEC_GET_BY_IDX(SPI1_NODE, cs_gpios, 0);

static const struct spi_config spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,
};

int ads1299_init(void)
{
    int err;

    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&pwdn)) {
        printk("PWDN GPIO not ready\n");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&cs_gpio)) {
        printk("CS GPIO not ready\n");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&pwdn, GPIO_OUTPUT_ACTIVE);
    if (err) {
        printk("PWDN config failed: %d\n", err);
        return err;
    }

    err = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        printk("CS config failed: %d\n", err);
        return err;
    }

    k_msleep(10);
    return 0;
}

int ads1299_read_id(unsigned char *id)
{
    int err;
    unsigned char tx_buf[3] = {0x20, 0x00, 0x00};
    unsigned char rx_buf[3] = {0};

    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf),
    };

    struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = sizeof(rx_buf),
    };

    struct spi_buf_set tx_set = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };

    struct spi_buf_set rx_set = {
        .buffers = &rx_spi_buf,
        .count = 1,
    };

    printk("Before CS low\n");
    gpio_pin_set_dt(&cs_gpio, 0);
    k_busy_wait(2);

    printk("Before spi_transceive\n");
    err = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    printk("After spi_transceive, err = %d\n", err);

    k_busy_wait(2);
    gpio_pin_set_dt(&cs_gpio, 1);
    printk("After CS high\n");

    if (err) {
        printk("SPI read ID failed: %d\n", err);
        return err;
    }

    *id = rx_buf[2];
    return 0;
}