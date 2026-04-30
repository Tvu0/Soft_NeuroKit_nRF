#include "ads1299.h"

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define SPI1_NODE DT_NODELABEL(spi1)
#define USER_NODE DT_PATH(zephyr_user)

static const struct device *spi_dev = DEVICE_DT_GET(SPI1_NODE);

static const struct gpio_dt_spec reset_gpio =
    GPIO_DT_SPEC_GET(USER_NODE, reset_gpios);

static const struct gpio_dt_spec drdy_gpio =
    GPIO_DT_SPEC_GET(USER_NODE, drdy_gpios);

static const struct gpio_dt_spec start_gpio =
    GPIO_DT_SPEC_GET(USER_NODE, start_gpios);

static const struct gpio_dt_spec cs_gpio =
    GPIO_DT_SPEC_GET_BY_IDX(SPI1_NODE, cs_gpios, 0);

/*
 * ADS1299 SPI configuration.
 *
 * This is the mode that produced the correct baseline on your setup:
 *   ID      = 0x3E
 *   CONFIG1 = 0x96
 *   CONFIG2 = 0xC0
 *   CONFIG3 = 0x60
 *
 * If the readback appears shifted, the SPI mode is likely wrong.
 */
static const struct spi_config spi_cfg = {
    .frequency = 125000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,
};

/*
 * Raw GPIO control is used intentionally.
 *
 * RESETB is active-low:
 *   assert reset  = 0
 *   release reset = 1
 *
 * START is active-high:
 *   stop conversions/start low = 0
 *   start conversions          = 1
 *
 * CS is active-low:
 *   select ADS   = 0
 *   deselect ADS = 1
 */
static int ads_reset_assert(void)
{
    return gpio_pin_set_raw(reset_gpio.port, reset_gpio.pin, 0);
}

static int ads_reset_release(void)
{
    return gpio_pin_set_raw(reset_gpio.port, reset_gpio.pin, 1);
}

static int ads_start_low(void)
{
    return gpio_pin_set_raw(start_gpio.port, start_gpio.pin, 0);
}

static int ads_start_high(void)
{
    return gpio_pin_set_raw(start_gpio.port, start_gpio.pin, 1);
}

static int ads_cs_low(void)
{
    return gpio_pin_set_raw(cs_gpio.port, cs_gpio.pin, 0);
}

static int ads_cs_high(void)
{
    return gpio_pin_set_raw(cs_gpio.port, cs_gpio.pin, 1);
}

static void ads1299_print_pin_states(const char *label)
{
    printk("%s\n", label);

    printk("RESET raw=%d\n",
           gpio_pin_get_raw(reset_gpio.port, reset_gpio.pin));

    printk("START raw=%d\n",
           gpio_pin_get_raw(start_gpio.port, start_gpio.pin));

    printk("CS    raw=%d\n",
           gpio_pin_get_raw(cs_gpio.port, cs_gpio.pin));

    printk("DRDY  raw=%d\n",
           gpio_pin_get_raw(drdy_gpio.port, drdy_gpio.pin));
}

static int ads_spi_transceive(uint8_t *tx, uint8_t *rx, size_t len)
{
    int err;

    struct spi_buf tx_buf = {
        .buf = tx,
        .len = len,
    };

    struct spi_buf rx_buf = {
        .buf = rx,
        .len = len,
    };

    struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = ads_cs_low();
    if (err) {
        printk("CS low failed: %d\n", err);
        return err;
    }

    k_busy_wait(10);

    err = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);

    k_busy_wait(10);

    ads_cs_high();

    if (err) {
        printk("spi_transceive failed: %d\n", err);
    }

    return err;
}

int ads1299_send_command(uint8_t cmd)
{
    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = { 0 };
    int err;

    err = ads_spi_transceive(tx, rx, sizeof(tx));
    if (err) {
        printk("Command 0x%02X failed: %d\n", cmd, err);
        return err;
    }

    k_msleep(2);
    return 0;
}

int ads1299_read_reg(uint8_t reg, uint8_t *value)
{
    int err;

    if (value == NULL) {
        return -EINVAL;
    }

    /*
     * RREG command:
     *   byte 0 = 0x20 | register address
     *   byte 1 = number of registers minus 1
     *   byte 2 = dummy byte to clock out register value
     */
    uint8_t tx[3] = {
        (uint8_t)(ADS_CMD_RREG | (reg & 0x1F)),
        0x00,
        0x00
    };

    uint8_t rx[3] = { 0 };

    err = ads_spi_transceive(tx, rx, sizeof(tx));
    if (err) {
        printk("RREG 0x%02X failed: %d\n", reg, err);
        return err;
    }

    *value = rx[2];

    printk("RREG reg=0x%02X value=0x%02X\n", reg, *value);

    return 0;
}

int ads1299_write_reg(uint8_t reg, uint8_t value)
{
    int err;

    /*
     * WREG command:
     *   byte 0 = 0x40 | register address
     *   byte 1 = number of registers minus 1
     *   byte 2 = value to write
     */
    uint8_t tx[3] = {
        (uint8_t)(ADS_CMD_WREG | (reg & 0x1F)),
        0x00,
        value
    };

    uint8_t rx[3] = { 0 };

    err = ads_spi_transceive(tx, rx, sizeof(tx));
    if (err) {
        printk("WREG 0x%02X failed: %d\n", reg, err);
        return err;
    }

    printk("WREG reg=0x%02X value=0x%02X\n", reg, value);

    k_msleep(2);
    return 0;
}

int ads1299_read_id(uint8_t *id)
{
    return ads1299_read_reg(ADS_REG_ID, id);
}

int ads1299_get_drdy_raw(void)
{
    return gpio_pin_get_raw(drdy_gpio.port, drdy_gpio.pin);
}

int ads1299_start_conversions(void)
{
    int err;

    /*
     * START pin high enables conversion start.
     * Then START command and RDATAC put ADS1299 into continuous data output mode.
     */
    err = ads_start_high();
    if (err) {
        printk("START high failed: %d\n", err);
        return err;
    }

    k_msleep(2);

    err = ads1299_send_command(ADS_CMD_START);
    if (err) {
        return err;
    }

    k_msleep(2);

    err = ads1299_send_command(ADS_CMD_RDATAC);
    if (err) {
        return err;
    }

    printk("ADS1299 conversions started\n");
    return 0;
}

int ads1299_stop_conversions(void)
{
    int err;

    err = ads1299_send_command(ADS_CMD_SDATAC);
    if (err) {
        return err;
    }

    k_msleep(2);

    err = ads1299_send_command(ADS_CMD_STOP);
    if (err) {
        return err;
    }

    k_msleep(2);

    err = ads_start_low();
    if (err) {
        printk("START low failed: %d\n", err);
        return err;
    }

    printk("ADS1299 conversions stopped\n");
    return 0;
}

int ads1299_read_data_frame(uint8_t *data, size_t len)
{
    int err;

    if (data == NULL || len != 27) {
        return -EINVAL;
    }

    /*
     * ADS1299 RDATAC frame:
     *   3 status bytes + 8 channels * 3 bytes = 27 bytes
     *
     * Even if only CH3 is used, the ADS1299 still outputs the full frame.
     */
    uint8_t tx[27] = { 0 };

    struct spi_buf tx_buf = {
        .buf = tx,
        .len = sizeof(tx),
    };

    struct spi_buf rx_buf = {
        .buf = data,
        .len = len,
    };

    struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = ads_cs_low();
    if (err) {
        printk("CS low failed during data frame read: %d\n", err);
        return err;
    }

    k_busy_wait(10);

    err = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);

    k_busy_wait(10);

    ads_cs_high();

    if (err) {
        printk("Data frame read failed: %d\n", err);
        return err;
    }

    return 0;
}

int ads1299_init(void)
{
    int err;

    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&reset_gpio) ||
        !gpio_is_ready_dt(&drdy_gpio)  ||
        !gpio_is_ready_dt(&start_gpio) ||
        !gpio_is_ready_dt(&cs_gpio)) {
        printk("One or more GPIO devices not ready\n");
        return -ENODEV;
    }

    /*
     * OUTPUT | INPUT is used during bring-up/debug so raw pin state can be read.
     */
    err = gpio_pin_configure(reset_gpio.port,
                             reset_gpio.pin,
                             GPIO_OUTPUT | GPIO_INPUT);
    if (err) {
        printk("RESET configure failed: %d\n", err);
        return err;
    }

    err = gpio_pin_configure(start_gpio.port,
                             start_gpio.pin,
                             GPIO_OUTPUT | GPIO_INPUT);
    if (err) {
        printk("START configure failed: %d\n", err);
        return err;
    }

    err = gpio_pin_configure(cs_gpio.port,
                             cs_gpio.pin,
                             GPIO_OUTPUT | GPIO_INPUT);
    if (err) {
        printk("CS configure failed: %d\n", err);
        return err;
    }

    err = gpio_pin_configure(drdy_gpio.port,
                             drdy_gpio.pin,
                             GPIO_INPUT);
    if (err) {
        printk("DRDY configure failed: %d\n", err);
        return err;
    }

    /*
     * Safe idle state:
     *   RESETB high = released from reset
     *   START low   = conversions stopped
     *   CS high     = SPI deselected
     */
    ads_reset_release();
    ads_start_low();
    ads_cs_high();

    k_msleep(20);

    ads1299_print_pin_states("ADS1299 idle before reset:");

    /*
     * Hardware reset pulse.
     */
    ads_reset_assert();
    k_msleep(10);

    ads_reset_release();
    k_msleep(100);

    ads1299_print_pin_states("ADS1299 after hardware reset:");

    /*
     * Software reset and stop continuous data mode before register writes.
     */
    err = ads1299_send_command(ADS_CMD_RESET);
    if (err) {
        return err;
    }

    k_msleep(20);

    err = ads1299_send_command(ADS_CMD_SDATAC);
    if (err) {
        return err;
    }

    k_msleep(20);

    ads1299_print_pin_states("ADS1299 init complete:");

    return 0;
}