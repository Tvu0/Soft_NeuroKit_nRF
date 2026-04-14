#include "ads1299.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <errno.h>

/* =========================================================
 * HARD-CODED PIN CONFIG
 * Sửa các số pin này theo mạch của bạn
 * ========================================================= */

/* SPI bus đang dùng: thường là spi1, spi2, spi3... */
#define ADS_SPI_NODE DT_NODELABEL(spi3)

/* SPI pins do board/pinctrl quyết định.
 * Ở bản hard-code đơn giản này, ta chỉ hard-code CS và GPIO điều khiển.
 * Nếu board của bạn chưa có SPI pinmux sẵn, lúc đó mới cần xử lý thêm.
 */

/* CS pin */
#define ADS_CS_PORT     DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define ADS_CS_PIN      17

/* DRDY / RESET / START / PWDN */
#define ADS_DRDY_PORT   DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define ADS_DRDY_PIN    18

#define ADS_RESET_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define ADS_RESET_PIN   19

#define ADS_START_PORT  DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define ADS_START_PIN   20

#define ADS_PWDN_PORT   DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define ADS_PWDN_PIN    21

/* =========================================================
 * ADS1299 commands
 * ========================================================= */
#define ADS_CMD_WAKEUP   0x02
#define ADS_CMD_STANDBY  0x04
#define ADS_CMD_RESET    0x06
#define ADS_CMD_START    0x08
#define ADS_CMD_STOP     0x0A
#define ADS_CMD_RDATAC   0x10
#define ADS_CMD_SDATAC   0x11
#define ADS_CMD_RDATA    0x12

/* =========================================================
 * ADS1299 register map
 * ========================================================= */
#define ADS_REG_ID           0x00
#define ADS_REG_CONFIG1      0x01
#define ADS_REG_CONFIG2      0x02
#define ADS_REG_CONFIG3      0x03
#define ADS_REG_LOFF         0x04
#define ADS_REG_CH1SET       0x05
#define ADS_REG_CH2SET       0x06
#define ADS_REG_CH3SET       0x07
#define ADS_REG_CH4SET       0x08
#define ADS_REG_BIAS_SENSP   0x0D
#define ADS_REG_BIAS_SENSN   0x0E
#define ADS_REG_MISC1        0x15
#define ADS_REG_CONFIG4      0x17

/* =========================================================
 * SPI config
 * ADS1299 dùng CPHA=1, CPOL=0 => Mode 1
 * ========================================================= */
static const struct device *spi_dev = DEVICE_DT_GET(ADS_SPI_NODE);

static struct spi_cs_control ads_cs_ctrl = {
    .gpio = {
        .port = ADS_CS_PORT,
        .pin = ADS_CS_PIN,
        .dt_flags = GPIO_ACTIVE_LOW,
    },
    .delay = 0,
};

static struct spi_config ads_spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,
    .cs = &ads_cs_ctrl,
};

/* =========================================================
 * GPIO devices
 * ========================================================= */
static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static struct gpio_callback drdy_cb_data;
static volatile bool drdy_flag = false;

/* 15 bytes = 3 status + 4 channels * 3 bytes */
static uint8_t frame_buf[15];

/* =========================================================
 * GPIO helpers
 * ========================================================= */
static inline int ads_reset_set(int value)
{
    return gpio_pin_set(ADS_RESET_PORT, ADS_RESET_PIN, value);
}

static inline int ads_start_set(int value)
{
    return gpio_pin_set(ADS_START_PORT, ADS_START_PIN, value);
}

static inline int ads_pwdn_set(int value)
{
    return gpio_pin_set(ADS_PWDN_PORT, ADS_PWDN_PIN, value);
}

/* =========================================================
 * DRDY interrupt
 * ========================================================= */
static void drdy_handler(const struct device *port,
                         struct gpio_callback *cb,
                         uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    drdy_flag = true;
}

/* =========================================================
 * SPI helpers
 * ========================================================= */
static int ads_spi_write(const uint8_t *tx_data, size_t len)
{
    const struct spi_buf tx_buf = {
        .buf = (void *)tx_data,
        .len = len,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    return spi_write(spi_dev, &ads_spi_cfg, &tx);
}

static int ads_spi_transceive(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    const struct spi_buf tx_buf = {
        .buf = (void *)tx_data,
        .len = len,
    };
    const struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = len,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    return spi_transceive(spi_dev, &ads_spi_cfg, &tx, &rx);
}

static int ads_send_cmd(uint8_t cmd)
{
    int ret = ads_spi_write(&cmd, 1);
    k_busy_wait(5);
    return ret;
}

/* Gộp header + payload thành 1 transaction */
static int ads_wreg(uint8_t addr, const uint8_t *data, uint8_t count)
{
    uint8_t tx[2 + 16];

    if (count > 16) {
        return -EINVAL;
    }

    tx[0] = 0x40 | (addr & 0x1F);
    tx[1] = (count - 1) & 0x1F;
    memcpy(&tx[2], data, count);

    int ret = ads_spi_write(tx, 2 + count);
    k_busy_wait(5);
    return ret;
}

/* Gộp thành 1 transaction */
static int ads_rreg(uint8_t addr, uint8_t *data, uint8_t count)
{
    uint8_t tx[2 + 16];
    uint8_t rx[2 + 16];

    if (count > 16) {
        return -EINVAL;
    }

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = 0x20 | (addr & 0x1F);
    tx[1] = (count - 1) & 0x1F;

    int ret = ads_spi_transceive(tx, rx, 2 + count);
    if (ret) {
        return ret;
    }

    memcpy(data, &rx[2], count);
    k_busy_wait(5);
    return 0;
}

static int ads_read_frame(uint8_t *data15)
{
    uint8_t tx[15] = {0};
    return ads_spi_transceive(tx, data15, 15);
}

/* =========================================================
 * Data convert
 * ========================================================= */
static int32_t int24_to_int32(uint8_t b0, uint8_t b1, uint8_t b2)
{
    int32_t v = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    if (v & 0x00800000) {
        v |= 0xFF000000;
    }
    return v;
}

/* =========================================================
 * HW init
 * ========================================================= */
static int ads_hw_init(void)
{
    int ret;

    if (!device_is_ready(spi_dev)) {
        return -ENODEV;
    }

    if (!device_is_ready(gpio0_dev)) {
        return -ENODEV;
    }

    if (!device_is_ready(ADS_CS_PORT)     ||
        !device_is_ready(ADS_DRDY_PORT)   ||
        !device_is_ready(ADS_RESET_PORT)  ||
        !device_is_ready(ADS_START_PORT)  ||
        !device_is_ready(ADS_PWDN_PORT)) {
        return -ENODEV;
    }

    ret = gpio_pin_configure(ADS_CS_PORT, ADS_CS_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure(ADS_RESET_PORT, ADS_RESET_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure(ADS_START_PORT, ADS_START_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure(ADS_PWDN_PORT, ADS_PWDN_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure(ADS_DRDY_PORT, ADS_DRDY_PIN, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure(ADS_DRDY_PORT, ADS_DRDY_PIN, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    gpio_init_callback(&drdy_cb_data, drdy_handler, BIT(ADS_DRDY_PIN));
    ret = gpio_add_callback(ADS_DRDY_PORT, &drdy_cb_data);
    if (ret) return ret;

    return 0;
}

static void ads_hw_reset(void)
{
    ads_pwdn_set(1);
    ads_start_set(0);
    ads_reset_set(1);

    k_msleep(10);

    ads_reset_set(0);
    k_busy_wait(10);
    ads_reset_set(1);

    k_msleep(1);
}

/* =========================================================
 * Public init
 * Channel 3,4 with SRB1 reference
 * ========================================================= */
int ads1299_init(void)
{
    int ret = ads_hw_init();
    if (ret) {
        return ret;
    }

    ads_hw_reset();

    /* Wake-up default là RDATAC => phải stop trước khi WREG */
    ret = ads_send_cmd(ADS_CMD_SDATAC);
    if (ret) return ret;

    /* CONFIG1 = 0x96 => 250 SPS
       CONFIG2 = 0xC0
       CONFIG3 = 0xE0
    */
    uint8_t cfg1_3[] = {
        0x96,
        0xC0,
        0xE0
    };
    ret = ads_wreg(ADS_REG_CONFIG1, cfg1_3, sizeof(cfg1_3));
    if (ret) return ret;

    /* CH1 off, CH2 off, CH3 on normal, CH4 on normal
       0xE1 = power down + shorted
       0x60 = gain 24 + normal electrode input
    */
    uint8_t chset[] = {
        0xE1,
        0xE1,
        0x60,
        0x60
    };
    ret = ads_wreg(ADS_REG_CH1SET, chset, sizeof(chset));
    if (ret) return ret;

    /* Nếu đã có bias loop hoàn chỉnh thì giữ 0x18/0x00
       Nếu chưa chắc, có thể đổi thành 0x00/0x00 để bring-up trước
    */
    uint8_t bias_regs[] = {
        0x18,
        0x00
    };
    ret = ads_wreg(ADS_REG_BIAS_SENSP, bias_regs, sizeof(bias_regs));
    if (ret) return ret;

    /* MISC1.SRB1 = 1 => SRB1 nối tới all inverting inputs */
    uint8_t misc1 = 0x20;
    ret = ads_wreg(ADS_REG_MISC1, &misc1, 1);
    if (ret) return ret;

    uint8_t cfg4 = 0x00;
    ret = ads_wreg(ADS_REG_CONFIG4, &cfg4, 1);
    if (ret) return ret;

    /* Có thể đọc lại để verify */
    /*
    uint8_t verify[8];
    ret = ads_rreg(ADS_REG_CONFIG1, verify, 8);
    if (ret) return ret;
    */

    ret = ads_send_cmd(ADS_CMD_RDATAC);
    if (ret) return ret;

    ads_start_set(1);
    k_msleep(1);

    return 0;
}

/* =========================================================
 * Read CH3 and CH4
 * ========================================================= */
bool ads1299_read_ch34(int32_t *ch3, int32_t *ch4)
{
    int ret;

    if (!drdy_flag) {
        return false;
    }

    drdy_flag = false;

    ret = ads_read_frame(frame_buf);
    if (ret) {
        return false;
    }

    /* [0..2] status
       [3..5] CH1
       [6..8] CH2
       [9..11] CH3
       [12..14] CH4
    */
    if (ch3) {
        *ch3 = int24_to_int32(frame_buf[9], frame_buf[10], frame_buf[11]);
    }

    if (ch4) {
        *ch4 = int24_to_int32(frame_buf[12], frame_buf[13], frame_buf[14]);
    }

    return true;
}