#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <errno.h>

#include "ads1299.h"
#include "ble.h"

/*
 * ECG acquisition settings.
 */
#define ADS_SAMPLE_RATE_HZ       500
#define ADS_NOTCH_FREQ_HZ        60.0f

/*
 * Print every N samples.
 *
 * Printing every sample at 500 SPS can slow down BLE streaming.
 * Set to 1 for full debug output.
 * Set to 10 or 50 for smoother BLE streaming.
 */
#define PRINT_EVERY_N_SAMPLES    1

/*
 * ADS1299 voltage conversion:
 *
 * voltage = raw_count * VREF / (gain * (2^23 - 1))
 *
 * This ECG bring-up uses gain 6 because gain 24 saturated too easily.
 */
#define ADS_VREF_VOLTS           4.5f
#define ADS_CH3_GAIN             6.0f
#define ADS_24BIT_FULL_SCALE     8388607.0f

/*
 * CHxSET values.
 *
 * 0x00 = gain 6, normal electrode input
 * 0x01 = gain 6, input shorted
 * 0x60 = gain 24, normal electrode input
 * 0x61 = gain 24, input shorted
 * 0x81 = channel powered down, input shorted
 */
#define ADS_CH3_GAIN6_NORMAL_INPUT    0x00
#define ADS_CH_POWERDOWN_SHORTED      0x81

/*
 * ADS1299 frame:
 *
 * 3 status bytes + 8 channels * 3 bytes = 27 bytes
 *
 * CH3 is:
 *   frame[9], frame[10], frame[11]
 */
static uint8_t frame[27];

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    float x1;
    float x2;
    float y1;
    float y2;
} biquad_t;

static biquad_t notch60;

static int32_t sign_extend_24(uint8_t b0, uint8_t b1, uint8_t b2)
{
    int32_t value = ((int32_t)b0 << 16) |
                    ((int32_t)b1 << 8)  |
                    ((int32_t)b2);

    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    return value;
}

static int32_t raw_to_microvolts_int(int32_t raw)
{
    float volts = ((float)raw * ADS_VREF_VOLTS) /
                  (ADS_CH3_GAIN * ADS_24BIT_FULL_SCALE);

    return (int32_t)(volts * 1000000.0f);
}

/*
 * 60 Hz notch filter.
 *
 * fs = 500 Hz
 * f0 = 60 Hz
 * Q = 30 gives a reasonably narrow notch.
 */
static void biquad_notch_init(biquad_t *filt, float fs_hz, float f0_hz, float q)
{
    float w0 = 2.0f * 3.14159265358979323846f * f0_hz / fs_hz;
    float c = cosf(w0);
    float s = sinf(w0);
    float alpha = s / (2.0f * q);

    float b0 = 1.0f;
    float b1 = -2.0f * c;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * c;
    float a2 = 1.0f - alpha;

    filt->b0 = b0 / a0;
    filt->b1 = b1 / a0;
    filt->b2 = b2 / a0;
    filt->a1 = a1 / a0;
    filt->a2 = a2 / a0;

    filt->x1 = 0.0f;
    filt->x2 = 0.0f;
    filt->y1 = 0.0f;
    filt->y2 = 0.0f;
}

static int32_t biquad_process_int_uv(biquad_t *filt, int32_t x_int)
{
    float x = (float)x_int;

    float y = filt->b0 * x +
              filt->b1 * filt->x1 +
              filt->b2 * filt->x2 -
              filt->a1 * filt->y1 -
              filt->a2 * filt->y2;

    filt->x2 = filt->x1;
    filt->x1 = x;

    filt->y2 = filt->y1;
    filt->y1 = y;

    return (int32_t)y;
}

/*
 * DRDY is active-low.
 *
 * This tries to wait for a fresh DRDY falling edge.
 * If DRDY stays low on the eval board, the caller falls back to a 2 ms delay,
 * which approximates 500 SPS.
 */
static int wait_for_drdy_falling_edge(int timeout_ms)
{
    int elapsed_us = 0;
    int timeout_us = timeout_ms * 1000;

    /*
     * Wait for DRDY to go high first.
     */
    while (ads1299_get_drdy_raw() == 0) {
        k_usleep(100);
        elapsed_us += 100;

        if (elapsed_us >= timeout_us) {
            return -1;
        }
    }

    /*
     * Then wait for DRDY to go low.
     */
    while (ads1299_get_drdy_raw() == 1) {
        k_usleep(100);
        elapsed_us += 100;

        if (elapsed_us >= timeout_us) {
            return -2;
        }
    }

    return 0;
}

/*
 * Configure ADS1299 for CH3 ECG:
 *
 *   500 SPS
 *   CH3 only
 *   gain 6
 *   normal electrode input
 *   BIAS enabled from CH3P and CH3N
 *   lead-off disabled for bring-up
 *
 * Electrode setup:
 *   CH3P -> left wrist
 *   CH3N -> right wrist
 *   BIAS -> third electrode connected to BIAS on JP25 or GND?
 *
 */
static int configure_ads1299_ch3_ecg_500sps_bias(void)
{
    int err;
    uint8_t check = 0;

    err = ads1299_send_command(ADS_CMD_SDATAC);
    if (err) {
        printk("SDATAC failed: %d\n", err);
        return err;
    }

    k_msleep(5);

    /*
     * CONFIG1 = 0x95 sets 500 SPS.
     */
    err = ads1299_write_reg(ADS_REG_CONFIG1, 0x95);
    if (err) {
        printk("CONFIG1 write failed: %d\n", err);
        return err;
    }

    /*
     * Power down unused channels.
     */
    err = ads1299_write_reg(ADS_REG_CH1SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    err = ads1299_write_reg(ADS_REG_CH2SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    /*
     * CH3 active, gain 6, normal electrode input.
     */
    err = ads1299_write_reg(ADS_REG_CH3SET, ADS_CH3_GAIN6_NORMAL_INPUT);
    if (err) return err;

    err = ads1299_write_reg(ADS_REG_CH4SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    err = ads1299_write_reg(ADS_REG_CH5SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    err = ads1299_write_reg(ADS_REG_CH6SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    err = ads1299_write_reg(ADS_REG_CH7SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    err = ads1299_write_reg(ADS_REG_CH8SET, ADS_CH_POWERDOWN_SHORTED);
    if (err) return err;

    /*
     * Disable lead-off during early testing.
     */
    ads1299_write_reg(ADS_REG_LOFF,       0x00);
    ads1299_write_reg(ADS_REG_LOFF_SENSP, 0x00);
    ads1299_write_reg(ADS_REG_LOFF_SENSN, 0x00);
    ads1299_write_reg(ADS_REG_LOFF_FLIP,  0x00);

    /*
     * Enable BIAS sensing from CH3P and CH3N.
     *
     * Bit 2 corresponds to channel 3.
     */
    ads1299_write_reg(ADS_REG_BIAS_SENSP, 0x04);
    ads1299_write_reg(ADS_REG_BIAS_SENSN, 0x04);

    /*
     * Keep SRB/common-reference routing disabled for this simple differential test.
     */
    ads1299_write_reg(ADS_REG_MISC1, 0x00);

    /*
     * Verify key settings.
     */
    ads1299_read_reg(ADS_REG_CONFIG1, &check);
    printk("CONFIG1 after 500SPS write = 0x%02X\n", check);

    ads1299_read_reg(ADS_REG_CH3SET, &check);
    printk("CH3SET = 0x%02X\n", check);

    ads1299_read_reg(ADS_REG_BIAS_SENSP, &check);
    printk("BIAS_SENSP = 0x%02X\n", check);

    ads1299_read_reg(ADS_REG_BIAS_SENSN, &check);
    printk("BIAS_SENSN = 0x%02X\n", check);

    ads1299_read_reg(ADS_REG_LOFF, &check);
    printk("LOFF = 0x%02X\n", check);

    return 0;
}

/*
 * Read one CH3 sample.
 *
 * Output:
 *   ch3_raw_out         = signed 24-bit ADS1299 ADC count
 *   ch3_filtered_uv_out = CH3 converted to microvolts after 60 Hz notch
 */
static int read_ch3_sample(int32_t *ch3_raw_out, int32_t *ch3_filtered_uv_out)
{
    int err;

    if (ch3_raw_out == NULL || ch3_filtered_uv_out == NULL) {
        return -EINVAL;
    }

    err = wait_for_drdy_falling_edge(1000);

    if (err) {
        /*
         * Fallback timing:
         * 2 ms approximates 500 SPS.
         */
        k_msleep(2);
    }

    err = ads1299_read_data_frame(frame, sizeof(frame));
    if (err) {
        return err;
    }

    /*
     * CH3 is bytes 9, 10, 11 in the 27-byte ADS1299 frame.
     */
    int32_t ch3_raw = sign_extend_24(frame[9], frame[10], frame[11]);
    int32_t ch3_uv = raw_to_microvolts_int(ch3_raw);
    int32_t ch3_notched_uv = biquad_process_int_uv(&notch60, ch3_uv);

    *ch3_raw_out = ch3_raw;
    *ch3_filtered_uv_out = ch3_notched_uv;

    return 0;
}

static int start_ads_streaming(void)
{
    int err;

    printk("Starting ADS1299 CH3 ECG streaming...\n");

    /*
     * Reset filter state each time streaming starts.
     */
    biquad_notch_init(&notch60,
                      (float)ADS_SAMPLE_RATE_HZ,
                      ADS_NOTCH_FREQ_HZ,
                      30.0f);

    err = configure_ads1299_ch3_ecg_500sps_bias();
    if (err) {
        printk("ADS1299 ECG config failed: %d\n", err);
        return err;
    }

    err = ads1299_start_conversions();
    if (err) {
        printk("ADS1299 start conversions failed: %d\n", err);
        return err;
    }

    /*
     * Discard first few frames after START/RDATAC.
     */
    for (int i = 0; i < 5; i++) {
        int32_t dummy_raw = 0;
        int32_t dummy_uv = 0;
        read_ch3_sample(&dummy_raw, &dummy_uv);
    }

    printk("ADS streaming started\n");
    return 0;
}

static void stop_ads_streaming(void)
{
    printk("Stopping ADS1299 streaming...\n");
    ads1299_stop_conversions();
    printk("ADS streaming stopped\n");
}

int main(void)
{
    int err;
    bool streaming = false;
    uint32_t sample_counter = 0;

    uint8_t id = 0;
    uint8_t config1 = 0;
    uint8_t config2 = 0;
    uint8_t config3 = 0;

    printk("Soft NeuroKit ADS1299 + BLE ECG START/STOP APP\n");

    /*
     * Initialize ADS1299 first.
     */
    err = ads1299_init();
    if (err) {
        printk("ads1299_init failed: %d\n", err);
        return 0;
    }

    /*
     * Confirm ADS1299 communication.
     *
     * Expected baseline for current working SPI mode:
     *   ID      = 0x3E
     *   CONFIG1 = 0x96
     *   CONFIG2 = 0xC0
     *   CONFIG3 = 0x60
     */
    ads1299_read_id(&id);
    ads1299_read_reg(ADS_REG_CONFIG1, &config1);
    ads1299_read_reg(ADS_REG_CONFIG2, &config2);
    ads1299_read_reg(ADS_REG_CONFIG3, &config3);

    printk("ADS baseline: ID=0x%02X CONFIG1=0x%02X CONFIG2=0x%02X CONFIG3=0x%02X\n",
           id, config1, config2, config3);

    /*
     * Initialize BLE.
     *
     * Mobile app enables notifications to start streaming.
     */
    err = ble_init();
    if (err) {
        printk("ble_init failed: %d\n", err);
        return 0;
    }

    printk("Ready. Enable BLE notifications to START ECG streaming.\n");
    printk("Disable BLE notifications or disconnect to STOP streaming.\n");

    while (1) {
        /*
         * START command from BLE notification enable.
         */
        if (ble_start_requested()) {
            ble_clear_start_request();
            ble_clear_stop_request();

            if (!streaming) {
                err = start_ads_streaming();
                if (!err) {
                    streaming = true;
                    sample_counter = 0;
                }
            }
        }

        /*
         * STOP command from BLE notification disable or disconnect.
         */
        if (ble_stop_requested()) {
            ble_clear_stop_request();

            if (streaming) {
                stop_ads_streaming();
                streaming = false;
            }
        }

        /*
         * If connected and notifications are enabled, stream CH3.
         */
        if (streaming && ble_is_connected() && ble_notifications_enabled()) {
            int32_t ch3_raw = 0;
            int32_t ch3_uv = 0;

            err = read_ch3_sample(&ch3_raw, &ch3_uv);
            if (err) {
                printk("CH3 read failed: %d\n", err);
                k_msleep(10);
                continue;
            }

            /*
             * Print ADS1299 CH3 signal to terminal.
             *
             * raw         = signed 24-bit ADS1299 ADC count
             * filtered_uV = microvolts after conversion + 60 Hz notch
             */
            if ((sample_counter % PRINT_EVERY_N_SAMPLES) == 0) {
                printk("ADS CH3 sample=%u raw=%d filtered_uV=%d\n",
                       sample_counter,
                       ch3_raw,
                       ch3_uv);
            }

            sample_counter++;

            /*
             * Current BLE payload format expects CH3 + CH4.
             *
             * We only use CH3 right now:
             *   CH3 = filtered microvolts
             *   CH4 = 0
             */
            err = ble_send_ch34(ch3_uv, 0);

            /*
             * BLE may return -EAGAIN/-ENOMEM if the controller is busy.
             * Do not stop streaming for transient notify failures.
             */
            if (err && err != -EAGAIN && err != -ENOMEM) {
                printk("BLE notify failed: %d\n", err);
            }
        } else {
            /*
             * Idle while waiting for BLE command.
             */
            k_msleep(20);
        }
    }

    return 0;
}