#ifndef ADS1299_H
#define ADS1299_H

#include <stdint.h>
#include <stddef.h>

/* ADS1299 commands */
#define ADS_CMD_WAKEUP   0x02
#define ADS_CMD_STANDBY  0x04
#define ADS_CMD_RESET    0x06
#define ADS_CMD_START    0x08
#define ADS_CMD_STOP     0x0A
#define ADS_CMD_RDATAC   0x10
#define ADS_CMD_SDATAC   0x11
#define ADS_CMD_RDATA    0x12
#define ADS_CMD_RREG     0x20
#define ADS_CMD_WREG     0x40

/* ADS1299 registers */
#define ADS_REG_ID       0x00
#define ADS_REG_CONFIG1  0x01
#define ADS_REG_CONFIG2  0x02
#define ADS_REG_CONFIG3  0x03
#define ADS_REG_LOFF     0x04

// EEG channels
#define ADS_REG_CH1SET   0x05
#define ADS_REG_CH2SET   0x06
#define ADS_REG_CH3SET   0x07
#define ADS_REG_CH4SET   0x08
#define ADS_REG_CH5SET   0x09
#define ADS_REG_CH6SET   0x0A
#define ADS_REG_CH7SET   0x0B
#define ADS_REG_CH8SET   0x0C


//bias
#define ADS_REG_BIAS_SENSP  0x0D
#define ADS_REG_BIAS_SENSN  0x0E
#define ADS_REG_LOFF_SENSP  0x0F
#define ADS_REG_LOFF_SENSN  0x10
#define ADS_REG_LOFF_FLIP   0x11
#define ADS_REG_MISC1       0x15

int ads1299_init(void);

int ads1299_send_command(uint8_t cmd);
int ads1299_read_reg(uint8_t reg, uint8_t *value);
int ads1299_write_reg(uint8_t reg, uint8_t value);
int ads1299_read_id(uint8_t *id);

int ads1299_get_drdy_raw(void);

int ads1299_start_conversions(void);
int ads1299_stop_conversions(void);

int ads1299_read_data_frame(uint8_t *data, size_t len);

#endif