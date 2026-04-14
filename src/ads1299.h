#ifndef ADS1299_H
#define ADS1299_H

#include <stdint.h>
#include <stdbool.h>

int ads1299_init(void);

bool ads1299_read_ch34(int32_t *ch3, int32_t *ch4);

#endif