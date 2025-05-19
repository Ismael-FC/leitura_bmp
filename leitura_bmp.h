#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#ifndef leitura_bmp
#define leitura_bmp

struct State_Machine{
    uint index;
    PIO pio;
    uint offset;
};

void sm_init(struct State_Machine sm, uint sda, uint scl);
void write_reg_i2c(struct State_Machine sm, uint8_t addr, uint8_t data[], uint data_len);
void write_i2c_burst(struct State_Machine sm, uint8_t addr, uint8_t data[], uint8_t data_len);
uint32_t read_reg_i2c(struct State_Machine sm, uint8_t addr, uint8_t reg, uint8_t data_len);

/* Phalanges addresses */
#define PCA_ADDRESS_TOP     0x72 
#define PCA_ADDRESS_MID     0x71
#define PCA_ADDRESS_BOTTOM  0x70

/* Command to close all channels */
#define PCA_CLOSE           0x00

/* Two possible BMP addresses */
#define BMP_ADDRESS_PRIM    0x76
#define BMP_ADDRESS_SEC     0x77

/* Chip ID, error flags and status
   of the pressure and temperature sensors */
#define BMP_CHIP_ID_REG     0x00
#define BMP_ERR_REG         0x02
#define BMP_STATUS_REG      0x03

/* First of each of the 3 data 
   registers for each quantity */
#define BMP_PRESS_REG_LSB   0x04
#define BMP_TEMP_REG_LSB    0x07
#define BMP_SENSOR_TIME     0x0c

/* Triggers on a soft-reset */
#define BMP_EVENT_FLAG      0x10

/* Fifo configurarions */
#define BMP_FIFO_LENGTH_0   0x12
#define BMP_FIFO_LENGTH_1   0x13
#define BMP_FIFO_DATA       0x14
#define BMP_FIFO_WTM_0      0x15
#define BMP_FIFO_WTM_1      0x16
#define BMP_FIFO_CONFIG_1   0x17
#define BMP_FIFO_CONFIG_2   0x18

/* Interrupt flags */
#define BMP_INT_STATUS      0x11
#define BMP_INT_CTRL        0x19

/* SPI and I2C configurations */
#define BMP_IF_CONFIG       0x1a

/* Sensor options */
#define BMP_PWR_CTRL        0x1b
#define BMP_OSR             0x1c
#define BMP_ODR             0x1d
#define BMP_FILTER_CONFIG   0x1F

/* Command's register */
#define BMP_CMD             0x7e

#endif