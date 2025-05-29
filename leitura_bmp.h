#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#ifndef leitura_bmp
#define leitura_bmp

struct State_Machine{
    uint index;
    PIO pio;
    uint offset;

    pio_sm_config config;
};

struct pca{
   uint8_t address;
   uint8_t channel;
   bool open;
};


struct bmp_sensor{
   uint8_t switch_addr;
   struct pca *pca_ptr;

   uint8_t address;
   uint8_t channel;

   bool active;
   uint address_fail;
   uint message_fail;
   
   uint32_t press_raw;
   uint32_t temp_raw;

   /*Calibration data*/
   double par_t1;
   double par_t2;
   double par_t3;
   
   double par_p1;
   double par_p2;
   double par_p3;
   double par_p4;
   double par_p5;
   double par_p6;
   double par_p7;
   double par_p8;
   double par_p9;
   double par_p10;
   double par_p11;
   /*End of calibration data*/

   double comp_temp;
   double comp_press[3];
};

#define SDA 26
#define SCL 27

/* Initiates a single state machine and stores its configuration */
void sm_init(struct State_Machine *sm, uint sda, uint scl);

/* Reboots a state machine if it was stuck */
void sm_soft_reboot(struct State_Machine *sm);

/* Opens a channel if it wasn't openned */
int channel_open(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Initializes a single BMP sensor. Failure to initialize will result
   in an inactive sensor. Inactive sensors cannot perform read/write operations */
void bmp_init(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Reads, converts and stores the unique calibration values of a BMP*/
void bmp_get_calib(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Refreshes the fail status of a BMP sensor based on an error mask.
   It does not take into account line failures */
int bmp_status_refresh(struct State_Machine *sm, struct bmp_sensor *bmp, uint error_mask);

/* Deactivates BMP sensors whose failure status exceeds a certain threshold */
int bmp_status_check(struct bmp_sensor *bmp);

/* Sends data via I2C to a sensor and automatically refreshes its status
   in case of failure */
int bmp_write(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t data[], uint8_t data_len);

/* Receives data via I2C from a BMP and automatically refreshes its status
   in case of failure. Can also reboot the SM if there's a line failure */
int bmp_read(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t reg, uint8_t data[], uint8_t data_len);

/* Turns raw temperature values into readable values in ºC */
double bmp_compensate_temperature(struct bmp_sensor *bmp);

/* Turns raw pressure values into readable values in Pa */
double bmp_compensate_pressure(struct bmp_sensor *bmp);

/* Reads raw pressure and temperature (for compensation) values from a BMP sensor */
void bmp_get_pressure(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Writes calibration values into a CSV file via UART */
void bmp_calib_file_helper(struct bmp_sensor *bmp);

/* Writes pressure and temperature values into a CSV file via UART */
void bmp_press_file_helper(struct bmp_sensor *bmp);

/* Sends data via I2C. Assumes data_len is, at least, equal to data */
int write_i2c(struct State_Machine *sm, uint8_t addr, uint8_t data[], uint8_t data_len);

/* Reads data_len from reg via I2C and writes it into rxbuff.*/
int read_i2c(struct State_Machine *sm, uint8_t addr, uint8_t reg, uint8_t rxbuff[], uint8_t data_len);

/* Phalanges addresses */
#define PCA_ADDRESS_TOP     0x72 
#define PCA_ADDRESS_MID     0x71
#define PCA_ADDRESS_BOTTOM  0x70

/* PCA Channels */
#define CHANNEL_1 0x01
#define CHANNEL_2 0x02
#define CHANNEL_3 0x04
#define CHANNEL_4 0x08

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

/* First and last calibration registers */
#define BMP_CALIB_REG_FIRST 0x31
#define BMP_CALIB_REG_LAST  0x45

/* Command's register */
#define BMP_CMD             0x7e

#define FILL_TXF            0xffffffff

/* Status */
#define NOT_OK              1
#define OK                  0

/* Errors */
#define BMP_INACTIVE        -1
#define CHANNEL_CLOSED      -2
#define ADDR_NOT_FOUND      1
#define LINE_DOWN           -4

#endif