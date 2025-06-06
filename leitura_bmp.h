#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#ifndef leitura_bmp
#define leitura_bmp

struct State_Machine{
   
   /* 0 to 7 on a RP2040, 0 to 12 on a RP2350 */
   uint index;

   /* pio0 or pio1 */
   PIO pio;

   /* Where the program starts. Offsets can 
   be accessed in .../build/filename.pio.h */
   uint offset;

   /* SM configuration variable */
   pio_sm_config config;
};

/* I2C switch name */
struct pca{
   /* 3 possible addresses */
   uint8_t address;

   uint8_t channel;
   bool open;
};


struct bmp_sensor{
   /* To which I2C switch is linked to */
   struct pca *pca_ptr;

   /* 1 of 2 possible addresses */
   uint8_t address;
   
   /* Doing active measurements */
   bool active;
   
   /* Calibration data (0->2, temperature, 3->13, pressure) */
   float calib_val[14];

   /* Uncompensated values */
   uint32_t press_raw;
   uint32_t temp_raw;

   /* Compensated values */
   float comp_temp[3];
   float comp_press[3];

   /* Rate of errors in an interval bigger than a second */ 
   int error_num;
   float error_rate;

   /* Timestamps, in ms */
   uint32_t last_error;
   uint32_t current_error;

   /* Increments if error rate is bigger than ERROR_THRESHOLD */
   int flag;
   
};

/* Initiates a single state machine and stores its configuration */
void sm_init(struct State_Machine *sm, uint sda, uint scl);

/* Reboots a state machine if it was stuck */
void sm_soft_reboot(struct State_Machine *sm);

/* Opens a channel if it was closed */
int channel_open(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Initializes a single BMP sensor. Failure to initialize will result
   in an inactive sensor. Inactive sensors cannot perform read/write operations */
void bmp_init(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Reads, converts and stores the unique calibration values of a BMP*/
void bmp_get_calib(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Refreshes the fail status of a BMP sensor based on an error mask.
   It does not take into account line failures */
int bmp_status_refresh(struct State_Machine *sm, struct bmp_sensor *bmp, int64_t error_mask);

/* Deactivates BMP sensors whose failure status exceeds a certain threshold */
int bmp_status_check(struct bmp_sensor *bmp);

/* Sends data via I2C to a sensor and automatically refreshes its status
   in case of failure */
int bmp_write(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t data[], uint8_t data_len);

/* Receives data via I2C from a BMP and automatically refreshes its status
   in case of failure. Can also reboot the SM if there's a line failure */
int bmp_read(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t reg, uint8_t data[], uint8_t data_len);

/* Turns raw temperature values into readable values in ºC */
float bmp_compensate_temperature(struct bmp_sensor *bmp);

/* Turns raw pressure values into readable values in Pa */
float bmp_compensate_pressure(struct bmp_sensor *bmp);

/* Reads raw pressure and temperature (for compensation) values from a BMP sensor */
void bmp_get_pressure(struct State_Machine *sm, struct bmp_sensor *bmp);

/* Writes calibration values into a CSV file via UART */
void bmp_calib_file_helper(struct bmp_sensor *bmp);

/* Writes pressure and temperature values into a CSV file via UART */
void bmp_press_file_helper(struct bmp_sensor *bmp);

/* Sends data via I2C. Assumes data_len is, at least, equal to data */
int64_t write_i2c(struct State_Machine *sm, uint8_t addr, uint8_t data[], uint8_t data_len);

/* Reads data_len from reg via I2C and writes it into rxbuff.*/
int64_t read_i2c(struct State_Machine *sm, uint8_t addr, uint8_t reg, uint8_t rxbuff[], uint8_t data_len);

/* Standard SDA and SCL lines*/
#define SDA 26
#define SCL 27
#define VCC 28

/* I2C switches possible addresses */
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

/* PIO Stop signal */
#define FILL_TXF            0xffffffff

/* Communication status */
#define OK                  0
#define NOT_OK              1

/* Errors causes */
#define BMP_INACTIVE        -1
#define CHANNEL_CLOSED      -2
#define ADDR_NOT_FOUND      1
#define LINE_DOWN           -4

/* Error status */
#define ERROR_THRESHOLD     0.3
#define GRACE_PERIOD         5000
#define SENSOR_OK            0
#define SENSOR_FLAGGED      -1
#define SENSOR_WARNED       -2

/* To convert most timestamps */
#define SEC_IN_MS            1000
#define MIN_TO_SEC           60

/* Pressure/temperature thresholds in the datasheet */
#define MAX_BMP_PRESS        125000
#define MIN_BMP_PRESS        30000 
#define MAX_BMP_TEMP         85
#define MIN_BMP_TEMP         -4

/* Arbitrary maximum variation permited, per 5 ms */
#define MAX_BMP_VAR_PRESS    5000

/* UART Utility */
#define UART_ID uart1
#define UART_TX 4
#define UART_RX 5
#define UART_ENABLE 7
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define BAUD_RATE 500000

#endif