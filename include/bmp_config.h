#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#ifndef bmp_config
#define bmp_config

/* Initializes all state machines given a hand with 5 fingers */
void hand_init(StateMachine *sm);

/* Helper function. Initializes all useful parameters automatically */
void construct_sensor(struct BmpSensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH]);

/* Helper function. Initializes all useful parameters. Switch_num determines which switches will
   always be active */
void construct_i2c_switch(struct I2cSwitch (*i2c_switch)[MAX_SWITCH_PER_SM], uint switch_num);

/* Helper function. sm_num and switch_num determine which state machines and switches are active.
   sm_num is not a mask. */
void construct_sm(StateMachine *sm, uint sm_num, uint switch_num);

/* Helper function. Receives an address, 2 buffers and the data_len. Can be used in both read and write operations */
void message_constructor(Message *msg, uint8_t addr, uint32_t *txbuff, uint32_t (*rxbuff)[5], uint data_len);

/* Helper function. Higher order constructor */
void construct_hand(StateMachine *sm, uint sm_num, uint switch_num_sm,
                    struct I2cSwitch (*i2c_switch)[MAX_SWITCH_PER_SM], uint switch_num_sw,
                    struct BmpSensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH]);

/* Helper function. Links data structures between themselves */
void linker(StateMachine *sm, struct I2cSwitch (*i2c_switch)[MAX_SWITCH_PER_SM], 
            struct BmpSensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH], uint active_sm);

#endif