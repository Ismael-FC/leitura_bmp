#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "bmp_cycle.h"
#include "bmp_config.h"
#include "bmp_cycle.pio.h"

void hand_init(StateMachine *sm){
    sm_init(&sm[0], SDA, SCL);
    sm_init(&sm[1], SDA1, SCL1);
    sm_init(&sm[2], SDA2, SCL2);
    sm_init(&sm[3], SDA3, SCL3);
    sm_init(&sm[4], SDA4, SCL4);
}

void construct_sensor(struct BmpSensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH]){
    
    for (size_t i = 0; i < MAX_SM; i++){
        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
                bmp[i][j][k].address = (k%2 == 0) ? BMP_ADDRESS_PRIM : BMP_ADDRESS_SEC;
                bmp[i][j][k].channel = CHANNEL_1 << k/2;
                bmp[i][j][k].active = false;
            }
        }
    }


}

void construct_i2c_switch(struct I2cSwitch (*i2c_switch)[MAX_SWITCH_PER_SM], uint switch_num){

    int counter = 0;

    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        for (size_t j = 0; j < MAX_SM; j++){
            i2c_switch[j][i].status = switch_num & (1 << counter);
            counter++;
        }
    }

    for (size_t i = 0; i < MAX_SM; i++){ 
        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            i2c_switch[i][j].address = PCA_ADDRESS_BOTTOM + j;
            i2c_switch[i][j].open_channel = 0x00;
            i2c_switch[i][j].magnetic_enc = j < 2 ? true : false;
        }
    }
    
}

void construct_sm(StateMachine *sm, uint sm_num, uint switch_num){

    int offset = pio_add_program(pio0, &bmp_cycle_program);

    for (size_t i = 0; i < MAX_SM; i++){
        sm[i].activeSwitch = 0;
    }

    for (size_t i = 0; i < MAX_SM; i++){
        sm[i].index = i;
        sm[i].pio = (i < 4) ? pio0 : pio1;
        sm[i].offset = offset;
        sm[i].status = (i < sm_num) ? true : false;

        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            sm[i].activeSwitch |= (switch_num >> i*3) & (1 << j);
        }
    }
}

void message_constructor(Message *msg, uint8_t addr, uint32_t *txbuff, uint32_t (*rxbuff)[5], uint data_len){
    msg->addr = addr;
    msg->txbuff = txbuff;
    msg->rxbuff = rxbuff;
    msg->data_len = data_len;
}


void construct_hand(StateMachine *sm, uint sm_num, uint switch_num_sm,
                    struct I2cSwitch (*i2c_switch)[MAX_SWITCH_PER_SM], uint switch_num_sw,
                    struct BmpSensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH]){

    construct_sensor(bmp);
    construct_i2c_switch(i2c_switch, switch_num_sw);
    construct_sm(sm, sm_num, switch_num_sm);                    
}


void linker(StateMachine *sm, struct I2cSwitch (*i2c_switch)[MAX_SWITCH_PER_SM], 
            struct BmpSensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH], uint active_sm){
    
    for (size_t i = 0; i < MAX_SM; i++){
        sm[i].activeSm = active_sm;    
        sm[i].p_i2cSwitch = i2c_switch[i];
        
        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            sm[i].p_i2cSwitch[j].p_bmp = bmp[i][j]; 
        }
    }
}