#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "leitura_bmp.h"
#include "leitura_bmp.pio.h"

void construct_sensor(struct bmp_sensor (*bmp)[MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH], uint sensor_num){
    
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

void construct_i2c_switch(struct pca (*i2c_switch)[MAX_SWITCH_PER_SM], uint switch_num){

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
            i2c_switch[i][j].open_channel = CHANNEL_CLOSED;

        }
    }
    
}

void construct_sm(struct State_Machine *sm, uint sm_num, uint switch_num){

    /* MARK */
    int offset = pio_add_program(pio0, &leitura_bmp_program);

    for (size_t i = 0; i < MAX_SM; i++){
        sm[i].active_switch = 0;
    }

    for (size_t i = 0; i < MAX_SM; i++){
        sm[i].index = i;
        sm[i].pio = (i < 4) ? pio0 : pio1;
        sm[i].offset = offset;
        sm[i].status = (i < sm_num) ? true : false;

        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            sm[i].active_switch |= (switch_num >> i*3) & (1 << j);
        }
    }
}