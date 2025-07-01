#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "bmp_cycle.h"

void print_basic_info(StateMachine *sm){
    for (size_t i = 0; i < MAX_SM; i++){
        char buffer_out[32];
        snprintf(buffer_out, sizeof(buffer_out), "SM Index - %d\n", sm[i].index);
        uart_puts(UART_ID, buffer_out);

        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            char buffer_out[64];
            snprintf(buffer_out, sizeof(buffer_out), "PCA Address - 0x%x, Activity - %s\n", 
                    sm[i].pca_ptr[j].address, sm[i].pca_ptr[j].status ? "true" : "false");
            uart_puts(UART_ID, buffer_out);

            for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
                char buffer_out[64];
                snprintf(buffer_out, sizeof(buffer_out), "BMP Address - 0x%x, BMP Channel - %d, BMP Status - %s\n", 
                     sm[i].pca_ptr[j].bmp[k].address, sm[i].pca_ptr[j].bmp[k].channel, sm[i].pca_ptr[j].bmp[k].active ? "True" : "False");
                uart_puts(UART_ID, buffer_out);
            }
        }
    }
}

void print_sm_info(StateMachine *sm){
    for (size_t i = 0; i < MAX_SM; i++){
        char buffer_out[64];
        snprintf(buffer_out, sizeof(buffer_out), "SM Index - %d\nSM Offset - %d\nSM Bus - %u\nSM Clock - %u\n", 
        sm[i].index, sm[i].offset, sm[i].sda, sm[i].scl);
        uart_puts(UART_ID, buffer_out);
    }
}

void test_pio(){
    for (size_t i = 0; i < 32; i++){
        pio_sm_put(pio0, 0, 0x01 << 24);
        pio_sm_put(pio0, 1, 0x01 << 24);

        pio_sm_put_blocking(pio0, 0, PCA_ADDRESS_BOTTOM << 25);
        pio_sm_put_blocking(pio0, 1, PCA_ADDRESS_BOTTOM << 25);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, CHANNEL_3 << 24);
        pio_sm_put(pio0, 1, CHANNEL_3 << 24);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, FILL_TXF);
        pio_sm_put(pio0, 1, FILL_TXF);

        pio_sm_put(pio0, 0, 0x02 << 24);
        pio_sm_put(pio0, 1, 0x02 << 24);

        pio_sm_put(pio0, 0, BMP_ADDRESS_PRIM << 25);
        pio_sm_put(pio0, 1, BMP_ADDRESS_PRIM << 25);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put_blocking(pio0, 0, BMP_PWR_CTRL << 24);
        pio_sm_put_blocking(pio0, 1, BMP_PWR_CTRL << 24);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, 0x33 << 24);
        pio_sm_put(pio0, 1, 0x33 << 24);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, FILL_TXF);
        pio_sm_put(pio0, 1, FILL_TXF);

        sleep_us(10);
    }
}



