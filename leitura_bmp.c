#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/multicore.h"

#include "leitura_bmp.h"
#include "leitura_bmp_func.c"
#include "config.c"

#include "leitura_bmp.pio.h"

void core_entry(){

    while (true){
        while (!multicore_fifo_rvalid()){
            tight_loop_contents();
        }

        struct bmp_sensor *bmp[MAX_SM];
        uint32_t n = multicore_fifo_pop_blocking();

        while (!multicore_fifo_rvalid()){
            tight_loop_contents();
        }

        func_ptr_t func = (func_ptr_t) multicore_fifo_pop_blocking();

        if (n > MAX_SM){
            n = MAX_SM;
        }
        
        for (size_t i = 0; i < n; i++){
            while (!multicore_fifo_rvalid()){
                tight_loop_contents();
            }

            bmp[i] = (struct bmp_sensor*) multicore_fifo_pop_blocking();
        }
        
        for (size_t i = 0; i < n; i++){
            func(bmp[i]);
        }

    }
    
}

int main(){
    stdio_init_all();
    uart_begin();

    multicore_reset_core1();
    multicore_launch_core1(core_entry);

    static struct bmp_sensor bmp_all[MAX_SM][MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH];
    static struct pca pca_all[MAX_SM][MAX_SWITCH_PER_SM];
    static struct State_Machine sm_all[MAX_SM];

    uint active_bmp = 12;
    uint active_switch_1 = 0x801; //Mask
    uint active_switch_2 = 0x21; //Mask
    uint active_sm = 2;

    gpio_put(UART_ENABLE, true);
    sleep_ms(3000);

    construct_sensor(bmp_all, active_bmp);
    construct_i2c_switch(pca_all, active_switch_1);
    construct_sm(sm_all, active_sm, active_switch_2);

    for (size_t i = 0; i < MAX_SM; i++){
        sm_all[i].active_sm = active_sm;    
        sm_all[i].pca_ptr = pca_all[i];
        
        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            sm_all[i].pca_ptr[j].bmp = bmp_all[i][j]; 
        }
    }

    sm_init(&sm_all[0], SDA, SCL);
    sm_init(&sm_all[1], SDA1, SCL1);

    bmp_init(sm_all);
    bmp_get_calib(sm_all);

    sleep_ms(3000);
    absolute_time_t now = get_absolute_time();
    while ((to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(now)) < 10*SEC_IN_MS){
        bmp_get_pressure(sm_all);
        bmp_press_file_helper(sm_all);
    }

    // gpio_put(UART_ENABLE, true);
    // while ((to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(now)) < 10*MIN_TO_SEC*SEC_IN_MS){
    //     for (size_t i = 0; i < 8; i++){
    //         bmp_get_pressure(&sm0, &bmp_top[i]);
    //     }
    //     for (size_t i = 0; i < 8; i++){
    //         bmp_top[i].pca_ptr->open = false;
    //     }

    //     for (size_t i = 0; i < 8; i++){
    //         bmp_press_file_helper(&bmp_top[i]);
    //     }
    //     uart_puts(UART_ID, "\n");
    // }

    // for (size_t i = 0; i < 8; i++){
    //     char buffer[16];

    //     if (bmp_top[i].active){
    //         snprintf(buffer, sizeof(buffer), "%f,", bmp_top[i].error_rate);
    //         uart_puts(UART_ID, buffer);
    //     }
    // }
    // uart_puts(UART_ID, "\n");
    // sleep_ms(10);
    
    // uint8_t pca_end[1] = {PCA_CLOSE};
    // write_i2c(&sm0, PCA_ADDRESS_TOP, pca_end, sizeof(pca_end));
    gpio_put(UART_ENABLE, false);
    
    while (true){
        tight_loop_contents();
    }

}


