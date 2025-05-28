#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "leitura_bmp.h"
#include "leitura_bmp_func.c"

#include "leitura_bmp.pio.h"



int main(){
    stdio_init_all();

    struct State_Machine sm0 = {
        0, 
        pio0, 
        pio_add_program(pio0, &leitura_bmp_program)
    };

    struct bmp_sensor bmp_top[8] = {
        {PCA_ADDRESS_TOP, BMP_ADDRESS_PRIM, CHANNEL_1, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_SEC , CHANNEL_1, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_PRIM, CHANNEL_2, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_SEC , CHANNEL_2, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_PRIM, CHANNEL_3, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_SEC , CHANNEL_3, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_PRIM, CHANNEL_4, false, 0, 0},
        {PCA_ADDRESS_TOP, BMP_ADDRESS_SEC , CHANNEL_4, false, 0, 0}
    };

    sm_init(sm0, SDA, SCL);

    
    for (size_t i = 0; i < 8; i++){
        bmp_init(&bmp_top[i], sm0);
        bmp_get_calib(sm0, &bmp_top[i]);
    }

    sleep_ms(5);
    for (size_t i = 0; i < 8; i++){
         bmp_calib_file_helper(sm0, &bmp_top[i]);
    }

    sleep_ms(5);
    for (size_t i = 0; i < 8; i++){
         bmp_get_pressure(sm0, &bmp_top[i]);
    }
    
    for (size_t i = 0; i < 8; i++){
         bmp_press_file_helper(sm0, &bmp_top[i]);
    }

    uint8_t pca_end[1] = {PCA_CLOSE};
    write_i2c_burst(sm0, PCA_ADDRESS_TOP, pca_end, sizeof(pca_end));
    
    while (true){
        tight_loop_contents();
    }

}


