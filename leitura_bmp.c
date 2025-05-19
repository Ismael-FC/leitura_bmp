#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "leitura_bmp.h"
#include "leitura_bmp_func.c"

#include "leitura_bmp.pio.h"

#define SDA 26
#define SCL 27

int main(){
    stdio_init_all();

    struct State_Machine sm0 = {
        0, 
        pio0, 
        pio_add_program(pio0, &leitura_bmp_program)
    };
    
    sm_init(sm0, SDA, SCL);

    uint8_t pca_init_one[1] = {0x01};
    uint8_t pca_init_two[1] = {0x02};
    uint8_t pca_init_three[1] = {0x04};
    uint8_t pca_init_four[1] = {0x08};

    uint8_t pca_channels[4] = {0x01, 0x02, 0x04, 0x08};
    uint8_t pca_end[1] = {PCA_CLOSE};
    uint8_t bmp_init[2] = {0x1b, 0x33};

    for (size_t i = 0; i < 4; i++){
        write_reg_i2c(sm0, PCA_ADDRESS_TOP, &pca_channels[i], sizeof(pca_init_one));
        write_reg_i2c(sm0, BMP_ADDRESS_PRIM, bmp_init, sizeof(bmp_init));
        write_reg_i2c(sm0, BMP_ADDRESS_SEC, bmp_init, sizeof(bmp_init));
    }

    read_reg_i2c(sm0, BMP_ADDRESS_SEC, BMP_PRESS_REG_LSB, 3);

    write_reg_i2c(sm0, PCA_ADDRESS_TOP, pca_end, sizeof(pca_end));
    
    while (true){
        tight_loop_contents();
    }

}


