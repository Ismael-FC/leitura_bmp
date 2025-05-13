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

    uint8_t pca_addr = PCA_ADDRESS_BOTTOM;
    uint8_t bmp_addr = BMP_ADDRESS_PRIM;
    
    sm_init(sm0, SDA, SCL);

    uint8_t pca_init[1] = {0x01};
    uint8_t dummy = 0x06;

    uint8_t bmp_init[] = {0x1b, 0x31};

    // //0x7e, 0xb6, 0x1c, 0x01, 0x1d, 0x00, 0x1f, 0x00, 0x19, 0x41, 0x17, 0x00, 0x7e, 0xb0, 

    uint8_t close = PCA_CLOSE;

    write_reg_i2c(sm0, pca_addr, pca_init, sizeof(pca_init));
    write_reg_i2c(sm0, bmp_addr, bmp_init, sizeof(bmp_init));

    sleep_ms(3000);
    printf("Pressure - %u\n", read_reg_i2c(sm0, bmp_addr, BMP_STATUS_REG));
    printf("Status - %u\n", read_reg_i2c(sm0, bmp_addr, BMP_PRESS_REG_LSB));
    sleep_ms(5);
    
    printf("Status - %u\n", read_reg_i2c(sm0, bmp_addr, BMP_STATUS_REG));
    
    write_reg_i2c(sm0, pca_addr, &close, 1);

    while (true){
        tight_loop_contents();
    }

}


