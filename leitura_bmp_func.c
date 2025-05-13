#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "leitura_bmp.h"

#include "leitura_bmp.pio.h"

/* Initiates a single state machine */
void sm_init(struct State_Machine sm, uint sda, uint scl){

    pio_sm_config c = leitura_bmp_program_get_default_config(sm.offset);

    pio_sm_set_consecutive_pindirs(sm.pio, sm.index, sda, 2, true);

    sm_config_set_in_pins(&c, sda);
    sm_config_set_out_pins(&c, sda, 1);
    sm_config_set_set_pins(&c, sda, 1);
    sm_config_set_sideset_pins(&c, scl);

    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_clkdiv(&c, 45.0f);
    

    gpio_pull_up(scl);
    gpio_pull_up(sda);
    uint32_t both_pins = (1u << sda) | (1u << scl);
    pio_sm_set_pins_with_mask(sm.pio, sm.index, both_pins, both_pins);
    pio_sm_set_pindirs_with_mask(sm.pio, sm.index, both_pins, both_pins);

    pio_gpio_init(sm.pio, sda);
    gpio_set_oeover(sda, GPIO_OVERRIDE_INVERT);
    pio_gpio_init(sm.pio, scl);
    gpio_set_oeover(scl, GPIO_OVERRIDE_INVERT);
    
    pio_sm_set_pins_with_mask(sm.pio, sm.index, 0, both_pins);

    pio_sm_init(sm.pio, sm.index, sm.offset, &c);
    pio_sm_set_enabled(sm.pio, sm.index, true);

    pio_sm_clear_fifos(sm.pio, sm.index);

    

}

/* Writes a single message to the specified register in single mode. 
   Stops after every message. Assumes data is organized in Register-Message pairs */
void write_reg_i2c(struct State_Machine sm, uint8_t addr, uint8_t data[], uint data_len){

    if (data_len < 2u){
        write_i2c_burst(sm, addr, data, data_len);
    } 
    else {
        for (size_t i = 0; i < data_len; i += 2){                   //Data array must be even
            uint8_t buffer[2] = {data[i], data[i+1]};               
            write_i2c_burst(sm, addr, buffer, sizeof(buffer));
        }
    }
    
}

/* Writes multiple messages without stopping. Data should be organized 
   in Resgister->Address pairs */
void write_i2c_burst(struct State_Machine sm, uint8_t addr, uint8_t data[], uint8_t data_len){

    uint32_t acks[32] = {0};
    pio_sm_put_blocking(sm.pio, sm.index, data_len << 24);      //The message is pulled from the left in the TX FIFO
    pio_sm_put_blocking(sm.pio, sm.index, addr << 25);          //Add write "0" bit
    pio_sm_get_blocking(sm.pio, sm.index);                      //Slave's acknowledge

    for (size_t i = 0; i < data_len; i++){
        pio_sm_put_blocking(sm.pio, sm.index, data[i] << 24);
        acks[i] = pio_sm_get_blocking(sm.pio, sm.index);
    }
    
    pio_sm_put_blocking(sm.pio, sm.index, 0x00);                //Signals the PIO to jump to stop

}

/* Reads a single message from a register */
uint32_t read_reg_i2c(struct State_Machine sm, uint8_t addr, uint8_t reg){

    uint32_t message = 0;

    write_reg_i2c(sm, addr, &reg, 1);                           

    pio_sm_put_blocking(sm.pio, sm.index, 0x00);                        //Message with length 0
    pio_sm_put_blocking(sm.pio, sm.index, ((addr << 1) + 1) << 24);     //Add read "1" bit
    pio_sm_get_blocking(sm.pio, sm.index);

    pio_sm_put_blocking(sm.pio, sm.index, 0xffffffff);              //Signals the PIO to jump to reading sequence
    message = pio_sm_get_blocking(sm.pio, sm.index);

    pio_sm_put_blocking(sm.pio, sm.index, 0x00);              //Signals the PIO to jump to reading sequence
    message = pio_sm_get_blocking(sm.pio, sm.index);

    pio_sm_put_blocking(sm.pio, sm.index, 0x00);              //Signals the PIO to jump to reading sequence
    message = pio_sm_get_blocking(sm.pio, sm.index);

    pio_sm_put_blocking(sm.pio, sm.index, 0xffffffff);                    //Stop condition
    
    return message;
}