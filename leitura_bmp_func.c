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
    sm_config_set_jmp_pin(&c, scl);

    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_clkdiv(&c, 7.8125f);
    
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

    sm.config = c;

    pio_sm_init(sm.pio, sm.index, sm.offset, &c);
    pio_sm_set_enabled(sm.pio, sm.index, true);

    pio_sm_clear_fifos(sm.pio, sm.index);

}

void sm_soft_reboot(struct State_Machine *sm){
    pio_sm_set_enabled(sm->pio, sm->index, false);
    sm_init(*sm, SDA, SCL);
    
}

/* Initializes a BMP sensor on a state machine */
void bmp_init(struct bmp_sensor *bmp, struct State_Machine sm){

    uint8_t init_seq[2] = {0x1b, 0x33};
    uint success = 0;

    if(bmp->address == BMP_ADDRESS_PRIM){                            // Skips if channel is already open
        write_i2c_burst(sm, bmp->switch_addr, &(bmp->channel), 1);
    }

    success = bmp_status_refresh(sm, bmp, write_i2c_burst(sm, bmp->address, init_seq, sizeof(init_seq)));
    
    if (success == 0){
        bmp->active = true;
    }

    return;
}

void bmp_get_calib(struct State_Machine sm, struct bmp_sensor *bmp){
    uint8_t nvm_buffer[21];

    for (size_t i = 0; i < sizeof(nvm_buffer); i++){
        if (bmp->active){
            read_reg_i2c(sm, bmp->address, BMP_CALIB_REG_FIRST + i, &nvm_buffer[i], 1);
        }
    }

    bmp->par_t1 = (uint16_t) (nvm_buffer[0] | (nvm_buffer[1] << 8))     * (double) (1llu << 8);
    bmp->par_t2 = (uint16_t) (nvm_buffer[2] | (nvm_buffer[3] << 8))     / (double) (1llu << 30);
    bmp->par_t3 = (int8_t) nvm_buffer[4]                                / (double) (1llu << 48);

    bmp->par_p1 = ((int16_t) (nvm_buffer[5] | (nvm_buffer[6] << 8)) - (1 << 14))      / (double) (1llu << 20);
    bmp->par_p2 = ((int16_t) (nvm_buffer[7] | (nvm_buffer[8] << 8)) - (1 << 14))      / (double) (1llu << 29);
    bmp->par_p3 = (int8_t) nvm_buffer[9]                                / (double) (1llu << 32);
    bmp->par_p4 = (int8_t) nvm_buffer[10]                               / (double) (1llu << 37);
    bmp->par_p5 = (uint16_t) (nvm_buffer[11] | (nvm_buffer[12] << 8))   * (double) (1llu << 3);
    bmp->par_p6 = (uint16_t) (nvm_buffer[13] | (nvm_buffer[14] << 8))   / (double) (1llu << 6);
    bmp->par_p7 = (int8_t) nvm_buffer[15]                               / (double) (1llu << 8);
    bmp->par_p8 = (int8_t) nvm_buffer[16]                               / (double) (1llu << 15);
    bmp->par_p9 = (int16_t) (nvm_buffer[17] | (nvm_buffer[18] << 8))    / (double) (1llu << 48);
    bmp->par_p10 =(int8_t) nvm_buffer[19]                               / (double) (1llu << 48); 
    bmp->par_p11 =(int8_t) nvm_buffer[20]                               / ((double) (1llu << 63) * 2.0f * 2.0f); 
    
}

int bmp_write(struct State_Machine sm, struct bmp_sensor *bmp, uint8_t data[], uint8_t data_len){
    bmp_status_check(bmp);

    if (!(bmp->active)){
        return BMP_INACTIVE;
    }
        
    if(write_i2c_burst(sm, bmp->switch_addr, &(bmp->channel), 1) != 0){
        return CHANNEL_CLOSED;
    }

    return bmp_status_refresh(sm, bmp, write_i2c_burst(sm, bmp->address, data, data_len));
}

int bmp_status_refresh(struct State_Machine sm, struct bmp_sensor *bmp, uint error_mask){
    if(error_mask == 0){
        return OK;
    } else if (error_mask % 2 == 1){
        bmp->address_fail += 1;
        return NOT_OK;
    } else if (error_mask > 1){
        bmp->message_fail += 1;
        return NOT_OK;
    } else {
        sm_soft_reboot(&sm);
        return LINE_DOWN;
    }
    
}

int bmp_status_check(struct bmp_sensor *bmp){
    if (bmp->address_fail > 3 || bmp->message_fail > 3){
        bmp->active = false;
    }

    return bmp->active;
}

int line_check(uint sda, uint scl){
    if (gpio_is_pulled_down(sda) || gpio_is_pulled_down(scl)){
        return LINE_DOWN;
    }
    return OK;
}


/* Writes multiple messages without stopping in between. Data should be organized 
   in Resgister->Address pairs */
uint write_i2c_burst(struct State_Machine sm, uint8_t addr, uint8_t data[], uint8_t data_len){

    uint acks_mask = 0;                                         //Mask of errors.
    
    pio_sm_put(sm.pio, sm.index, data_len << 24);               //The message is pulled from the left in the TX FIFO
    pio_sm_put_blocking(sm.pio, sm.index, addr << 25);          //Add write "0" bit
    acks_mask = pio_sm_get_blocking(sm.pio, sm.index);          //Slave's acknowledge

    for (size_t i = 0; i < data_len; i++){
        pio_sm_put(sm.pio, sm.index, data[i] << 24);
        acks_mask |= (pio_sm_get_blocking(sm.pio, sm.index) << (i+1));
    }
    
    pio_sm_put(sm.pio, sm.index, FILL_TXF);                     //Signals the PIO to jump to stop
    return acks_mask;                     
}

/* Reads data_len messages from message from a register and stores them in rxbuff */
int read_reg_i2c(struct State_Machine sm, uint8_t addr, uint8_t reg, uint8_t rxbuff[], uint8_t data_len){
    
    uint64_t acks_mask = 0;

    acks_mask = write_i2c_burst(sm, addr, &reg, 1);
    
    pio_sm_put(sm.pio, sm.index, 0x00);                                 //Message with length 0
    pio_sm_put_blocking(sm.pio, sm.index, ((addr << 1) + 1) << 24);     //Add read "1" bit
    acks_mask |= pio_sm_get_blocking(sm.pio, sm.index) << 2;

    if (acks_mask > 0){
        pio_sm_put(sm.pio, sm.index, FILL_TXF);
        return acks_mask;
    }
    
    pio_sm_put(sm.pio, sm.index, 0x00);

    for (size_t i = 0; i < data_len-1; i++){                            //Signals the PIO to jump to reading sequence
        rxbuff[i] = pio_sm_get_blocking(sm.pio, sm.index);
        pio_sm_put(sm.pio, sm.index, 0x00);
    }

    rxbuff[data_len-1] = pio_sm_get_blocking(sm.pio, sm.index);
    pio_sm_put(sm.pio, sm.index, FILL_TXF);                             //Stop condition
    
    return line_check(SDA, SCL);
}

int bmp_read(struct State_Machine sm, struct bmp_sensor *bmp, uint8_t reg, uint8_t data[], uint8_t data_len){
    bmp_status_check(bmp);

    if (!(bmp->active)){
        return BMP_INACTIVE;
    }
    
    if(write_i2c_burst(sm, bmp->switch_addr, &(bmp->channel), 1) != 0){
        return CHANNEL_CLOSED;
    }

    return bmp_status_refresh(sm, bmp, read_reg_i2c(sm, bmp->address, reg, data, data_len));
}

/* Writes raw values to CSV file */
void bmp_calib_file_helper(struct State_Machine sm, struct bmp_sensor *bmp){
    if (!(bmp->active)){
        return;
    }

    gpio_pull_up(7);
    printf("Addr - 0x%02x, Channel - %u\n", bmp->address, bmp->channel);
    printf("%.3e\n", bmp->par_t1);
    printf("%.3e\n", bmp->par_t2);
    printf("%.3e\n", bmp->par_t3);
    printf("%.3e\n", bmp->par_p1);
    printf("%.3e\n", bmp->par_p2);
    printf("%.3e\n", bmp->par_p3);
    printf("%.3e\n", bmp->par_p4);
    printf("%.3e\n", bmp->par_p5);
    printf("%.3e\n", bmp->par_p6);
    printf("%.3e\n", bmp->par_p7);
    printf("%.3e\n", bmp->par_p8);
    printf("%.3e\n", bmp->par_p9);
    printf("%.3e\n", bmp->par_p10);
    printf("%.3e\n", bmp->par_p11);
    gpio_pull_down(7);
}

double bmp_compensate_temperature(uint32_t uncomp_temp, struct bmp_sensor *bmp){
    double partial_data1;
    double partial_data2;

    partial_data1 = (double) (uncomp_temp - bmp->par_t1);
    partial_data2 = (double) (partial_data1 * bmp->par_t2);

    bmp->comp_temp = partial_data2 + (partial_data1 * partial_data1) * bmp->par_t3;

    return bmp->comp_temp;
}

double bmp_compensate_pressure(uint32_t uncomp_press, struct bmp_sensor *bmp){
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;

    double partial_out1;
    double partial_out2;

    partial_data1 = bmp->par_p6 * bmp->comp_temp;
    partial_data2 = bmp->par_p7 * (bmp->comp_temp * bmp->comp_temp);
    partial_data3 = bmp->par_p8 * (bmp->comp_temp * bmp->comp_temp * bmp->comp_temp);
    partial_out1 = bmp->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = bmp->par_p2 * bmp->comp_temp;
    partial_data2 = bmp->par_p3 * (bmp->comp_temp * bmp->comp_temp);
    partial_data3 = bmp->par_p4 * (bmp->comp_temp * bmp->comp_temp * bmp->comp_temp);
    partial_out2 = (double) uncomp_press * (bmp->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (double) uncomp_press * (double) uncomp_press;
    partial_data2 = bmp->par_p9 + bmp->par_p10 * bmp->comp_temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((double) uncomp_press * (double) uncomp_press * (double) uncomp_press) * bmp->par_p11;

    bmp->comp_press[0] = partial_out1 + partial_out2 + partial_data4;

    return bmp->comp_press[0];
}

void bmp_get_pressure(struct State_Machine sm, struct bmp_sensor *bmp){

    bmp->comp_press[2] = bmp->comp_press[1];
    bmp->comp_press[1] = bmp->comp_press[0]; 

    uint8_t temp_buffer[3];
    uint8_t press_buffer[3];

    bmp_read(sm, bmp, BMP_TEMP_REG_LSB, temp_buffer, 3);
    bmp_read(sm, bmp, BMP_PRESS_REG_LSB, press_buffer, 3);

    uint32_t temp_raw = (temp_buffer[2] << 16) | (temp_buffer[1] << 8) | temp_buffer[0];
    uint32_t press_raw = (press_buffer[2] << 16) | (press_buffer[1] << 8) | press_buffer[0];

    bmp_compensate_temperature(temp_raw, bmp);
    bmp_compensate_pressure(press_raw, bmp);

}

void bmp_press_file_helper(struct State_Machine sm, struct bmp_sensor *bmp){
    if (!(bmp->active)){
        return;
    }
    
    gpio_pull_up(7);
    printf("Addr - 0x%02x, Channel - %u\n", bmp->address, bmp->channel);
    printf("Pressure -> %.2lf, Temperature -> %.2f\n", bmp->comp_press[0], bmp->comp_temp);
    gpio_pull_down(7);
}