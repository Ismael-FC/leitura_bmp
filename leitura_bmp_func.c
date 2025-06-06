#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "leitura_bmp.h"
#include "leitura_bmp.pio.h"

void uart_begin(){

    uart_init(UART_ID, 115200);
    gpio_set_function(UART_TX, UART_FUNCSEL_NUM(UART_ID, UART_TX));
    gpio_set_function(UART_RX, UART_FUNCSEL_NUM(UART_ID, UART_RX));

    uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);

    // Electrical architecture RS485
    gpio_init(UART_ENABLE);
    gpio_set_dir(UART_ENABLE, true);
    gpio_pull_up(UART_ENABLE);
}

void sm_init(struct State_Machine *sm, uint sda, uint scl){

    sm->config = leitura_bmp_program_get_default_config(sm->offset);

    pio_sm_set_consecutive_pindirs(sm->pio, sm->index, sda, 2, true);

    // State machine configuration
    sm_config_set_in_pins(&(sm->config), sda);
    sm_config_set_out_pins(&(sm->config), sda, 1);
    sm_config_set_set_pins(&(sm->config), sda, 1);
    sm_config_set_sideset_pins(&(sm->config), scl);
    sm_config_set_jmp_pin(&(sm->config), scl);

    sm_config_set_in_shift(&(sm->config), false, true, 8);
    sm_config_set_out_shift(&(sm->config), false, true, 8);
    
    //7.8125 = 1 MHz, 19.53125 = 400 kHz
    sm_config_set_clkdiv(&(sm->config), 7.8125f);               

    // Initialize power
    gpio_init(VCC);
    gpio_set_dir(VCC, true);
    gpio_put(VCC, true);
    
    gpio_pull_up(scl);
    gpio_pull_up(sda);
    uint32_t both_pins = (1u << sda) | (1u << scl);
    pio_sm_set_pindirs_with_mask(sm->pio, sm->index, both_pins, both_pins);
    pio_sm_set_pins_with_mask(sm->pio, sm->index, both_pins, both_pins);

    pio_gpio_init(sm->pio, scl);
    gpio_set_oeover(scl, GPIO_OVERRIDE_INVERT);
    pio_gpio_init(sm->pio, sda);
    gpio_set_oeover(sda, GPIO_OVERRIDE_INVERT);

    pio_sm_set_pins_with_mask(sm->pio, sm->index, 0, both_pins);

    // Initialize the state machine
    pio_sm_init(sm->pio, sm->index, sm->offset, &(sm->config));
    pio_sm_set_enabled(sm->pio, sm->index, true);

    pio_sm_clear_fifos(sm->pio, sm->index);

}

void sm_soft_reboot(struct State_Machine *sm){
    pio_sm_set_enabled(sm->pio, sm->index, false);

    gpio_put(VCC, false);
    sleep_ms(250);
    gpio_put(VCC, true);

    sm_init(sm, SDA, SCL);
}

int channel_open(struct State_Machine *sm, struct bmp_sensor *bmp){
    if (!(bmp->pca_ptr->open)){

        if(write_i2c(sm, bmp->pca_ptr->address, &(bmp->pca_ptr->channel), 1) != 0){
            //Something happened to the switch   
            return CHANNEL_CLOSED;
        }
        bmp->pca_ptr->open = true;

    }
    return OK;
}

void bmp_init(struct State_Machine *sm, struct bmp_sensor *bmp){

    uint8_t init_seq[2] = {BMP_PWR_CTRL, 0x33};
    int success = 0;

    printf("???\n");
    if (channel_open(sm, bmp) == CHANNEL_CLOSED){
        printf("Here\n");
        return;
    }
    
    success = bmp_status_refresh(sm, bmp, write_i2c(sm, bmp->address, init_seq, sizeof(init_seq)));
    
    //Activate and (re)initialize variables    
    if (success == 0){
        printf("!!!\n");
        bmp->active = true;
        bmp->press_raw = 0;
        bmp->temp_raw = 0;
        bmp->error_num = 0;
        bmp->error_rate = 0;
        bmp->last_error = 0;
        bmp->current_error = 0;
        bmp->flag = 0;
    }

    return;
}

void bmp_get_calib(struct State_Machine *sm, struct bmp_sensor *bmp){
    
    uint8_t nvm_buffer[21];

    if (channel_open(sm, bmp) == CHANNEL_CLOSED){
        return;
    }

    //BMP automatically increases the register pointer at every read
    if(bmp_read(sm, bmp, BMP_CALIB_REG_FIRST, nvm_buffer, sizeof(nvm_buffer)) != OK){
        return;
    }

    //Temperature coefficients
    bmp->calib_val[0] = (uint16_t) (nvm_buffer[0] | (nvm_buffer[1] << 8))   * (float) (1llu << 8);
    bmp->calib_val[1] = (uint16_t) (nvm_buffer[2] | (nvm_buffer[3] << 8))   / (float) (1llu << 30);
    bmp->calib_val[2] = (int8_t) nvm_buffer[4]                              / (float) (1llu << 48);

    //Pressure coeficients
    bmp->calib_val[3] = ((int16_t) (nvm_buffer[5] | (nvm_buffer[6] << 8)) - (1 << 14)) / (float) (1llu << 20);
    bmp->calib_val[4] = ((int16_t) (nvm_buffer[7] | (nvm_buffer[8] << 8)) - (1 << 14)) / (float) (1llu << 29);
    bmp->calib_val[5] = (int8_t) nvm_buffer[9]                              / (float) (1llu << 32);
    bmp->calib_val[6] = (int8_t) nvm_buffer[10]                             / (float) (1llu << 37);
    bmp->calib_val[7] = (uint16_t) (nvm_buffer[11] | (nvm_buffer[12] << 8)) * (float) (1llu << 3);
    bmp->calib_val[8] = (uint16_t) (nvm_buffer[13] | (nvm_buffer[14] << 8)) / (float) (1llu << 6);
    bmp->calib_val[9] = (int8_t) nvm_buffer[15]                             / (float) (1llu << 8);
    bmp->calib_val[10] = (int8_t) nvm_buffer[16]                            / (float) (1llu << 15);
    bmp->calib_val[11] = (int16_t) (nvm_buffer[17] | (nvm_buffer[18] << 8)) / (float) (1llu << 48);
    bmp->calib_val[12] = (int8_t) nvm_buffer[19]                            / (float) (1llu << 48); 
    bmp->calib_val[13] = (int8_t) nvm_buffer[20]                            / ((float) (1llu << 63) * 2.0f * 2.0f); 
    
}

int bmp_write(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t data[], uint8_t data_len){

    bmp_status_check(bmp);

    if (!(bmp->active)){
        return BMP_INACTIVE;
    }
    
    // Attempts to open channel
    if (channel_open(sm, bmp) == CHANNEL_CLOSED){
        return CHANNEL_CLOSED;
    }

    return bmp_status_refresh(sm, bmp, write_i2c(sm, bmp->address, data, data_len));
}

int bmp_read(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t reg, uint8_t data[], uint8_t data_len){
    bmp_status_check(bmp);

    if (!(bmp->active)){
        return BMP_INACTIVE;
    }

    // Attempts to open channel
    if (channel_open(sm, bmp) == CHANNEL_CLOSED){
        return CHANNEL_CLOSED;
    }

    return bmp_status_refresh(sm, bmp, read_i2c(sm, bmp->address, reg, data, data_len));
}


int bmp_status_refresh(struct State_Machine *sm, struct bmp_sensor *bmp, int64_t error){

    if(error == 0){
        return OK;
    } else if (error > 0){
        bmp->last_error = bmp->current_error;
        bmp->current_error = error;
        bmp->error_num += 1;
        return NOT_OK;
    } else {
        sm_soft_reboot(sm);
        return NOT_OK;
    }
}

int bmp_status_check(struct bmp_sensor *bmp){
    uint32_t time_interval = bmp->current_error - bmp->last_error;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Measuring the rate by intervals bigger than a second mitigates glitches effects
    if ((bmp->current_error != 0) && (time_interval > SEC_IN_MS)){

        // Given in err per sec
        bmp->error_rate = (float) (bmp->error_num * SEC_IN_MS) / (bmp->current_error - bmp->last_error);
        bmp->error_num = 0; 
    } 
    
    // Decrements flag status if sensor has not given considerable errors recently
    if(bmp->flag != SENSOR_OK && (now - bmp->current_error > GRACE_PERIOD)){
        bmp->flag += 1;
        bmp->current_error = now;
        printf("Sensor has been pardoned.\n");
    }

    if (bmp->error_rate > ERROR_THRESHOLD){
        switch (bmp->flag){
        case SENSOR_OK:
            bmp->flag = SENSOR_FLAGGED;
            printf("Sensor has been flagged.\n");
            break;
        case SENSOR_FLAGGED:
            bmp->flag = SENSOR_WARNED;
            printf("Sensor has been warned.\n");
            break;
        case SENSOR_WARNED:
            bmp->active = false;
            printf("Sensor has been deactivated.\n");
            break;
        default:
            // panic(), unknown status (?)
            break;
        }
    }
    
    return bmp->active;
}

int line_check(uint sda, uint scl){
    // Line should be HIGH after STOP. Only the slave can keep it low
    if (gpio_is_pulled_down(sda) || gpio_is_pulled_down(scl)){
        return LINE_DOWN;
    }
    return OK;
}

void bmp_get_pressure(struct State_Machine *sm, struct bmp_sensor *bmp){

    //Stores up to three values to (possibly) be used in interpolation/approximation
    bmp->comp_press[2] = bmp->comp_press[1];
    bmp->comp_press[1] = bmp->comp_press[0]; 
    
    bmp->comp_temp[2] = bmp->comp_temp[1];
    bmp->comp_temp[1] = bmp->comp_temp[0];

    //Temperature is used in compensation
    uint8_t buffer[6] = {0};
    
    if (bmp_read(sm, bmp, BMP_PRESS_REG_LSB, buffer, 6) != OK){
        //Invalid value, do a linear approximation
        bmp->comp_press[0] = (2 * bmp->comp_press[1]) - bmp->comp_press[2];
        return;
    }

    bmp->temp_raw = (buffer[5] << 16) | (buffer[4] << 8) | buffer[3];
    bmp->press_raw = (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];

    bmp_compensate_pressure(bmp);

    if ((bmp->comp_press[0] < MIN_BMP_PRESS) 
        || (bmp->comp_press[0] > MAX_BMP_PRESS)
        || ((bmp->comp_press[0] - bmp->comp_press[1] > MAX_BMP_VAR_PRESS) && bmp->comp_press[1] != 0)
        || ((bmp->comp_press[1] - bmp->comp_press[0] > MAX_BMP_VAR_PRESS) && bmp->comp_press[1] != 0)){

        //Invalid value, do a linear approximation
        bmp->comp_press[0] = (2 * bmp->comp_press[1]) - bmp->comp_press[2];
    }

}

float bmp_compensate_temperature(struct bmp_sensor *bmp){
    float partial_data1;
    float partial_data2;

    partial_data1 = (float) (bmp->temp_raw - bmp->calib_val[0]);
    partial_data2 = (float) (partial_data1 * bmp->calib_val[1]);

    bmp->comp_temp[0] = partial_data2 + (partial_data1 * partial_data1) * bmp->calib_val[2];

    return bmp->comp_temp[0];
}

float bmp_compensate_pressure(struct bmp_sensor *bmp){
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;

    float partial_out1;
    float partial_out2;

    bmp_compensate_temperature(bmp);

    if ((bmp->comp_temp[0] < MIN_BMP_TEMP) || (bmp->comp_temp[0] > MAX_BMP_TEMP)){
        //Invalid value, do a linear approximation
        bmp->comp_temp[0] = (2 * bmp->comp_temp[1]) - bmp->comp_temp[2];
    }

    partial_data1 = bmp->calib_val[8] * bmp->comp_temp[0];
    partial_data2 = bmp->calib_val[9] * (bmp->comp_temp[0] * bmp->comp_temp[0]);
    partial_data3 = bmp->calib_val[10] * (bmp->comp_temp[0] * bmp->comp_temp[0] * bmp->comp_temp[0]);
    partial_out1 = bmp->calib_val[7] + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = bmp->calib_val[4] * bmp->comp_temp[0];
    partial_data2 = bmp->calib_val[5] * (bmp->comp_temp[0] * bmp->comp_temp[0]);
    partial_data3 = bmp->calib_val[6] * (bmp->comp_temp[0] * bmp->comp_temp[0] * bmp->comp_temp[0]);
    partial_out2 = (float) bmp->press_raw * (bmp->calib_val[3] + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float) bmp->press_raw * (float) bmp->press_raw;
    partial_data2 = bmp->calib_val[11] + bmp->calib_val[12] * bmp->comp_temp[0];
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float) bmp->press_raw * (float) bmp->press_raw * (float) bmp->press_raw) * bmp->calib_val[13];

    bmp->comp_press[0] = partial_out1 + partial_out2 + partial_data4;

    return bmp->comp_press[0];
}

void bmp_calib_file_helper(struct bmp_sensor *bmp){
    if (!(bmp->active)){
        return;
    }

    gpio_put(UART_ENABLE, true);
    sleep_us(250);

    char buffer_out[32];
    snprintf(buffer_out, sizeof(buffer_out), "Addr - 0x%02x, Channel - %u\n", bmp->address, bmp->pca_ptr->channel);
    uart_puts(UART_ID, buffer_out);

    for (size_t i = 0; i < 14; i++){
        char buffer[16];

        snprintf(buffer, sizeof(buffer), "%.3e\n", bmp->calib_val[i]);
        uart_puts(UART_ID, buffer);
    }

    sleep_us(250);
    gpio_put(UART_ENABLE, true);
}

void bmp_press_file_helper(struct bmp_sensor *bmp){

    char buffer[16];

    if (!(bmp->active)){
            return;
    }

    snprintf(buffer, sizeof(buffer), "%.0f,", bmp->comp_press[0]);
    uart_puts(UART_ID, buffer);
    
}

int64_t write_i2c(struct State_Machine *sm, uint8_t addr, uint8_t data[], uint8_t data_len){

    //Stores the timestamp of a NACK
    int64_t acks = 0;                                        
    
    //Pulling from the left in the RX FIFO
    pio_sm_put(sm->pio, sm->index, data_len << 24);
    
    //Add write "0" bit
    pio_sm_put_blocking(sm->pio, sm->index, addr << 25);          
    acks = pio_sm_get_blocking(sm->pio, sm->index);          

    for (size_t i = 0; i < data_len; i++){
        pio_sm_put(sm->pio, sm->index, data[i] << 24);
        acks += pio_sm_get_blocking(sm->pio, sm->index);
    }

    //Stop condition
    pio_sm_put(sm->pio, sm->index, FILL_TXF);

    if (acks > 0){
        acks = to_ms_since_boot(get_absolute_time());
    } 
    
    if (line_check(SDA, SCL) == LINE_DOWN){
        acks = LINE_DOWN;
    }

    return acks;
}

int64_t read_i2c(struct State_Machine *sm, uint8_t addr, uint8_t reg, uint8_t rxbuff[], uint8_t data_len){

    int64_t acks = write_i2c(sm, addr, &reg, 1);

    if(acks != 0){
        return acks;
    }

    //Message with 0 length
    pio_sm_put(sm->pio, sm->index, 0x00);

    //Add a read, "1", bit
    pio_sm_put_blocking(sm->pio, sm->index, ((addr << 1) + 1) << 24);    

    //Aborts reading if NACK to save time
    if (pio_sm_get_blocking(sm->pio, sm->index) > 0){
        pio_sm_put(sm->pio, sm->index, FILL_TXF);
        return to_ms_since_boot(get_absolute_time());
    }
    
    //Read condition
    pio_sm_put(sm->pio, sm->index, 0x00);

    for (size_t i = 0; i < data_len-1; i++){                            
        rxbuff[i] = pio_sm_get_blocking(sm->pio, sm->index);
        pio_sm_put(sm->pio, sm->index, 0x00);
    }

    rxbuff[data_len-1] = pio_sm_get_blocking(sm->pio, sm->index);

    //Stop condition
    pio_sm_put(sm->pio, sm->index, FILL_TXF);
    
    //Only possible error after all the writing checks
    return line_check(SDA, SCL);
}


