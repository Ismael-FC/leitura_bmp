#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pico/multicore.h"

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
    
    sm->sda = sda;
    sm->scl = scl;

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

int channel_open(struct State_Machine *sm, message *msg, uint blacklist){

    int success = 0;
    int j = 0;
        
    switch (msg->addr){
    case PCA_ADDRESS_BOTTOM:
        j = 0;
        break;
    case PCA_ADDRESS_MID:
        j = 1;
        break;
    case PCA_ADDRESS_TOP:
        j = 2;
        break;
    default:
    //panic(Unknown Address)
        break;
    }

    for (size_t i = 0; i < sm[i].active_sm; i++){
        if (msg->txbuff[0] != sm[i].pca_ptr[j].open_channel){
            continue;
        } else {
            return i;
        }
    }
    
    success = write_i2c(sm, msg, blacklist);

    for (size_t i = 0; i < sm[0].active_sm; i++){
        if (msg->rxbuff[0][i] > 0 && (blacklist & (1u << i))){
            continue;
        }

        sm[i].pca_ptr[j].open_channel = msg->txbuff[0];

    }
    
    return success;
}

void bmp_init(struct State_Machine *sm){

    uint32_t init_seq[2] = {BMP_PWR_CTRL, 0x33};
    int success = 0;

    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        uint blacklist = 0;

        for (size_t k = 0; k < sm[0].active_sm; k++){
            if ((~sm[k].active_switch & (1U << i))){
                blacklist |= (1 << k);
            }
        }

        for (size_t j = 0; j < MAX_BMP_PER_SWITCH; j++){
            uint32_t acks_switch[1][5] = {0};
            message msg;

            message_constructor(&msg, sm[0].pca_ptr[i].address, &sm[0].pca_ptr[i].bmp[j].channel, acks_switch, 1);
            
            channel_open(sm, &msg, blacklist);
            
            uint32_t acks_msg[1][5] = {0};
            message_constructor(&msg, sm[0].pca_ptr[i].bmp[j].address, init_seq, acks_msg, sizeof(init_seq)/4);

            success = write_i2c(sm, &msg, blacklist);

            for (size_t k = 0; k < sm[0].active_sm; k++){
                if (acks_msg[0][k] > 0){
                    continue;
                }

                //Activate and (re)initialize variables
                sm[k].pca_ptr[i].bmp[j].comp_temp[0] = 0;
                sm[k].pca_ptr[i].bmp[j].comp_temp[1] = 0;
                sm[k].pca_ptr[i].bmp[j].comp_temp[2] = 0;

                sm[k].pca_ptr[i].bmp[j].comp_press[0] = 0;
                sm[k].pca_ptr[i].bmp[j].comp_press[1] = 0;
                sm[k].pca_ptr[i].bmp[j].comp_press[2] = 0;

                sm[k].pca_ptr[i].bmp[j].active = true;
                sm[k].pca_ptr[i].bmp[j].press_raw = 0;
                sm[k].pca_ptr[i].bmp[j].temp_raw = 0;
                sm[k].pca_ptr[i].bmp[j].error_num = 0;
                sm[k].pca_ptr[i].bmp[j].error_rate = 0;
                sm[k].pca_ptr[i].bmp[j].last_error = 0;
                sm[k].pca_ptr[i].bmp[j].current_error = 0;
                sm[k].pca_ptr[i].bmp[j].flag = 0;
            }
        }
    }

    return;
}

void bmp_get_calib(struct State_Machine *sm){
    
    uint32_t nvm_buffer[21][MAX_SM];

    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        uint blacklist = 0;

        for (size_t k = 0; k < sm[0].active_sm; k++){
            if ((~sm[k].active_switch & (1U << i))){
                blacklist |= (1 << k);
            }
        }

        for (size_t j = 0; j < MAX_BMP_PER_SWITCH; j++){
            uint32_t acks_switch[1][5] = {0};
            message msg;

            message_constructor(&msg, sm[0].pca_ptr[i].address, &sm[0].pca_ptr[i].bmp[j].channel, acks_switch, 1);
            channel_open(sm, &msg, blacklist);

            uint32_t buffer[1] = {BMP_CALIB_REG_FIRST};

            message_constructor(&msg, sm[0].pca_ptr[i].bmp[j].address, buffer, nvm_buffer, 21);
            read_i2c(sm, &msg, blacklist);

            for (size_t k = 0; k < sm[0].active_sm; k++){
                
                // Temperature coefficients
                sm[k].pca_ptr[i].bmp[j].calib_val[0] = (uint16_t) (nvm_buffer[0][k] | (nvm_buffer[1][k] << 8)) * (float) (1llu << 8);
                sm[k].pca_ptr[i].bmp[j].calib_val[1] = (uint16_t) (nvm_buffer[2][k] | (nvm_buffer[3][k] << 8)) / (float) (1llu << 30);
                sm[k].pca_ptr[i].bmp[j].calib_val[2] = (int8_t) nvm_buffer[4][k]                               / (float) (1llu << 48);

                //Pressure coeficients
                 sm[k].pca_ptr[i].bmp[j].calib_val[3] = ((int16_t) (nvm_buffer[5][k] | (nvm_buffer[6][k] << 8)) - (1 << 14)) / (float) (1llu << 20);
                 sm[k].pca_ptr[i].bmp[j].calib_val[4] = ((int16_t) (nvm_buffer[7][k] | (nvm_buffer[8][k] << 8)) - (1 << 14)) / (float) (1llu << 29);
                 sm[k].pca_ptr[i].bmp[j].calib_val[5] = (int8_t) nvm_buffer[9][k]    / (float) (1llu << 32);
                 sm[k].pca_ptr[i].bmp[j].calib_val[6] = (int8_t) nvm_buffer[10][k]    / (float) (1llu << 37);
                 sm[k].pca_ptr[i].bmp[j].calib_val[7] = (uint16_t) (nvm_buffer[11][k] | (nvm_buffer[12][k] << 8)) * (float) (1llu << 3);
                 sm[k].pca_ptr[i].bmp[j].calib_val[8] = (uint16_t) (nvm_buffer[13][k] | (nvm_buffer[14][k] << 8)) / (float) (1llu << 6);
                 sm[k].pca_ptr[i].bmp[j].calib_val[9] = (int8_t) nvm_buffer[15][k]    / (float) (1llu << 8);
                 sm[k].pca_ptr[i].bmp[j].calib_val[10] = (int8_t) nvm_buffer[16][k]   / (float) (1llu << 15);
                 sm[k].pca_ptr[i].bmp[j].calib_val[11] = (int16_t) (nvm_buffer[17][k] | (nvm_buffer[18][k] << 8)) / (float) (1llu << 48);
                 sm[k].pca_ptr[i].bmp[j].calib_val[12] = (int8_t) nvm_buffer[19][k]   / (float) (1llu << 48); 
                 sm[k].pca_ptr[i].bmp[j].calib_val[13] = (int8_t) nvm_buffer[20][k]   / ((float) (1llu << 63) * 2.0f * 2.0f); 
            }
        }
    }
}

/* TODO */
int bmp_write(struct State_Machine *sm, struct bmp_sensor *bmp, uint8_t data[], uint8_t data_len){

    // bmp_status_check(bmp);

    // if (!(bmp->active)){
    //     return BMP_INACTIVE;
    // }
    
    // Attempts to open channel
    // if (channel_open(sm, bmp) == CHANNEL_CLOSED){
    //     return CHANNEL_CLOSED;
    // }

    // return bmp_status_refresh(sm, bmp, write_i2c(sm, bmp->address, data, data_len, 0));
    return 0;
}

/* TODO */
int bmp_read(struct State_Machine *sm, message *msg, size_t s, size_t b){
    
    uint blacklist = 0;
    uint counter = 0;

    bmp_status_check(sm, s, b);

    for (size_t i = 0; i < sm[0].active_sm; i++){
        if (!(sm[i].pca_ptr[s].bmp[b].active)){
            blacklist |= (1 << i);
            counter++;
        }
    }

    if (counter == sm[0].active_sm){
        return 1;
    }

    read_i2c(sm, msg, blacklist);

    bmp_status_refresh(sm, s, b, msg->txbuff);

    return 0;

    // return bmp_status_refresh(sm, bmp, read_i2c(sm, bmp->address, reg, data, data_len));
}

/* TODO */
int bmp_status_refresh(struct State_Machine *sm, size_t s, size_t b, uint32_t *errors){

    for (size_t i = 0; i < sm->active_sm; i++){
        if(errors[i] == 0){
            continue; 
        } else {
            sm[i].pca_ptr[s].bmp[b].last_error = sm[i].pca_ptr[s].bmp[b].current_error;
            sm[i].pca_ptr[s].bmp[b].current_error = errors[i];
            sm[i].pca_ptr[s].bmp[b].error_num += 1;
        } 
    }

    return 0;
}

/* TODO */
int bmp_status_check(struct State_Machine *sm, size_t s, size_t b){

    for (size_t i = 0; i < sm->active_sm; i++){
        if (!(sm[i].pca_ptr[s].bmp[b].active)){
            continue;
        }
        
        uint32_t time_interval = sm[i].pca_ptr[s].bmp[b].current_error - sm[i].pca_ptr[s].bmp[b].last_error;
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Measuring the rate by intervals bigger than a second mitigates glitches effects
        if ((sm[i].pca_ptr[s].bmp[b].current_error != 0) && (time_interval > SEC_IN_MS)){

            // Given in err per sec
            sm[i].pca_ptr[s].bmp[b].error_rate = (float) (sm[i].pca_ptr[s].bmp[b].error_num * SEC_IN_MS) / (time_interval);
            sm[i].pca_ptr[s].bmp[b].error_num = 0;
            printf("Error - %f\n", sm[i].pca_ptr[s].bmp[b].error_rate);

        } 
        
        // Decrements flag status if sensor has not given considerable errors recently
        if(sm[i].pca_ptr[s].bmp[b].flag != SENSOR_OK && (now - sm[i].pca_ptr[s].bmp[b].current_error > GRACE_PERIOD)){
            sm[i].pca_ptr[s].bmp[b].flag += 1;
            sm[i].pca_ptr[s].bmp[b].current_error = now;
            printf("Sensor has been pardoned.");
        }

        if (sm[i].pca_ptr[s].bmp[b].error_rate > ERROR_THRESHOLD){
            switch (sm[i].pca_ptr[s].bmp[b].flag){
                case SENSOR_OK:
                    sm[i].pca_ptr[s].bmp[b].flag = SENSOR_FLAGGED;
                    sm[i].pca_ptr[s].bmp[b].error_rate = 0;
                    printf("Sensor has been flagged. Address - 0x%x, PCA - %d, SM - %d\n", sm[i].pca_ptr[s].bmp[b].address, s, i);
                    break;
                case SENSOR_FLAGGED:
                    sm[i].pca_ptr[s].bmp[b].flag = SENSOR_WARNED;
                    sm[i].pca_ptr[s].bmp[b].error_rate = 0;
                    printf("Sensor has been warned. 0x%x, PCA - %d, SM - %d\n", sm[i].pca_ptr[s].bmp[b].address, s, i);
                    break;
                case SENSOR_WARNED:
                    sm[i].pca_ptr[s].bmp[b].active = false;
                    printf("Sensor has been deactivated. 0x%x, PCA - %d, SM - %d\n", sm[i].pca_ptr[s].bmp[b].address, s, i);
                    break;
                default:
                    // panic(), unknown status (?)
                    break;
            }
        }
    }

    return 0;
}

int line_check(uint sda, uint scl){
    // Line should be HIGH after STOP. Only the slave can keep it low
    if (gpio_is_pulled_down(sda) || gpio_is_pulled_down(scl)){
        return LINE_DOWN;
    }
    return OK;
}

/* TODO */
void bmp_get_pressure(struct State_Machine *sm){

    //Stores up to three values to (possibly) be used in interpolation/approximation
    // bmp->comp_press[2] = bmp->comp_press[1];
    // bmp->comp_press[1] = bmp->comp_press[0]; 
    
    // bmp->comp_temp[2] = bmp->comp_temp[1];
    // bmp->comp_temp[1] = bmp->comp_temp[0];

    //Temperature is used in compensation
    
    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        uint outer_counter = 0;
        uint blacklist = 0;

        for (size_t k = 0; k < sm[0].active_sm; k++){
            if ((~sm[k].active_switch & (1U << i))){
                blacklist |= (1 << k);
                outer_counter++;
            }
        }
        if (outer_counter == sm[0].active_sm){
            continue;
        }
        
        for (size_t j = 0; j < MAX_BMP_PER_SWITCH; j++){
            uint32_t acks_switch[1][5] = {0};
            message msg;

            message_constructor(&msg, sm[0].pca_ptr[i].address, &sm[0].pca_ptr[i].bmp[j].channel, acks_switch, 1);
            channel_open(sm, &msg, blacklist);

            uint32_t buffer_tx[1] = {BMP_PRESS_REG_LSB+1};
            uint32_t buffer_rx[5][5] = {0};

            message_constructor(&msg, sm[0].pca_ptr[i].bmp[j].address, buffer_tx, buffer_rx, 5);
            
            if(bmp_read(sm, &msg, i, j) == 1){
                continue;
            }

            multicore_fifo_push_blocking(sm[0].active_sm);
            multicore_fifo_push_blocking((uint32_t) &bmp_compensate_pressure);

            for (size_t k = 0; k < sm->active_sm; k++){
                sm[k].pca_ptr[i].bmp[j].temp_raw = (buffer_rx[4][k] << 16) | (buffer_rx[3][k] << 8) | 0x00;
                sm[k].pca_ptr[i].bmp[j].press_raw = (buffer_rx[1][k] << 16) | (buffer_rx[0][k] << 8) | 0x00;

                multicore_fifo_push_blocking((uint32_t) &sm[k].pca_ptr[i].bmp[j]);
            }
        }
    }

    
    // if (bmp_read(sm, bmp, BMP_PRESS_REG_LSB, buffer, 6) != OK){
    //     //Invalid value, do a linear approximation
    //     bmp->comp_press[0] = (2 * bmp->comp_press[1]) - bmp->comp_press[2];
    //     return;
    // }

    // bmp->temp_raw = (buffer[5] << 16) | (buffer[4] << 8) | buffer[3];
    // bmp->press_raw = (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];

    // bmp_compensate_pressure(bmp);

    // if ((bmp->comp_press[0] < MIN_BMP_PRESS) 
    //     || (bmp->comp_press[0] > MAX_BMP_PRESS)
    //     || ((bmp->comp_press[0] - bmp->comp_press[1] > MAX_BMP_VAR_PRESS) && bmp->comp_press[1] != 0)
    //     || ((bmp->comp_press[1] - bmp->comp_press[0] > MAX_BMP_VAR_PRESS) && bmp->comp_press[1] != 0)){

    //     //Invalid value, do a linear approximation
    //     bmp->comp_press[0] = (2 * bmp->comp_press[1]) - bmp->comp_press[2];
    // }

}

/* TODO */
float bmp_compensate_temperature(struct bmp_sensor *bmp){
    float partial_data1;
    float partial_data2;

    partial_data1 = (float) (bmp->temp_raw - bmp->calib_val[0]);
    partial_data2 = (float) (partial_data1 * bmp->calib_val[1]);

    bmp->comp_temp[0] = partial_data2 + (partial_data1 * partial_data1) * bmp->calib_val[2];

    return bmp->comp_temp[0];
}

/* TODO */
float bmp_compensate_pressure(struct bmp_sensor *bmp){
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;

    float partial_out1;
    float partial_out2;

    bmp->comp_press[2] = bmp->comp_press[1];
    bmp->comp_press[1] = bmp->comp_press[0]; 
    
    bmp->comp_temp[2] = bmp->comp_temp[1];
    bmp->comp_temp[1] = bmp->comp_temp[0];

    bmp_compensate_temperature(bmp);

    if ((bmp->comp_temp[0] < MIN_BMP_TEMP) || (bmp->comp_temp[0] > MAX_BMP_TEMP) && bmp->comp_temp[2] != 0){
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

    if ((bmp->comp_press[0] < MIN_BMP_PRESS) 
        || (bmp->comp_press[0] > MAX_BMP_PRESS)
        || ((bmp->comp_press[0] - bmp->comp_press[1] > MAX_BMP_VAR_PRESS) && bmp->comp_press[2] != 0)
        || ((bmp->comp_press[1] - bmp->comp_press[0] > MAX_BMP_VAR_PRESS) && bmp->comp_press[2] != 0)){

        //Invalid value, do a linear approximation
        bmp->comp_press[0] = (2 * bmp->comp_press[1]) - bmp->comp_temp[2];
        
    }

    return bmp->comp_press[0];
}

void bmp_calib_file_helper(struct State_Machine *sm){

    for (size_t i = 0; i < sm[0].active_sm; i++){
    char buffer_outter[64];
    snprintf(buffer_outter, sizeof(buffer_outter), "SM Index - %d, SM Switches - %d\n", sm[i].index, sm[i].active_switch);
    uart_puts(UART_ID, buffer_outter);

        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            if (!(sm[i].pca_ptr[j].status)){
                continue;
            }
            char buffer_out[64];
                snprintf(buffer_out, sizeof(buffer_out), "PCA Address - 0x%x, Activity - %s\n", 
                        sm[i].pca_ptr[j].address, sm[i].pca_ptr[j].status ? "True" : "False");
                uart_puts(UART_ID, buffer_out);

            for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
                if (!(sm[i].pca_ptr[j].bmp[k].active)){
                continue;
            }
                for (size_t l = 0; l < 14; l++){
                    char buffer_out[64];
                    snprintf(buffer_out, sizeof(buffer_out), "BMP Address - 0x%x, BMP Channel - %d, BMP Calib - %.3e\n", 
                        sm[i].pca_ptr[j].bmp[k].address, sm[i].pca_ptr[j].bmp[k].channel, sm[i].pca_ptr[j].bmp[k].calib_val[l]);
                    uart_puts(UART_ID, buffer_out);
                }
            }
        }
    }

}

/* TODO */
void bmp_press_file_helper(struct State_Machine *sm){

    for (size_t i = 0; i < MAX_SM; i++){
        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
                if (!(sm[i].pca_ptr[j].bmp[k].active)){    
                    continue;
                }

                char buffer_in[64];
                snprintf(buffer_in, sizeof(buffer_in), "%.2f,", sm[i].pca_ptr[j].bmp[k].comp_press[0]);
                uart_puts(UART_ID, buffer_in);
            }
        }

        uart_puts(UART_ID, "\n");
    }
}

uint32_t write_i2c(struct State_Machine *sm, message *msg, uint blacklist){

    send_msgs(pio_sm_put, sm, msg->data_len << 24, blacklist);
    send_msgs(pio_sm_put_blocking, sm, msg->addr << 25, blacklist);

    receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, 0, blacklist);

    for (size_t i = 0; i < msg->data_len; i++){   
        send_msgs(pio_sm_put, sm, msg->txbuff[i] << 24, blacklist);
        receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, 0, blacklist);
    }
    
    send_msgs(pio_sm_put, sm, FILL_TXF, blacklist);

    for (size_t i = 0; i < sm[0].active_sm; i++){
        if (msg->rxbuff[0][i] > 0 && !(blacklist & (1u << i))){
           msg->rxbuff[0][i] = to_ms_since_boot(get_absolute_time());
        }

        if (line_check(sm[i].sda, sm[i].scl) == LINE_DOWN){
            sm_soft_reboot(&sm[i]);
        }
    }

    return 0;
}

uint32_t read_i2c(struct State_Machine *sm, message *msg, uint32_t blacklist){

    uint32_t rx_len = msg->data_len;
    msg->data_len = 1;

    write_i2c(sm, msg, blacklist);
    msg->data_len = rx_len;

    for (size_t i = 0; i < sm[0].active_sm; i++){
        msg->txbuff[i] = 0;
        if (msg->rxbuff[0][i] > 0 && !(blacklist & (1u << i))){
            blacklist |= (1 << i);
            msg->txbuff[i] = to_ms_since_boot(get_absolute_time());
        }
    }
    
    send_msgs(pio_sm_put, sm, 0x00, blacklist);
    send_msgs(pio_sm_put_blocking, sm, ((msg->addr << 1) + 1) << 24, blacklist);

    receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, 0, blacklist);
    send_msgs(pio_sm_put, sm, 0x00, blacklist);

    size_t counter = 0;
    for (counter = 0; counter < msg->data_len-1; counter++){    
        receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, counter, blacklist);
        send_msgs(pio_sm_put, sm, 0x00, blacklist);
    }

    receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, counter, blacklist);
    send_msgs(pio_sm_put, sm, FILL_TXF, blacklist);
    
    for (size_t i = 0; i < sm[0].active_sm; i++){
        if (line_check(sm[i].sda, sm[i].scl) == LINE_DOWN){
            sm_soft_reboot(&sm[i]);
        }
    }
    
    return 0;
}

int send_msgs(void (*put)(PIO, uint, uint32_t), struct State_Machine *sm, uint32_t data, uint blacklist){
    for (size_t i = 0; i < sm[0].active_sm; i++){
        if (blacklist & (1u << i)){
            continue;
        }
        
        (*put)(sm[i].pio, sm[i].index, data);
    }

    return 0;
}

int receive_msgs(uint32_t (*get)(PIO, uint), struct State_Machine *sm, uint32_t (*buffer)[5], size_t index, uint blacklist){
    for (size_t i = 0; i < sm[0].active_sm; i++){
        if (blacklist & (1u << i)){
            if (buffer[index][i] == 0){
                buffer[index][i] = 1;
            }
            continue;
        }
        
        buffer[index][i] = (*get)(sm[i].pio, sm[i].index);
    }

    return 0;
}

void message_constructor(message *msg, uint8_t addr, uint32_t *txbuff, uint32_t (*rxbuff)[5], uint data_len){
    msg->addr = addr;
    msg->txbuff = txbuff;
    msg->rxbuff = rxbuff;
    msg->data_len = data_len;
}