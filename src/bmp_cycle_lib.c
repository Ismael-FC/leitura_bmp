#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"

#include "bmp_cycle.h"
#include "bmp_config.h"
#include "bmp_cycle.pio.h"

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

void sm_init(StateMachine *sm, uint sda, uint scl){

    sm->config = bmp_cycle_program_get_default_config(sm->offset);
    
    sm->sda = sda;
    sm->scl = scl;

    pio_sm_set_consecutive_pindirs(sm->pio, sm->index, sda, 2, true);

    // State machine configuration
    sm_config_set_in_pins(&(sm->config), sda);
    sm_config_set_out_pins(&(sm->config), sda, 1);
    sm_config_set_set_pins(&(sm->config), sda, 1);
    sm_config_set_sideset_pins(&(sm->config), scl);
    sm_config_set_jmp_pin(&(sm->config), scl);

    //Receive data MSB first (left-shift), autopull every 8 bits
    sm_config_set_in_shift(&(sm->config), false, true, 8);

    //Transmit data MSB first (left-shift), autopush every 8 bits
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

void sm_soft_reboot(StateMachine *sm){
    //Disable and reenable a single state machine
    pio_sm_set_enabled(sm->pio, sm->index, false);

    gpio_put(VCC, false);
    sleep_ms(250);
    gpio_put(VCC, true);

    sm_init(sm, SDA, SCL);
}

int channel_open(StateMachine *sm, Message *msg, uint blacklist){

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

    for (size_t i = 0; i < sm->activeSm; i++){
        if (msg->txbuff[0] != sm[i].p_i2cSwitch[j].open_channel){
            continue;
        } else {
            // Return if all switches have the same open channel
            return i;
        }
    }
    
    write_i2c(sm, msg, blacklist);

    for (size_t i = 0; i < sm->activeSm; i++){
        if (msg->rxbuff[0][i] > 0 || (blacklist & (1u << i))){
            continue;
        }
        //Set open channel to all active switches
        sm[i].p_i2cSwitch[j].open_channel = msg->txbuff[0];
    }
    
    return 0;
}

void bmp_init(StateMachine *sm){
    //Initialization sequence: 0x33 = enable temperature and pressure, 0x31 only pressure (not recommended)
    uint32_t init_seq[2] = {BMP_PWR_CTRL, 0x33};

    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        //Blacklist state machines whose i2c switch does not have sensors
        uint blacklist = blacklist_i2c_switch(sm, i);

        for (size_t j = 0; j < MAX_BMP_PER_SWITCH; j++){
            uint32_t acks_switch[1][MAX_SM] = {0};
            uint32_t acks_msg[1][MAX_SM] = {0};
            Message channel;
            Message init;

            message_constructor(&channel, sm[0].p_i2cSwitch[i].address, &sm[0].p_i2cSwitch[i].p_bmp[j].channel, acks_switch, 1);
            message_constructor(&init, sm[0].p_i2cSwitch[i].p_bmp[j].address, init_seq, acks_msg, sizeof(init_seq)/4);
            
            channel_open(sm, &channel, blacklist);
            write_i2c(sm, &init, blacklist);

            for (size_t k = 0; k < sm->activeSm; k++){
                //Skip if not acknowledged
                if (acks_msg[0][k] > 0){
                    continue;
                }

                //Activate and (re)initialize variables
                sm[k].p_i2cSwitch[i].p_bmp[j].comp_temp[0] = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].comp_temp[1] = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].comp_temp[2] = 0;

                sm[k].p_i2cSwitch[i].p_bmp[j].comp_press[0] = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].comp_press[1] = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].comp_press[2] = 0;

                sm[k].p_i2cSwitch[i].p_bmp[j].active = true;
                sm[k].p_i2cSwitch[i].p_bmp[j].press_raw = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].temp_raw = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].error_num = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].error_rate = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].last_error = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].current_error = 0;

                sm[k].p_i2cSwitch[i].p_bmp[j].flag = 0;
                sm[k].p_i2cSwitch[i].p_bmp[j].check = false;
            }
        }
    }

    return;
}

void bmp_get_calib(StateMachine *sm){
    
    //Stores all calibration values of a single sensor
    uint32_t nvm_buffer[21][MAX_SM];

    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        //Blacklist state machines whose i2c switch does not have sensors
        uint blacklist = blacklist_i2c_switch(sm, i);

        for (size_t j = 0; j < MAX_BMP_PER_SWITCH; j++){
            uint32_t acks_switch[1][5] = {0};
            uint32_t buffer[1] = {BMP_CALIB_REG_FIRST};
            Message channel;
            Message calib;

            message_constructor(&channel, sm[0].p_i2cSwitch[i].address, &sm[0].p_i2cSwitch[i].p_bmp[j].channel, acks_switch, 1);
            message_constructor(&calib, sm[0].p_i2cSwitch[i].p_bmp[j].address, buffer, nvm_buffer, 21);
            
            channel_open(sm, &channel, blacklist);
            read_i2c(sm, &calib, blacklist);

            for (size_t k = 0; k < sm->activeSm; k++){
                
                // Temperature coefficients
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[0] = (uint16_t) (nvm_buffer[0][k] | (nvm_buffer[1][k] << 8)) * (float) (1llu << 8);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[1] = (uint16_t) (nvm_buffer[2][k] | (nvm_buffer[3][k] << 8)) / (float) (1llu << 30);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[2] = (int8_t) nvm_buffer[4][k]                               / (float) (1llu << 48);

                //Pressure coeficients
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[3] = ((int16_t) (nvm_buffer[5][k] | (nvm_buffer[6][k] << 8)) - (1 << 14)) / (float) (1llu << 20);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[4] = ((int16_t) (nvm_buffer[7][k] | (nvm_buffer[8][k] << 8)) - (1 << 14)) / (float) (1llu << 29);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[5] = (int8_t) nvm_buffer[9][k]    / (float) (1llu << 32);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[6] = (int8_t) nvm_buffer[10][k]    / (float) (1llu << 37);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[7] = (uint16_t) (nvm_buffer[11][k] | (nvm_buffer[12][k] << 8)) * (float) (1llu << 3);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[8] = (uint16_t) (nvm_buffer[13][k] | (nvm_buffer[14][k] << 8)) / (float) (1llu << 6);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[9] = (int8_t) nvm_buffer[15][k]    / (float) (1llu << 8);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[10] = (int8_t) nvm_buffer[16][k]   / (float) (1llu << 15);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[11] = (int16_t) (nvm_buffer[17][k] | (nvm_buffer[18][k] << 8)) / (float) (1llu << 48);
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[12] = (int8_t) nvm_buffer[19][k]   / (float) (1llu << 48); 
                sm[k].p_i2cSwitch[i].p_bmp[j].calib_val[13] = (int8_t) nvm_buffer[20][k]   / ((float) (1llu << 63) * 2.0f * 2.0f); 
            }
        }
    }
}


int bmp_write(StateMachine *sm, Message *msg){

    uint blacklist = 0;

    //Helper variables that know exactly which sensor is being addressed
    uint s = msg->switch_i;
    uint b = msg->bmp_j;

    bmp_status_check(sm, s, b);

    //Blacklist every BMP that's not active
    for (size_t i = 0; i < sm->activeSm; i++){
        if (!(sm[i].p_i2cSwitch[s].p_bmp[b].active)){
            blacklist |= (1 << i);
        }
    }

    //Returns if all sensors are inactive across state machines
    if (blacklist == MAX_BMP_MASK){
        return 1;
    }

    write_i2c(sm, msg, blacklist);

    bmp_status_refresh(sm, s, b, msg->txbuff);

    return 0;
}


int bmp_read(StateMachine *sm, Message *msg){
    
    uint blacklist = 0;

    //Helper variables that know exactly which sensor is being addressed
    uint s = msg->switch_i;
    uint b = msg->bmp_j;

    bmp_status_check(sm, s, b);

    //Blacklist every BMP that's not active
    for (size_t i = 0; i < sm->activeSm; i++){
        if (!(sm[i].p_i2cSwitch[s].p_bmp[b].active)){
            blacklist |= (1 << i);
        }
    }

    //Returns if all sensors are inactive across state machines
    if (blacklist == MAX_BMP_MASK){
        return 1;
    }

    read_i2c(sm, msg, blacklist);

    bmp_status_refresh(sm, s, b, msg->txbuff);

    return 0;
}

int bmp_status_refresh(StateMachine *sm, size_t s, size_t b, uint32_t *errors){

    for (size_t i = 0; i < sm->activeSm; i++){
        if(errors[i] == 0){
            continue; 
        } else {
            
            // errors[i] is a timestamp
            sm[i].p_i2cSwitch[s].p_bmp[b].last_error = sm[i].p_i2cSwitch[s].p_bmp[b].current_error;
            sm[i].p_i2cSwitch[s].p_bmp[b].current_error = errors[i];

            //Num is used to determine the error rate
            sm[i].p_i2cSwitch[s].p_bmp[b].error_num += 1;

            //Raise check flag if bmp had an error
            sm[i].p_i2cSwitch[s].p_bmp[b].check = true;
        } 
    }

    return 0;
}


int bmp_status_check(StateMachine *sm, size_t s, size_t b){

    for (size_t i = 0; i < sm->activeSm; i++){
        //Check active bmp's whose flag was raised
        if (!(sm[i].p_i2cSwitch[s].p_bmp[b].active) || !(sm[i].p_i2cSwitch[s].p_bmp[b].check)){
            continue;
        }
        
        uint32_t time_interval = sm[i].p_i2cSwitch[s].p_bmp[b].current_error - sm[i].p_i2cSwitch[s].p_bmp[b].last_error;
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Measuring the rate by intervals bigger than a second mitigates glitches effects
        if ((sm[i].p_i2cSwitch[s].p_bmp[b].current_error != 0) && (time_interval > SEC_IN_MS)){

            // Given in err per sec. Resets num only if the time_interval is bigger than a second
            sm[i].p_i2cSwitch[s].p_bmp[b].error_rate = (float) (sm[i].p_i2cSwitch[s].p_bmp[b].error_num * SEC_IN_MS) / (time_interval);
            sm[i].p_i2cSwitch[s].p_bmp[b].error_num = 0;
        }
        
        // Decrements flag status if sensor has not given considerable errors recently
        if(sm[i].p_i2cSwitch[s].p_bmp[b].flag != SENSOR_OK && (now - sm[i].p_i2cSwitch[s].p_bmp[b].current_error > GRACE_PERIOD)){
            sm[i].p_i2cSwitch[s].p_bmp[b].flag += 1;
            sm[i].p_i2cSwitch[s].p_bmp[b].current_error = now;
            // printf("Sensor has been pardoned.");
        }

        if (sm[i].p_i2cSwitch[s].p_bmp[b].error_rate > ERROR_THRESHOLD){
            switch (sm[i].p_i2cSwitch[s].p_bmp[b].flag){
                case SENSOR_OK:
                    sm[i].p_i2cSwitch[s].p_bmp[b].flag = SENSOR_FLAGGED;
                    sm[i].p_i2cSwitch[s].p_bmp[b].error_rate = 0;
                    // printf("Sensor has been flagged. Address - 0x%x, PCA - %d, SM - %d\n", sm[i].p_i2cSwitch[s].p_bmp[b].address, s, i);
                    break;
                case SENSOR_FLAGGED:
                    sm[i].p_i2cSwitch[s].p_bmp[b].flag = SENSOR_WARNED;
                    sm[i].p_i2cSwitch[s].p_bmp[b].error_rate = 0;
                    // printf("Sensor has been warned. 0x%x, PCA - %d, SM - %d\n", sm[i].p_i2cSwitch[s].p_bmp[b].address, s, i);
                    break;
                case SENSOR_WARNED:
                    sm[i].p_i2cSwitch[s].p_bmp[b].active = false;
                    // printf("Sensor has been deactivated. 0x%x, PCA - %d, SM - %d\n", sm[i].p_i2cSwitch[s].p_bmp[b].address, s, i);
                    break;
                default:
                    // panic(), unknown status (?)
                    break;
            }
        }
    }

    return 0;
}

/* TODO */
int magnetic_read(){
    return 0;
}

int line_check(uint sda, uint scl){
    // Line should be HIGH after STOP. Only the slave can keep it low
    if (gpio_is_pulled_down(sda) || gpio_is_pulled_down(scl)){
        return 1;
    }
    return 0;
}

void bmp_get_pressure(StateMachine *sm){

    for (size_t i = 0; i < MAX_SWITCH_PER_SM; i++){
        //Blacklist sm's with inactive switches
        uint blacklist = blacklist_i2c_switch(sm, i);
        
        //Skip if all switches do not have sensors
        if (blacklist == MAX_BMP_MASK){
            continue;
        }
        
        for (size_t j = 0; j < MAX_BMP_PER_SWITCH; j++){
            uint32_t acks_switch[1][5] = {0};
            uint32_t buffer_rx[5][5] = {0};
            uint32_t buffer_tx[1] = {BMP_PRESS_REG_LSB+1};

            //To open the channel
            Message channel;
            //To get a pressure
            Message press;
            
            message_constructor(&channel, sm[0].p_i2cSwitch[i].address, &sm[0].p_i2cSwitch[i].p_bmp[j].channel, acks_switch, 1);
            message_constructor(&press, sm[0].p_i2cSwitch[i].p_bmp[j].address, buffer_tx, buffer_rx, 5);

            //Helper variables, aid in identifying which sensor it is
            press.switch_i = i;
            press.bmp_j = j;
            
            channel_open(sm, &channel, blacklist);
            
            //Skips data processing if all BMP's in a given switch are not active
            if(bmp_read(sm, &press) == 1){
                continue;
            }
            
            //Data processing
            multicore_fifo_push_blocking_inline(sm->activeSm);

            for (size_t k = 0; k < sm->activeSm; k++){
                sm[k].p_i2cSwitch[i].p_bmp[j].temp_raw = (buffer_rx[4][k] << 16) | (buffer_rx[3][k] << 8) | 0x00;
                sm[k].p_i2cSwitch[i].p_bmp[j].press_raw = (buffer_rx[1][k] << 16) | (buffer_rx[0][k] << 8) | 0x00;

                multicore_fifo_push_blocking_inline((uint32_t) &sm[k].p_i2cSwitch[i].p_bmp[j]);
            }
        }

        if (sm[0].p_i2cSwitch[i].magnetic_enc){
            magnetic_read();
        }
        
    }
}


float bmp_compensate_temperature(struct BmpSensor *bmp){

    // Temperature compensation is done in floating point. Optimization can be done
    // if all variables are passed to uint, including calibration ones
    float partial_data1;
    float partial_data2;

    partial_data1 = (float) (bmp->temp_raw - bmp->calib_val[0]);
    partial_data2 = (float) (partial_data1 * bmp->calib_val[1]);

    bmp->comp_temp[0] = partial_data2 + (partial_data1 * partial_data1) * bmp->calib_val[2];

    return bmp->comp_temp[0];
}


float bmp_compensate_pressure(struct BmpSensor *bmp){
    
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

    //Pressure compensation automatically calls temperature compensation
    bmp_compensate_temperature(bmp);

    if ((((bmp->comp_temp[0] < MIN_BMP_TEMP) || (bmp->comp_temp[0] > MAX_BMP_TEMP)) && (bmp->comp_temp[2] != 0))){
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

void bmp_calib_file_helper(StateMachine *sm){

    for (size_t i = 0; i < sm[0].activeSm; i++){
    char buffer_outter[64];
    snprintf(buffer_outter, sizeof(buffer_outter), "SM Index - %d, SM Switches - %d\n", sm[i].index, sm[i].activeSwitch);
    uart_puts(UART_ID, buffer_outter);

        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            if (!(sm[i].p_i2cSwitch[j].status)){
                continue;
            }
            char buffer_out[64];
                snprintf(buffer_out, sizeof(buffer_out), "PCA Address - 0x%x, Activity - %s\n", 
                        sm[i].p_i2cSwitch[j].address, sm[i].p_i2cSwitch[j].status ? "True" : "False");
                uart_puts(UART_ID, buffer_out);

            for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
                if (!(sm[i].p_i2cSwitch[j].p_bmp[k].active)){
                continue;
            }
                for (size_t l = 0; l < 14; l++){
                    char buffer_out[64];
                    snprintf(buffer_out, sizeof(buffer_out), "BMP Address - 0x%lx, BMP Channel - %ld, BMP Calib - %.3e\n", 
                        sm[i].p_i2cSwitch[j].p_bmp[k].address, sm[i].p_i2cSwitch[j].p_bmp[k].channel, sm[i].p_i2cSwitch[j].p_bmp[k].calib_val[l]);
                    uart_puts(UART_ID, buffer_out);
                }
            }
        }
    }
}

void bmp_press_file_helper(StateMachine *sm){

    for (size_t i = 0; i < MAX_SM; i++){
        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
                if (!(sm[i].p_i2cSwitch[j].p_bmp[k].active)){    
                    continue;
                }
                char buffer_in[64];
                snprintf(buffer_in, sizeof(buffer_in), "%u,", (uint) sm[i].p_i2cSwitch[j].p_bmp[k].comp_press[0]);
                uart_puts(UART_ID, buffer_in);
            }
        }
    }

    //Newline inserted after all bmp's are written
    uart_puts(UART_ID, "\n");
}


uint32_t write_i2c(StateMachine *sm, Message *msg, uint blacklist){

    //Send the number of bytes to write, shifted due to MSB first
    send_msgs(pio_sm_put, sm, msg->data_len << 24, blacklist);

    //Send address
    send_msgs(pio_sm_put_blocking, sm, msg->addr << 25, blacklist);

    for (size_t i = 0; i < msg->data_len; i++){
        //Load data first, read data later
        send_msgs(pio_sm_put, sm, msg->txbuff[i] << 24, blacklist); 
        receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, 0, blacklist);
    }

    //Stop condition, full TX FIFO
    send_msgs(pio_sm_put, sm, FILL_TXF, blacklist);
    receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, 0, blacklist);

    for (size_t i = 0; i < sm[0].activeSm; i++){
        if (msg->rxbuff[0][i] > 0 && !(blacklist & (1u << i))){
            //Get timestamp if error was detected
            msg->rxbuff[0][i] = to_ms_since_boot(get_absolute_time());
        }

        //Reinitialize state machine if in a bad state
        if (line_check(sm[i].sda, sm[i].scl) == 1){
            sm_soft_reboot(&sm[i]);
        }
    }

    return 0;
}

uint32_t read_i2c(StateMachine *sm, Message *msg, uint32_t blacklist){

    uint32_t rx_len = msg->data_len;
    msg->data_len = 1;

    //Write to address the register of interest
    write_i2c(sm, msg, blacklist);
    msg->data_len = rx_len;

    for (size_t i = 0; i < sm[0].activeSm; i++){
        msg->txbuff[i] = 0;
        if (msg->rxbuff[0][i] > 0 && !(blacklist & (1u << i))){

            //Blacklist all sensors whose writing was not acknowledged
            blacklist |= (1 << i);

            //Use msg.tx_buffer to store the errors
            msg->txbuff[i] = to_ms_since_boot(get_absolute_time());
        }
    }
    
    //Data write with no length
    send_msgs(pio_sm_put, sm, 0x00, blacklist);

    //Address shifted + '1' read byte
    send_msgs(pio_sm_put_blocking, sm, ((msg->addr << 1) + 1) << 24, blacklist);

    //Jump directly to read sequence
    send_msgs(pio_sm_put, sm, 0x00, blacklist);

    //Receive the first acknowledges
    receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, 0, blacklist);

    size_t counter = 0;
    for (counter = 0; counter < msg->data_len-1u; counter++){
        //Keep reading and get messages
        send_msgs(pio_sm_put, sm, 0x00, blacklist); 
        receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, counter, blacklist);
    }

    //Stop condition
    send_msgs(pio_sm_put, sm, FILL_TXF, blacklist);

    //Receive last messages
    receive_msgs(pio_sm_get_blocking, sm, msg->rxbuff, counter, blacklist);
    
    for (size_t i = 0; i < sm[0].activeSm; i++){
        if (line_check(sm[i].sda, sm[i].scl) == 1){
            sm_soft_reboot(&sm[i]);
        }
    }
    
    return 0;
}

int send_msgs(void (*put)(PIO, uint, uint32_t), StateMachine *sm, uint32_t data, uint blacklist){
    for (size_t i = 0; i < sm[0].activeSm; i++){
        if (blacklist & (1u << i)){
            //Ignore blacklisted state machines
            continue;
        }
        
        (*put)(sm[i].pio, sm[i].index, data);
    }

    return 0;
}


int receive_msgs(uint32_t (*get)(PIO, uint), StateMachine *sm, uint32_t (*buffer)[5], size_t index, uint blacklist){
    for (size_t i = 0; i < sm[0].activeSm; i++){
        if (blacklist & (1u << i)){
            if (buffer[index][i] == 0){
                buffer[index][i] = 1;
            }
            //Ignore blacklisted state machines and puts them in a NACK state
            continue;
        }
        
        buffer[index][i] = (*get)(sm[i].pio, sm[i].index);
    }

    return 0;
}


int blacklist_i2c_switch(StateMachine *sm, size_t i){
    uint blacklist = 0;

    //For switch use only
    for (size_t k = 0; k < sm->activeSm; k++){
        if (!(sm[k].p_i2cSwitch[i].status)){
           blacklist |= (1 << k);
        }
    }

    return blacklist;
}