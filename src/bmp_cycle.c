#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"

#include "bmp_cycle.h"
#include "bmp_cycle_lib.c"
#include "bmp_config.c"

#include "bmp_cycle.pio.h"

void core_entry(){

    while (true){
        //Wait while there is no data to send
        while (!multicore_fifo_rvalid()){
            tight_loop_contents();
        }

        struct BmpSensor *bmp[MAX_SM];
        
        // Receives a pointer to a sensor. A 32-bit processor has 32-bit memory addresses. Functions can
        // also be passed the same way using func_ptr_t
        uint32_t n = multicore_fifo_pop_blocking();

        //To prevent overflows
        if (n > MAX_SM){
            n = MAX_SM;
        }
        
        for (size_t i = 0; i < n; i++){
            //Gets the address of all bmp's structs sent. Cannot exceed num of sm's
            while (!multicore_fifo_rvalid()){
                tight_loop_contents();
            }

            bmp[i] = (struct BmpSensor*) multicore_fifo_pop_blocking();
            
        }
        
        // bmp_compensate_pressure can be substituted by a generic function, given that said function was passed
        // as an address
        for (size_t i = 0; i < n; i++){
            bmp_compensate_pressure(bmp[i]);
        }
    }

}

int main(){
    stdio_init_all();
    uart_begin();
    sleep_ms(3000);

    multicore_reset_core1();
    multicore_launch_core1(core_entry);
    gpio_put(UART_ENABLE, true);

    // All data structures needed. bmp's are linked to i2c_switches linked to state machines
    // Data has to be static if it is going to be shared between multiple cores. 
    static struct BmpSensor bmp_all[MAX_SM][MAX_SWITCH_PER_SM][MAX_BMP_PER_SWITCH];
    static struct I2cSwitch pca_all[MAX_SM][MAX_SWITCH_PER_SM];
    static StateMachine sm_all[MAX_SM];

    uint active_switch_1;
    uint active_switch_2;
    uint active_sm;

    switch (CONFIG_TYPE){
    case 0:
        active_switch_1 = 0x7ff; // '11111 11111 11111'
        active_switch_2 = 0x7ff; 
        active_sm = 5;
        break;
    case 1:
        break;
    case 2:
        active_switch_1 = 0x801; // '00010 00000 00001'
        active_switch_2 = 0x21;  // '000 000 000 100 001' 
        active_sm = 2;
        break;
    default:
        break;
    }
    
    // Construct functions to help build the data structures. The masks dictate the
    // layout of the active sm's, switches and bmp's
    construct_hand(sm_all, active_sm, active_switch_2, pca_all, active_switch_1, bmp_all);
    linker(sm_all, pca_all, bmp_all, active_sm);
    hand_init(sm_all);

    // Initializes all sensors, only those successfully activated will perform operations 
    bmp_init(sm_all);
    bmp_get_calib(sm_all);

    absolute_time_t now = get_absolute_time();

    // Measurement cycle. Replace with while(true) for infinite measurements
    while ((to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(now)) < 10*SEC_IN_MS){
        bmp_get_pressure(sm_all);
        bmp_press_file_helper(sm_all);
    }

    gpio_put(UART_ENABLE, false);
    
    while (true){
        tight_loop_contents();
    }

}


