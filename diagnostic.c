
for (size_t i = 0; i < MAX_SM; i++){
    char buffer_out[32];
    snprintf(buffer_out, sizeof(buffer_out), "SM Index - %d\n", sm_all[i].index);
    uart_puts(UART_ID, buffer_out);
    for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
        char buffer_out[64];
        snprintf(buffer_out, sizeof(buffer_out), "PCA Address - 0x%x, Activity - %s\n", 
                 sm_all[i].pca_ptr[j].address, sm_all[i].pca_ptr[j].status ? "true" : "false");
        uart_puts(UART_ID, buffer_out);
        for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
            char buffer_out[64];
            snprintf(buffer_out, sizeof(buffer_out), "BMP Address - 0x%x, BMP Channel - %d\n", 
                     sm_all[i].pca_ptr[j].bmp[k].address, sm_all[i].pca_ptr[j].bmp[k].channel);
            uart_puts(UART_ID, buffer_out);
        }
    }
}

for (size_t i = 0; i < MAX_SM; i++){
        char buffer_out[64];
        snprintf(buffer_out, sizeof(buffer_out), "SM Index - %d\nSM Offset - %d\nSM Bus - %u\nSM Clock - %u\n", 
        sm_all[i].index, sm_all[i].offset, sm_all[i].sda, sm_all[i].scl);
        uart_puts(UART_ID, buffer_out);
}

for (size_t i = 0; i < 32; i++){
        pio_sm_put(pio0, 0, 0x01 << 24);
        pio_sm_put(pio0, 1, 0x01 << 24);

        pio_sm_put_blocking(pio0, 0, PCA_ADDRESS_BOTTOM << 25);
        pio_sm_put_blocking(pio0, 1, PCA_ADDRESS_BOTTOM << 25);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, CHANNEL_3 << 24);
        pio_sm_put(pio0, 1, CHANNEL_3 << 24);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, FILL_TXF);
        pio_sm_put(pio0, 1, FILL_TXF);

        pio_sm_put(pio0, 0, 0x02 << 24);
        pio_sm_put(pio0, 1, 0x02 << 24);

        pio_sm_put(pio0, 0, BMP_ADDRESS_PRIM << 25);
        pio_sm_put(pio0, 1, BMP_ADDRESS_PRIM << 25);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put_blocking(pio0, 0, BMP_PWR_CTRL << 24);
        pio_sm_put_blocking(pio0, 1, BMP_PWR_CTRL << 24);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, 0x33 << 24);
        pio_sm_put(pio0, 1, 0x33 << 24);

        pio_sm_get_blocking(pio0, 0);
        pio_sm_get_blocking(pio0, 1);

        pio_sm_put(pio0, 0, FILL_TXF);
        pio_sm_put(pio0, 1, FILL_TXF);

        sleep_us(10);
    }

for (size_t i = 0; i < MAX_SM; i++){
    char buffer_outter[64];
    snprintf(buffer_outter, sizeof(buffer_outter), "SM Index - %d, SM Switches - %d\n", sm_all[i].index, sm_all[i].active_switch);
    uart_puts(UART_ID, buffer_outter);

        for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
            char buffer_out[64];
            snprintf(buffer_out, sizeof(buffer_out), "PCA Address - 0x%x, Activity - %s\n", 
                    sm_all[i].pca_ptr[j].address, sm_all[i].pca_ptr[j].status ? "true" : "false");
            uart_puts(UART_ID, buffer_out);
        }
    }

for (size_t i = 0; i < MAX_SM; i++){
    char buffer_outter[64];
    snprintf(buffer_outter, sizeof(buffer_outter), "SM Index - %d, SM Switches - %d\n", sm_all[i].index, sm_all[i].active_switch);
    uart_puts(UART_ID, buffer_outter);

    for (size_t j = 0; j < MAX_SWITCH_PER_SM; j++){
        char buffer_out[64];
            snprintf(buffer_out, sizeof(buffer_out), "PCA Address - 0x%x, Activity - %s\n", 
                    sm_all[i].pca_ptr[j].address, sm_all[i].pca_ptr[j].status ? "True" : "False");
            uart_puts(UART_ID, buffer_out);
        for (size_t k = 0; k < MAX_BMP_PER_SWITCH; k++){
            char buffer_out[64];
            snprintf(buffer_out, sizeof(buffer_out), "BMP Address - 0x%x, BMP Channel - %d, BMP Status - %s\n", 
                     sm_all[i].pca_ptr[j].bmp[k].address, sm_all[i].pca_ptr[j].bmp[k].channel, sm_all[i].pca_ptr[j].bmp[k].active ? "True" : "False");
            uart_puts(UART_ID, buffer_out);
        }
    }
}

