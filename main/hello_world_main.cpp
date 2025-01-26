/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <iostream>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "bmp280_esp32_api.hpp"

#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_21


extern "C" void app_main(void)
{

    i2c_master_bus_config_t i2c_mst_config;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT,
    i2c_mst_config.i2c_port = 0;
    i2c_mst_config.scl_io_num = SCL_PIN;
    i2c_mst_config.sda_io_num = SDA_PIN;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.intr_priority = 0;
    i2c_mst_config.flags.enable_internal_pullup = true;
    
    i2c_master_bus_handle_t bus_handle;
    
    i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    BMP280::Bmp280 baro(bus_handle, 
                        0x76, 
                        100000, 
                        16, 
                        2, 
                        0.5, 
                        16, 
                        'n');
                        
    BMP280::measurements_t bmp280_meas;
    for (;;) {
        bmp280_meas = baro.read();
        std::cout << "PRESSURE\t=\t" << bmp280_meas.P << std::endl;
        std::cout << "TEMPERATURE\t=\t" << bmp280_meas.T << std::endl;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
