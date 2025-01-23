#pragma once

#include <stdint.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "bmp280_esp32_api.hpp"

using namespace BMP280;

Bmp280::Bmp280( i2c_master_bus_handle_t bus_handle, 
                uint16_t bmp280_adress, 
                uint32_t scl_speed_hz, 
                osrs_t pressure_oversampling, 
                osrs_t temperature_oversampling, 
                tsb_t t_sb, 
                filter_t filter,
                mode_t mode){
    
    i2c_device_config_t bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmp280_adress,
        .scl_speed_hz = scl_speed_hz,
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp280_cfg, &handle));
    set_mode(mode);
    set_pressure_oversampling(pressure_oversampling);
    set_temperature_oversampling(temperature_oversampling);
    set_t_sb(t_sb);
    set_filter(filter);
    read_compensation_parameters();
    return;
}

esp_err_t Bmp280::set_pressure_oversampling(osrs_t osrs_p){
    uint8_t ctrl_means;
    uint8_t ctrl_means_adr = 0xF4;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &ctrl_means_adr, 1, &ctrl_means, 1, -1);
    if (error_code){
        // printf("Programm has faild in function ***bmp280_set_pressure_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    ctrl_means &= ~(7 << 2);
    ctrl_means |= ((uint8_t)osrs_p << 2);
    uint8_t ctrl_means_write[2] = {ctrl_means_adr, ctrl_means};
    error_code = i2c_master_transmit(handle, ctrl_means_write, 2, -1); 
    if (error_code){
        // printf("Programm has faild in function ***bmp280_set_pressure_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
}      

esp_err_t Bmp280::set_temperature_oversampling(osrs_t osrs_t){
    uint8_t ctrl_means;
    uint8_t ctrl_means_adr = 0xF4;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &ctrl_means_adr, 1, &ctrl_means, 1, -1);
    if (error_code){
        // printf("Programm has faild in function ***bmp280_set_temperature_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    ctrl_means &= ~(7 << 5);
    ctrl_means |= ((uint8_t)osrs_t << 5);
    uint8_t ctrl_means_write[2] = {ctrl_means_adr, ctrl_means};
    error_code = i2c_master_transmit(handle, ctrl_means_write, 2, -1); 
    if (error_code){
        // printf("Programm has faild in function ***bmp280_set_temperature_oversampling()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
} 

esp_err_t Bmp280::set_mode(mode_t mode){
    uint8_t ctrl_means;
    uint8_t ctrl_means_adr = 0xF4;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &ctrl_means_adr, 1, &ctrl_means, 1, -1);
    if (error_code){
        // printf("Programm has faild in function ***Bmp280::set_mode()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    ctrl_means &= ~3;
    ctrl_means |= (uint8_t)mode;
    uint8_t ctrl_means_write[2] = {ctrl_means_adr, ctrl_means};
    error_code = i2c_master_transmit(handle, ctrl_means_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***Bmp280::set_mode()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
} 

esp_err_t Bmp280::set_t_sb(tsb_t t_sb){
    uint8_t config;
    uint8_t config_adr = 0xF5;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &config_adr, 1, &config, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_t_sb()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    config &= ~(7 << 5);
    config |= ((uint8_t)t_sb << 5);
    uint8_t config_write[2] = {config_adr, config};
    error_code = i2c_master_transmit(handle, config_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_t_sb()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
}  

esp_err_t Bmp280::set_filter(filter_t filter){
    uint8_t config;
    uint8_t config_adr = 0xF5;
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &config_adr, 1, &config, 1, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_filter()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    config &= ~(7 << 2);
    config |= ((uint8_t)filter << 2);
    uint8_t config_write[2] = {config_adr, config};
    error_code = i2c_master_transmit(handle, config_write, 2, -1); 
    if (error_code){
        printf("Programm has faild in function ***bmp280_set_filter()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    return ESP_OK;
} 

esp_err_t Bmp280::read_compensation_parameters(){ // cp - compensation_parameters
    uint8_t parameters_adr = 0x88;
    uint8_t buf[24];
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &parameters_adr, 1, buf, 24, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_read_compensation_parameters()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    T1 = (uint16_t)buf[0 ] | (uint16_t)buf[1 ] << 8;
    T2 = (int16_t )buf[2 ] | (int16_t )buf[3 ] << 8;
    T3 = (int16_t )buf[4 ] | (int16_t )buf[5 ] << 8;
    P1 = (uint16_t)buf[6 ] | (uint16_t)buf[7 ] << 8;
    P2 = (int16_t )buf[8 ] | (int16_t )buf[9 ] << 8;
    P3 = (int16_t )buf[10] | (int16_t )buf[11] << 8;
    P4 = (int16_t )buf[12] | (int16_t )buf[13] << 8;
    P5 = (int16_t )buf[14] | (int16_t )buf[15] << 8;
    P6 = (int16_t )buf[16] | (int16_t )buf[17] << 8;
    P7 = (int16_t )buf[18] | (int16_t )buf[19] << 8;
    P8 = (int16_t )buf[20] | (int16_t )buf[21] << 8;
    P9 = (int16_t )buf[22] | (int16_t )buf[23] << 8;
    return ESP_OK;
}

int32_t Bmp280::read_raw_pressure(){
    uint8_t pressure_adr = 0xF7;
    uint8_t buf[3];
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &pressure_adr, 1, buf, 3, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_read_raw_pressure()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    int32_t raw_pressure   =  (int32_t)buf[0] << 12 | (int32_t)buf[1] << 4 | (int32_t)buf[2] >> 4;
    return raw_pressure;    
}

int32_t Bmp280::read_raw_temperature(){
    uint8_t temperature_adr = 0xFA;
    uint8_t buf[3];
    esp_err_t error_code;
    error_code = i2c_master_transmit_receive(handle, &temperature_adr, 1, buf, 3, -1);
    if (error_code){
        printf("Programm has faild in function ***bmp280_read_raw_temperature()*** with error code %s", esp_err_to_name(error_code));
        return error_code;
    }
    int32_t raw_temperature = (int32_t)buf[0] << 12 | (int32_t)buf[1] << 4 | (int32_t)buf[2] >> 4;
    return raw_temperature;
}

measurements_t Bmp280::read(){
    int32_t T_raw = read_raw_temperature();
    int32_t P_raw = read_raw_pressure();
    double var1, var2, var3, var4;
    measurements_t meas;
    
    int32_t t_fine;
    var1 = ((double)T_raw/16384.0 - (double)T1/1024.0) * (double)T2;
    var2 = ((((double)T_raw)/131072.0 - ((double)T1)/8192.0) * (((double)T_raw)/131072.0 - ((double)T1)/8192.0)) * ((double)T3);
    t_fine = (int32_t)(var1 + var2);
    meas.T = (var1 + var2) / 5120.0;

    var3 = (double)t_fine/2.0 - 64000.0;
    var4 = var3 * var3 * (double)P6/32768.0;
    var4 = var4 + var3 * (double)P5*2.0;
    var4 = var4/4.0 + (double)P4*65536.0;
    var3 = ((double)P3*var3*var3/524288.0 + (double)P2*var3) / 524288.0;
    var3 = (1.0 + var3/32768.0)*(double)P1;
    if (var3 == 0.0){
        meas.P = 0.0;
        return meas;
    }
    meas.P = 1048576.0 - (double)P_raw;
    meas.P = (meas.P - var4/4096.0) * 6250.0/var3;
    var3 = (double)P9 * meas.P * meas.P / 2147483648.0;
    var4 = meas.P * (double)P8 / 32768.0;
    meas.P = meas.P + (var3 + var4 + (double)P7) / 16.0;
    return meas;
}



