
#pragma once

#include <stdint.h>
#include "esp_system.h"

#define BMP280_OSRS_P x16
#define BMP280_OSRS_T x2
#define BMP280_MODE normal_mode

namespace BMP280
{


enum class tsb_t{
    t_sb_0_5,
    t_sb_62_5,
    t_sb_125,
    t_sb_250,
    t_sb_500,
    t_sb_1000,
    t_sb_2000,
    t_sb_4000
};

enum class filter_t{
    filter_off,
    filter_2,
    filter_4,
    filter_8,
    filter_16
};

enum class osrs_t{
    skipped,
    x1,
    x2,
    x4,
    x8,
    x16
};

enum class mode_t{
    sleep_mode,
    forced_mode,
    normal_mode = 3
};

typedef struct {
    double T;
    double P;
} measurements_t;

class Bmp280{
    private:
        uint16_t    T1;
        int16_t     T2;
        int16_t     T3;
        uint16_t    P1;
        int16_t     P2;
        int16_t     P3;
        int16_t     P4;
        int16_t     P5;
        int16_t     P6;
        int16_t     P7;
        int16_t     P8;
        int16_t     P9;
        i2c_master_dev_handle_t handle;

        int32_t read_raw_pressure();

        int32_t read_raw_temperature();

        esp_err_t read_compensation_parameters();

    public:
        esp_err_t set_pressure_oversampling(osrs_t osrs_p);

        esp_err_t set_temperature_oversampling(osrs_t osrs_t);

        esp_err_t set_mode(mode_t mode);

        esp_err_t set_t_sb(tsb_t t_sb);

        esp_err_t set_filter(filter_t filter);

        measurements_t read();

        Bmp280( i2c_master_bus_handle_t bus_handle, 
                uint16_t bmp280_adress, 
                uint32_t scl_speed_hz, 
                osrs_t pressure_oversampling, 
                osrs_t temperature_oversampling, 
                tsb_t t_sb, 
                filter_t filter,
                mode_t mode);

};




} // end bmp280