#ifndef _BMP280_SPI_
#define _BMP280_SPI_

#include <stdio.h>
#include <math.h>
#include "spi.h"

#define QNH                         1020
#define POWER_MODE                  3   // normal mode
#define OSRS_T                      5   // 20 Bit
#define OSRS_P                      5   // 20 Bit ultra high resolution
#define FILTER                      4
#define T_SB                        4   // 500ms

// Register definitions
#define BMP280_REGISTER_DIG_T1            0x88
#define BMP280_REGISTER_DIG_T2            0x8A
#define BMP280_REGISTER_DIG_T3            0x8C
#define BMP280_REGISTER_DIG_P1            0x8E
#define BMP280_REGISTER_DIG_P2            0x90
#define BMP280_REGISTER_DIG_P3            0x92
#define BMP280_REGISTER_DIG_P4            0x94
#define BMP280_REGISTER_DIG_P5            0x96
#define BMP280_REGISTER_DIG_P6            0x98
#define BMP280_REGISTER_DIG_P7            0x9A
#define BMP280_REGISTER_DIG_P8            0x9C
#define BMP280_REGISTER_DIG_P9            0x9E
#define BMP280_REGISTER_CHIPID            0xD0
#define BMP280_REGISTER_VERSION			  0xD1
#define BMP280_REGISTER_SOFTRESET		  0xE0
#define BMP280_REGISTER_CONTROL           0xF4
#define BMP280_REGISTER_CONFIG            0xF5
#define BMP280_REGISTER_STATUS			  0xF3


// Register definitions for BMP280
#define BMP280_REGISTER_TEMPDATA_MSB      0xFA
#define BMP280_REGISTER_TEMPDATA_LSB      0xFB
#define BMP280_REGISTER_TEMPDATA_XLSB     0xFC
#define BMP280_REGISTER_PRESSDATA_MSB     0xF7
#define BMP280_REGISTER_PRESSDATA_LSB     0xF8
#define BMP280_REGISTER_PRESSDATA_XLSB    0xF9

// thử đổi lại
// #define BMP280_REGISTER_TEMPDATA_MSB		0xF7
// #define BMP280_REGISTER_TEMPDATA_LSB		0xF8
// #define BMP280_REGISTER_TEMPDATA_XLSB		0xF9
// #define BMP280_REGISTER_PRESSDATA_MSB		0xFA
// #define BMP280_REGISTER_PRESSDATA_LSB		0xFB
// #define BMP280_REGISTER_PRESSDATA_XLSB		0xFC

// SPI configuration variables
// spi_device_handle_t bmp280_spi_handle;
int BMP280_CONFIG = (T_SB << 5) + (FILTER << 2); // combine bits for config
int CTRL_MEAS = (OSRS_T << 5) + (OSRS_P << 2) + POWER_MODE; // combine bits for ctrl_meas

struct {
    int chip_id;
    int chip_version;
    double temperature;
    double pressure;
    double altitude;
} bmp;

uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

uint16_t read_word_data_unsigned(uint8_t reg_addr) {
    uint16_t result = 0;
    result = spi_read_2bytes(spi_handle_bmp, reg_addr);
    if (result == 0) {
        ESP_LOGE("BMP280", "Failed to read register 0x%02X", reg_addr);
    }
	ESP_LOGI("BMP280", "read register 0x%02X", reg_addr);
	return result;
}

int16_t read_word_data_signed(uint8_t reg_addr){
	int result = 0;
	result = read_word_data_unsigned(reg_addr);
	if (result > 32767){
		result = result - 65536;
	}
	return result;
}

// static esp_err_t spi_read_word_signed(uint8_t reg_addr, uint16_t *data) {
//     uint16_t unsigned_data;
//     esp_err_t ret = spi_read_word_unsigned(reg_addr, &unsigned_data);
//     *data = unsigned_data > 32767 ? unsigned_data - 65536 : unsigned_data;
//     return ret;
// }

void BMP280_read_id() {
    uint8_t chip_id;
    chip_id = spi_read(spi_handle_bmp,BMP280_REGISTER_CHIPID);
    bmp.chip_id = chip_id;
    ESP_LOGI("BMP280", "Chip ID: 0x%02X", bmp.chip_id);
}

// void BMP280_read_id(){
// 	uint16_t result;
// 	// result = read_word_data_unsigned(BMP280_REGISTER_CHIPID);
//     result = spi_read_2bytes(BMP280_REGISTER_CHIPID);
// 	// char*hex_value = convert_to_hex(result);
// 	// bmp.chip_id = result & 0xFF;
// 	bmp.chip_id = result;
// 	bmp.chip_version = result >> 8;
//     ESP_LOGI("BMP280", "Chip ID: 0x%02X", bmp.chip_id);
// }

void BMP280_reg_check() {
    spi_write(spi_handle_bmp,BMP280_REGISTER_SOFTRESET, 0xB6);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    spi_write(spi_handle_bmp,BMP280_REGISTER_CONTROL, CTRL_MEAS);
	vTaskDelay(200 / portTICK_PERIOD_MS);
    spi_write(spi_handle_bmp,BMP280_REGISTER_CONFIG, BMP280_CONFIG);
	vTaskDelay(200 / portTICK_PERIOD_MS);
    
	dig_T1 = read_word_data_unsigned(BMP280_REGISTER_DIG_T1); // read correction settings
	ESP_LOGI("BMP280", "dig_T1: %d", dig_T1);
	// read_word_data_signed(BMP280_REGISTER_DIG_T2, &dig_T2);
	dig_T2 = read_word_data_signed(BMP280_REGISTER_DIG_T2);
	ESP_LOGI("BMP280", "dig_T2: %d", dig_T2);
	dig_T3 = read_word_data_signed(BMP280_REGISTER_DIG_T3);
	ESP_LOGI("BMP280", "dig_T3: %d", dig_T3);
	dig_P1 = read_word_data_unsigned(BMP280_REGISTER_DIG_P1);
	ESP_LOGI("BMP280", "dig_P1: %d", dig_P1);
	dig_P2 = read_word_data_signed(BMP280_REGISTER_DIG_P2);
	ESP_LOGI("BMP280", "dig_P2: %d", dig_P2);
	dig_P3 = read_word_data_signed(BMP280_REGISTER_DIG_P3);
	ESP_LOGI("BMP280", "dig_P3: %d", dig_P3);
	dig_P4 = read_word_data_signed(BMP280_REGISTER_DIG_P4);
	ESP_LOGI("BMP280", "dig_P4: %d", dig_P4);
	dig_P5 = read_word_data_signed(BMP280_REGISTER_DIG_P5);
	ESP_LOGI("BMP280", "dig_P5: %d", dig_P5);
	dig_P6 = read_word_data_signed(BMP280_REGISTER_DIG_P6);
	ESP_LOGI("BMP280", "dig_P6: %d", dig_P6);
	dig_P7 = read_word_data_signed(BMP280_REGISTER_DIG_P7);
	ESP_LOGI("BMP280", "dig_P7: %d", dig_P7);
	dig_P8 = read_word_data_signed(BMP280_REGISTER_DIG_P8);
	ESP_LOGI("BMP280", "dig_P8: %d", dig_P8);
	dig_P9 = read_word_data_signed(BMP280_REGISTER_DIG_P9);
	ESP_LOGI("BMP280", "dig_P9: %d", dig_P9);
}

// void BMP280_read_data() {
//     uint8_t msb, lsb, xlsb;
//     int32_t raw_temp, raw_press;
//     double var1, var2, temperature, t_fine, pressure, altitude;

//     esp_err_t ret = spi_read(BMP280_REGISTER_TEMPDATA_MSB, &msb);
//     if (ret != ESP_OK) {
//     ESP_LOGE("BMP280", "Error reading temperature MSB: %s", esp_err_to_name(ret));
//     }
//     spi_read(BMP280_REGISTER_TEMPDATA_LSB, &lsb);
//     spi_read(BMP280_REGISTER_TEMPDATA_XLSB, &xlsb);
//     raw_temp = (msb << 12) | (lsb << 4) | (xlsb >> 4);

//     spi_read(BMP280_REGISTER_PRESSDATA_MSB, &msb);
//     spi_read(BMP280_REGISTER_PRESSDATA_LSB, &lsb);
//     spi_read(BMP280_REGISTER_PRESSDATA_XLSB, &xlsb);
//     raw_press = (msb << 12) | (lsb << 4) | (xlsb >> 4);

//     var1 = (raw_temp / 16384.0 - dig_T1 / 1024.0) * dig_T2;
//     var2 = ((raw_temp / 131072.0 - dig_T1 / 8192.0) * (raw_temp / 131072.0 - dig_T1 / 8192.0)) * dig_T3;
//     t_fine = var1 + var2; //need for pressure caculation
//     temperature = t_fine / 5120.0;

//     var1 = t_fine / 2.0 - 64000.0;
//     var2 = var1 * var1 * dig_P6 / 32768.0;
//     var2 += var1 * dig_P5 * 2;
//     var2 = (var2 / 4.0) + (dig_P4 * 65536.0);
//     var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
//     var1 = (1.0 + var1 / 32768.0) * dig_P1;

//     pressure = 1048576.0 - raw_press;
//     pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
//     var1 = dig_P9 * pressure * pressure / 2147483648.0;
//     var2 = pressure * dig_P8 / 32768.0;
//     pressure = pressure + (var1 + var2 + dig_P7) / 16.0;

//     altitude = 44330.0 * (1.0 - pow(pressure / (QNH * 100), 0.1903));

//     bmp.temperature = temperature;
//     bmp.pressure = pressure / 100.0;
//     bmp.altitude = altitude / 10;
// }
void bmp280_read(){
	uint8_t raw_temperature_msb, raw_temperature_lsb, raw_temperature_xlsb;
	uint8_t raw_pressure_msb, raw_pressure_lsb, raw_pressure_xlsb;
	uint32_t raw_temperature;
	uint32_t raw_pressure;
	double var1, var2, t_fine, pressure, altitude;
	double temperature;
    
    raw_temperature_msb = spi_read(spi_handle_bmp,BMP280_REGISTER_TEMPDATA_MSB); // read raw temperature msb
	ESP_LOGI("BMP280", "Raw temperature MSB: 0x%02X (%u)", raw_temperature_msb, raw_temperature_msb);
	raw_temperature_lsb = spi_read(spi_handle_bmp,BMP280_REGISTER_TEMPDATA_LSB); // read raw temperature lsb
	ESP_LOGI("BMP280", "Raw temperature LSB: 0x%02X (%u)", raw_temperature_lsb, raw_temperature_lsb);
	raw_temperature_xlsb = spi_read(spi_handle_bmp,BMP280_REGISTER_TEMPDATA_XLSB); // read raw temperature xlsb
	ESP_LOGI("BMP280", "Raw temperature XLSB: 0x%02X (%u)", raw_temperature_xlsb, raw_temperature_xlsb);
	raw_pressure_msb = spi_read(spi_handle_bmp,BMP280_REGISTER_PRESSDATA_MSB); // read raw pressure msb
	ESP_LOGI("BMP280", "Raw pressure MSB: 0x%02X (%u)", raw_pressure_msb, raw_pressure_msb);
	raw_pressure_lsb = spi_read(spi_handle_bmp,BMP280_REGISTER_PRESSDATA_LSB); // read raw pressure lsb
	ESP_LOGI("BMP280", "Raw pressure LSB: 0x%02X (%u)", raw_pressure_lsb, raw_pressure_lsb);
	raw_pressure_xlsb = spi_read(spi_handle_bmp,BMP280_REGISTER_PRESSDATA_XLSB); // read raw pressure xlsb
	ESP_LOGI("BMP280", "Raw pressure XLSB: 0x%02X (%u)", raw_pressure_xlsb, raw_pressure_xlsb);

	raw_temperature=(raw_temperature_msb <<12) | (raw_temperature_lsb<<4) | (raw_temperature_xlsb>>4); // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right
	ESP_LOGI("BMP280", "raw_temperature: %lu (0x%lX)", raw_temperature, raw_temperature);
	raw_pressure=(raw_pressure_msb <<12) | (raw_pressure_lsb <<4) | (raw_pressure_xlsb >>4); // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right
	ESP_LOGI("BMP280", "raw_pressure: %lu (0x%lX)", raw_pressure, raw_pressure);

	var1=(raw_temperature/16384.0-dig_T1/1024.0)*dig_T2; // formula for temperature from datasheet
	var2=(raw_temperature/131072.0-dig_T1/8192.0)*(raw_temperature/131072.0-dig_T1/8192.0)*dig_T3; // formula for temperature from datasheet
	temperature=(var1+var2)/5120.0; // formula for temperature from datasheet
	ESP_LOGI("BMP280", "temperature: %.2f °C", temperature);
	t_fine=(var1+var2); // need for pressure calculation

    var1=t_fine/2.0-64000.0; // formula for pressure from datasheet
	var2=var1*var1*dig_P6/32768.0; // formula for pressure from datasheet
	var2=var2+var1*dig_P5*2; // formula for pressure from datasheet
	var2=var2/4.0+dig_P4*65536.0; // formula for pressure from datasheet
	var1=(dig_P3*var1*var1/524288.0+dig_P2*var1)/524288.0; // formula for pressure from datasheet
	var1=(1.0+var1/32768.0)*dig_P1; // formula for pressure from datasheet
	
	pressure=1048576.0-raw_pressure; // formula for pressure from datasheet
	pressure=(pressure-var2/4096.0)*6250.0/var1; // formula for pressure from datasheet
	var1=dig_P9*pressure*pressure/2147483648.0; // formula for pressure from datasheet
	var2=pressure*dig_P8/32768.0; // formula for pressure from datasheet
	pressure=pressure+(var1+var2+dig_P7)/16.0; // formula for pressure from datasheet

	altitude= 44330.0 * (1.0 - pow(pressure / (QNH*100), (1.0/5.255))); // formula for altitude from airpressure

	bmp.temperature = temperature;
	bmp.pressure = pressure / 100.0;
	bmp.altitude = altitude / 10;
}

#endif