#ifndef _SPIH_
#define _SPIH_

#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU_SPI_CS_PIN        2       // Chip select pin
#define BMP_SPI_CS_PIN        1
#define SPI_MISO_PIN          8
#define SPI_MOSI_PIN          9
#define SPI_CLK_PIN           7
#define SPI_HOST              SPI2_HOST
#define SPI_CLOCK_SPEED       1000000 // SPI frequency: 1 MHz
#define SPI_DMA_CHANNEL       SPI_DMA_CH_AUTO

// SPI configuration variables
static spi_device_handle_t spi_handle_mpu; // Handle cho thiết bị MPU
static spi_device_handle_t spi_handle_bmp; // Handle cho thiết bị BMP

static esp_err_t spi_master_init() {
    esp_err_t ret;

    // cấu hình SPI bus 
    spi_bus_config_t bus_cfg = {
        .miso_io_num = SPI_MISO_PIN,     // MOSI pin
        .mosi_io_num = SPI_MOSI_PIN,     // MOSI pin
        .sclk_io_num = SPI_CLK_PIN,      // SCK pin
        .quadwp_io_num = -1,             // Not used
        .quadhd_io_num = -1              // Not used
        // .max_transfer_sz = 4096          // thêm dòng này (chưa hiểu để làm gì)
    };
    ret = spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CHANNEL);
    if (ret != ESP_OK) {
        return ret;
    }

    // cấu hình thiết bị SPI MPU
    spi_device_interface_config_t dev_cfg1 = {
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .mode = 0,                // SPI mode 0 (CPOL = 0, CPHA = 0)
        .spics_io_num = MPU_SPI_CS_PIN,  // CS pin
        .queue_size = 7,          // Queue size for transactions
        .pre_cb = NULL            // No pre callback
        // .flags = SPI_DEVICE_HALFDUPLEX      // Giao tiếp một chiều (code thêm vào, chắc tương đương cái trên)
    };
    ret = spi_bus_add_device(SPI_HOST, &dev_cfg1, &spi_handle_mpu);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Cấu hình thiết bị SPI BMP
    spi_device_interface_config_t dev_cfg2 = {
        .clock_speed_hz = SPI_CLOCK_SPEED,    // Tốc độ xung SPI
        .mode = 0,                            // Chế độ SPI 0 (CPOL = 0, CPHA = 0)
        .spics_io_num = BMP_SPI_CS_PIN,       // Pin CS cho BMP
        .queue_size = 7,                      // Kích thước hàng đợi giao dịch
        .pre_cb = NULL                        // Không có callback trước giao dịch
    };
    ret = spi_bus_add_device(SPI_HOST, &dev_cfg2, &spi_handle_bmp);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to add SPI device 2: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("SPI", "SPI initialized successfully with MPU and BMP devices");
    return ESP_OK;
}


static void spi_deinit() {
    spi_bus_remove_device(spi_handle_mpu);
    spi_bus_remove_device(spi_handle_bmp);
    spi_bus_free(SPI_HOST);
}

// Hàm ghi 1 byte vào thiết bị SPI
static esp_err_t spi_write(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t tx_data[2] = { reg_addr, data };
    spi_transaction_t t = {
        .length = 8 * 2,           // Transfer 2 bytes
        .tx_buffer = tx_data,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to write byte: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Hàm đọc 1 byte từ thiết bị SPI
static uint8_t spi_read(spi_device_handle_t spi_handle, uint8_t reg_addr) {
    uint8_t tx_data[2] = { reg_addr | 0x80, 0x00 }; // Read command (MSB = 1)
    uint8_t rx_data[2];

    spi_transaction_t t = {
        .length = 8 * 2,           // Transfer 2 bytes
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    // if (ret == ESP_OK) {
    //     *data = rx_data[1];  // Receive the actual data
    // }
    // return ret;
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Failed to read byte: %s", esp_err_to_name(ret));
        return 0;
    }
    return rx_data[1];
}

// Hàm đọc 2 byte từ thiết bị SPI
static uint16_t spi_read_2bytes(spi_device_handle_t spi_handle, uint8_t data_addr) {
    uint8_t tx_data[3]; // Data to send (1 byte address + 2 dummy bytes for read)
    uint8_t rx_data[3]; // Data received

    tx_data[0] = data_addr | 0x80; // Set MSB to 1 for read operation
    tx_data[1] = 0x00;            // Dummy byte
    tx_data[2] = 0x00;            // Dummy byte

    spi_transaction_t trans = {
        .length = 8 * 3,           // 3 bytes (24 bits)
        .tx_buffer = tx_data,      // Transmit buffer
        .rx_buffer = rx_data,      // Receive buffer
    };

    // Transmit and receive data
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "Error reading from SPI device: %s", esp_err_to_name(ret));
        return 0;
    }

    // Combine received bytes (rx_data[1] and rx_data[2])
    uint16_t result = (rx_data[1] << 8) | rx_data[2];
    return result;
}

#endif