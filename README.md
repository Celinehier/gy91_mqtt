# GY-91 với ESP32-S3 qua giao thức SPI

## Mô tả dự án
Dự án này sử dụng **ESP32-S3** để giao tiếp với cảm biến **GY-91** thông qua giao thức **SPI**. Cảm biến GY-91 tích hợp hai cảm biến quan trọng: **MPU9250** (IMU - gia tốc kế, con quay hồi chuyển, từ kế) và **BMP280** (đo áp suất và nhiệt độ).

Mục tiêu của dự án:
- Kết nối GY-91 với ESP32-S3 qua SPI.
- Đọc dữ liệu từ MPU9250 (gia tốc, con quay, từ trường).
- Đọc dữ liệu từ BMP280 (nhiệt độ, áp suất, độ cao).
- Hiển thị dữ liệu lên terminal và truyền qua MQTT.

---

## Phần cứng sử dụng
### 1. **GY-91**
- **MPU9250**: 
  - Giao tiếp: SPI/I2C.
  - Cung cấp dữ liệu gia tốc, con quay hồi chuyển và từ kế.
- **BMP280**:
  - Giao tiếp: SPI/I2C.
  - Đo áp suất khí quyển, nhiệt độ, tính toán độ cao.

### 2. **ESP32-S3**
- Bộ vi điều khiển hỗ trợ Wi-Fi/Bluetooth, mạnh mẽ để xử lý dữ liệu cảm biến.
- Giao tiếp: SPI (cùng với các giao tiếp khác như I2C, UART).

### 3. **Kết nối**
| ESP32-S3        | GY-91 Pin        | Mô tả                  |
|------------------|------------------|------------------------|
| `3.3V`          | `VCC`            | Nguồn cấp cho GY-91    |
| `GND`           | `GND`            | GND                    |
| `IO`          | `SCL`            | SPI Clock (SCK)        |
| `IO`          | `SDA`            | SPI Data (MOSI)        |
| `IO`          | `SDO`            | SPI Data (MISO)        |
| `IO`           | `CS`             | SPI Chip Select        |

---

## Phần mềm sử dụng
### **ESP-IDF**
- ESP-IDF là framework chính thức từ Espressif để lập trình ESP32.
- Đảm bảo bạn đã cài đặt ESP-IDF trên máy. Xem hướng dẫn tại: [ESP-IDF Setup Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).


## Cách sử dụng
### 1. Kết nối phần cứng
- Đảm bảo kết nối đúng các chân như bảng ở trên.
- Kiểm tra nguồn cấp (3.3V).

### 2. Cấu hình ESP-IDF
- Cấu hình giao thức SPI và chân GPIO trong file `sdkconfig`:
  ```plaintext
  CONFIG_SPI_MOSI_GPIO=9
  CONFIG_SPI_MISO_GPIO=8
  CONFIG_SPI_SCLK_GPIO=7
  CONFIG_SPI_CS_GPIO=1/2

### 3. Build và flash

