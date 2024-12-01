#include <stdio.h>
#include "esp_log.h"
#include "mpu9250.h"
#include "bmp280.h"
#include "mqtt.h"


int get_data_gy91(int argc, char **argv){    
    
    int16_t ACCxyz[3];
	int16_t GYRxyz[3];

    // spi_master_init();

    // // Initialize BMP280
    // BMP280_read_id();
    // BMP280_reg_check();

    // // Initialize MPU6500
    // MPU6500();
    // initMPU6500();

    // while(1)
    // {
    //     readAccelData(ACCxyz);
    //     readGyroData(GYRxyz);
    //     bmp280_read();
        
    //     printf("MPU6500:\r\n");
    //     printf("ACC:  \tX: %5.4f  \tY: %5.4f  \tZ: %5.4f\r\n",ACCxyz[0]*getAres(),ACCxyz[1]*getAres(),ACCxyz[2]*getAres());
    //     printf("GYRO: \tX: %7.4f  \tY: %7.4f  \tZ: %7.4f\r\n",GYRxyz[0]*getGres(),GYRxyz[1]*getGres(),GYRxyz[2]*getGres());
    //     printf("Temp: \t%3.1f°C\r\n\r\n",readTempInC());

    //     printf("BMP280:\r\n");
    //     printf("Temp: \t%3.1f°C\r\n", bmp.temperature);
	// 	printf("Pressure:\t%5.4f mbar\r\n", bmp.pressure);
	// 	printf("Altitude:\t%5.3f m\r\n\r\n", bmp.altitude);

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    while(1)
    {
        readAccelData(ACCxyz);
        readGyroData(GYRxyz);
        bmp280_read();
        
        // Tạo JSON string để gửi qua MQTT
        char mqtt_data[256];
        snprintf(
            mqtt_data, sizeof(mqtt_data),
            "{"
            "  \"MPU6500\": {"
            "    \"ACC\": {"
            "      \"X\": %.4f,"
            "      \"Y\": %.4f,"
            "      \"Z\": %.4f"
            "    },"
            "    \"GYRO\": {"
            "      \"X\": %.4f,"
            "      \"Y\": %.4f,"
            "      \"Z\": %.4f"
            "    },"
            "    \"Temp\": %.1f"
            "  },"
            "  \"BMP280\": {"
            "    \"Temp\": %.1f,"
            "    \"Pressure\": %.4f,"
            "    \"Altitude\": %.3f"
            "  }"
            "}",
            ACCxyz[0] * getAres(),
            ACCxyz[1] * getAres(),
            ACCxyz[2] * getAres(),
            GYRxyz[0] * getGres(),
            GYRxyz[1] * getGres(),
            GYRxyz[2] * getGres(),
            readTempInC(),
            bmp.temperature,
            bmp.pressure,
            bmp.altitude
        );
        // Publish dữ liệu qua MQTT
        esp_mqtt_client_publish(client, "sensor/data", mqtt_data, 0, 1, 0);

        printf("MPU6500:\r\n");
        printf("ACC:  \tX: %5.4f  \tY: %5.4f  \tZ: %5.4f\r\n",ACCxyz[0]*getAres(),ACCxyz[1]*getAres(),ACCxyz[2]*getAres());
        printf("GYRO: \tX: %7.4f  \tY: %7.4f  \tZ: %7.4f\r\n",GYRxyz[0]*getGres(),GYRxyz[1]*getGres(),GYRxyz[2]*getGres());
        printf("Temp: \t%3.1f°C\r\n\r\n",readTempInC());

        printf("BMP280:\r\n");
        printf("Temp: \t%3.1f°C\r\n", bmp.temperature);
		printf("Pressure:\t%5.4f mbar\r\n", bmp.pressure);
		printf("Altitude:\t%5.3f m\r\n\r\n", bmp.altitude);

        // Đợi 1 giây
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xTaskCreate(get_data_gy91, "Get GY91", 8192, NULL, 1, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
}

void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void){
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect()); 

    ESP_LOGI(TAG, "Initializing SPI and MQTT...");

    spi_master_init();

    // Initialize BMP280
    BMP280_read_id();
    BMP280_reg_check();

    // Initialize MPU6500
    MPU6500();
    initMPU6500();

    mqtt_app_start();

    // while(1)
    // {
    //     readAccelData(ACCxyz);
    //     readGyroData(GYRxyz);
    //     bmp280_read();
        
    //     printf("MPU6500:\r\n");
    //     printf("ACC:  \tX: %5.4f  \tY: %5.4f  \tZ: %5.4f\r\n",ACCxyz[0]*getAres(),ACCxyz[1]*getAres(),ACCxyz[2]*getAres());
    //     printf("GYRO: \tX: %7.4f  \tY: %7.4f  \tZ: %7.4f\r\n",GYRxyz[0]*getGres(),GYRxyz[1]*getGres(),GYRxyz[2]*getGres());
    //     printf("Temp: \t%3.1f°C\r\n\r\n",readTempInC());

    //     printf("BMP280:\r\n");
    //     printf("Temp: \t%3.1f°C\r\n", bmp.temperature);
	// 	printf("Pressure:\t%5.4f mbar\r\n", bmp.pressure);
	// 	printf("Altitude:\t%5.3f m\r\n\r\n", bmp.altitude);

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // //Start test task
    // xTaskCreate(get_data_gy91, "Get BPM", 8192, NULL, 1, NULL);
}