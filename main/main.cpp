#include <cstdio>
#include <cinttypes>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "sensor_dht22.h"

static const char* TAG = "TEMP_HUMID_MONITOR";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");
    
    // Initialize NVS (required for Zigbee)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // TODO: Initialize Zigbee stack here
    ESP_LOGI(TAG, "Zigbee initialization - TO BE IMPLEMENTED");
    
    // Print detailed chip information
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip Details: %d CPU core(s)", chip_info.cores);
    
    if (esp_flash_get_size(nullptr, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %" PRIu32 "MB", flash_size / (1024 * 1024));
    }
    
    // Main operation loop
    int count = 0;
    float temperature = 0, humidity = 0;
    
    while (true) {
        // Read DHT22 every 30 seconds (15 cycles)
        if (count % 15 == 0) {
            esp_err_t ret = sensor_dht22_read(&temperature, &humidity);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
                // TODO: Update Zigbee attributes here
            } else {
                ESP_LOGW(TAG, "Failed to read DHT22 sensor");
            }
        }
        
        count++;
        vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second delay
    }
}
