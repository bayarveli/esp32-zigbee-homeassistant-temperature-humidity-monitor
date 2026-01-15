#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_core.h"
#include "bme280.h"
#include "i2c_bus.h"

static const char* TAG = "ZIGBEE_TEMP_HUMID_MONITOR";

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT          10      /* esp temperature sensor device endpoint, used for temperature measurement */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

#define ESP_TEMP_SENSOR_UPDATE_INTERVAL (1)     /* Local sensor update interval (second) */
#define ESP_TEMP_SENSOR_MIN_VALUE       (-10)   /* Local sensor min measured value (degree Celsius) */
#define ESP_TEMP_SENSOR_MAX_VALUE       (80)    /* Local sensor max measured value (degree Celsius) */

/* I2C and BME280 configuration */
#define I2C_MASTER_SCL_IO               2       /* GPIO2 for I2C SCL */
#define I2C_MASTER_SDA_IO               1       /* GPIO1 for I2C SDA */
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_FREQ_HZ              400000  /* 400kHz */
#define BME280_I2C_ADDRESS              0x76    /* BME280 I2C address (SDO to GND) */

/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
//  */
#define MANUFACTURER_NAME "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

/* I2C and BME280 handles */
static i2c_bus_handle_t i2c_bus_handle = NULL;
static bme280_handle_t bme280_handle = NULL;
static bool bme280_ready = false;  /* Indicates BME280 is initialized and ready */
static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static void esp_app_humidity_sensor_handler(float humidity)
{
    uint16_t measured_value = (uint16_t)(humidity * 100);
    /* Update humidity sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &measured_value, false);
    esp_zb_lock_release();
}

static void esp_app_temp_sensor_handler(float temperature)
{
    int16_t measured_value = zb_temperature_to_s16(temperature);
    /* Update temperature sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_value, false);
    esp_zb_lock_release();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited) {
        /* Initialize I2C bus using i2c_bus API (compatible with BME280 library) */
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };

        i2c_bus_handle = i2c_bus_create(I2C_MASTER_NUM, &i2c_conf);
        if (i2c_bus_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C bus");
            return ESP_FAIL;
        }

        /* Initialize BME280 */
        bme280_handle = bme280_create(i2c_bus_handle, BME280_I2C_ADDRESS);
        if (bme280_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create BME280 handle");
            return ESP_FAIL;
        }

        ESP_RETURN_ON_ERROR(bme280_default_init(bme280_handle), TAG, "Failed to initialize BME280");
        ESP_LOGI(TAG, "BME280 initialized successfully");
        bme280_ready = true;  /* Mark as ready for reading */
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

static void bme280_sensor_task(void *pvParameters)
{
    /* Wait for BME280 to be fully initialized and ready */
    while (!bme280_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Extra delay to ensure BME280 is fully stabilized */
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "BME280 sensor task started");

    while (true) {
        float temperature = 0.0f, humidity = 0.0f, pressure = 0.0f;

        /* Read sensor values */
        if (bme280_read_temperature(bme280_handle, &temperature) == ESP_OK &&
            bme280_read_humidity(bme280_handle, &humidity) == ESP_OK &&
            bme280_read_pressure(bme280_handle, &pressure) == ESP_OK) {

            ESP_LOGI(TAG, "Temp: %.1f°C, Humidity: %.1f%%, Pressure: %.0fhPa",
                     temperature, humidity, pressure);

            /* Update Zigbee attributes */
            esp_app_temp_sensor_handler(temperature);
            esp_app_humidity_sensor_handler(humidity);
        } else {
            ESP_LOGW(TAG, "Failed to read BME280 sensor");
        }
        /* Read every 30 seconds (battery friendly) */
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)(*p_sg_p);
    
    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:            
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
                ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted");
                }
            } else {
                ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                        esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                    ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                        extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                        extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                        esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            } else {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                    esp_err_to_name(err_status));
            break;
    }
}

static esp_zb_cluster_list_t *custom_sensor_clusters_create(esp_zb_temperature_sensor_cfg_t *temp_cfg, esp_zb_humidity_meas_cluster_cfg_t *humidity_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    /* Add basic cluster with device info */
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(temp_cfg->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void*)MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void*)MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add identify cluster */
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(temp_cfg->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    
    /* Add temperature measurement cluster */
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(temp_cfg->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    /* Add humidity measurement cluster */
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(humidity_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    return cluster_list;
}

static esp_zb_ep_list_t *custom_sensor_ep_create(uint8_t endpoint_id, esp_zb_temperature_sensor_cfg_t *temp_cfg, esp_zb_humidity_meas_cluster_cfg_t *humidity_cfg)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_sensor_clusters_create(temp_cfg, humidity_cfg), endpoint_config);
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create customized multi-sensor endpoint */
    
    /* Configure temperature sensor */
    esp_zb_temperature_sensor_cfg_t temp_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    temp_cfg.temp_meas_cfg.min_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    temp_cfg.temp_meas_cfg.max_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);
    
    /* Configure humidity sensor */
    esp_zb_humidity_meas_cluster_cfg_t humidity_cfg = {
        .min_value = 0,
        .max_value = 10000,  /* 0-100% humidity in ZCL format (0.01% steps) */
    };
    
    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &temp_cfg, &humidity_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Config the reporting info for temperature */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 60,      /* Wait at least 60s between reports */
        .u.send_info.max_interval = 3600,    /* Force report every hour */
        .u.send_info.def_min_interval = 60,
        .u.send_info.def_max_interval = 3600,
        .u.send_info.delta.u16 = 50,         /* Report on 0.5°C change */
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&reporting_info);

    /* Config the reporting info for humidity */
    esp_zb_zcl_reporting_info_t humidity_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 60,      /* Wait at least 60s between reports */
        .u.send_info.max_interval = 3600,    /* Force report every hour */
        .u.send_info.def_min_interval = 60,
        .u.send_info.def_max_interval = 3600,
        .u.send_info.delta.u16 = 50,         /* Report on 0.5% humidity change */
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&humidity_reporting_info);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting...");
    
    // Initialize NVS (required for Zigbee)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Print detailed chip information
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip Details: %d CPU core(s)", chip_info.cores);
    
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %" PRIu32 "MB", flash_size / (1024 * 1024));
    }

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    /* Start BME280 sensor task */
    xTaskCreate(bme280_sensor_task, "BME280_Sensor", 2048, NULL, 4, NULL);
}
