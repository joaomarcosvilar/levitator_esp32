#include "comon.h"

#include "vl53l0x.h"

#include "freertos/FreeRTOS.h"


#define TAG                     "HEIGHT_SENSOR"

#define VL53L0X_XSHUT_PIN       (-1)
#define VL53L0X_ADDRESS         (0x29)

vl53l0x_t *sensor = NULL;

esp_err_t height_sensor_init(void) {
    sensor = vl53l0x_config(
        I2C_NUM,         
        I2C_SCL_GPIO,
        I2C_SDA_GPIO,
        VL53L0X_XSHUT_PIN,
        VL53L0X_ADDRESS,
        1
    );
    if (!sensor) {
        ESP_LOGE(TAG, "Failed to configure VL53L0X sensor");
        return ESP_FAIL;
    }

    const char *err = vl53l0x_init(sensor);
    if (err) {
        ESP_LOGE(TAG, "VL53L0X initialization failed: %s", err);
        vl53l0x_end(sensor);
        return ESP_FAIL;
    }

    err = vl53l0x_setMeasurementTimingBudget(sensor, 20 * 1000);
    if (err) {
        ESP_LOGW(TAG, "Failed to set timing budget: %s", err);
        return ESP_FAIL;
    }

    vl53l0x_startContinuous(sensor, 0);
    
    return ESP_OK;
}

uint16_t height_sensor_get(void)
{
    if(!sensor)
    {
        ESP_LOGE(TAG, "Device not init");
        return 0;
    }

    return vl53l0x_readRangeContinuousMillimeters(sensor);
}