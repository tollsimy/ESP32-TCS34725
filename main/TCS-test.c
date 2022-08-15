#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_TCS34725.h"

static const char *TAG = "Demo-TCS34725";

#define SDA_PIN (27)
#define SCL_PIN (32)
#define I2C_PORT (0)

I2C_CONF={
    .mode = I2C_MODE_MASTER;
    .sda_io_num = SDA_PIN;
    .scl_io_num = SCL_PIN;
    .sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    .scl_pullup_en = GPIO_PULLUP_DISABLE;
    .master.clk_speed = 400000;               //I2C Full Speed
}

void TCS_task(){
    //Install I2C Driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(I2C_CONF)));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    for(;;){
        float red, green, blue;
        uint16_t temp, lux;
        ESP32_TCS34725 TCS={0};
        if(TCS_init(&TCS, I2C_PORT)==ESP_OK){
            ESP_LOGI(TAG, "Init complete!");
        }
        TCS_enable(&TCS);
        TCS_setGain(&TCS,TCS34725_GAIN_1X);
        TCS_setIntegrationTime(&TCS,TCS34725_INTEGRATIONTIME_50MS);
        TCS_getRGB(&TCS,&red,&green,&blue);
        temp=TCS_calculateColorTemperature(red,green,blue);
        lux=TCS_calculateLux(red,green,blue);
        ESP_LOGI(TAG, "RED: %f, GREEN: %f, BLUE: %f", red, green, blue);
        ESP_LOGI(TAG, "Color Temperature: %d, LUX: %d", temp, lux);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
}

void app_main(void)
{   
    xTaskCreate(&TCS_task, "TCS_task", 2048, NULL, 5, NULL);
}
