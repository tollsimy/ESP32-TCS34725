#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t SDA_PIN=18;
uint8_t SCL_PIN=19;
uint8_t I2C_PORT=0;

#include "ESP32_TCS34725.h"

static const char *TAG = "Demo-TCS34725";

void TCS_task(){
    for(;;){
        float red, green, blue;
        uint16_t temp, lux;
        ESP32_TCS34725 TCS={0};
        if(TCS_init(&TCS)){
            ESP_LOGI(TAG, "Init complete!");
        }
        TCS_enable(&TCS);
        setGain(&TCS,TCS34725_GAIN_1X);
        setIntegrationTime(&TCS,TCS34725_INTEGRATIONTIME_50MS);
        getRGB(&TCS,&red,&green,&blue);
        temp=calculateColorTemperature(red,green,blue);
        lux=calculateLux(red,green,blue);
        ESP_LOGI(TAG, "RED: %f, GREEN: %f, BLUE: %f", red, green, blue);
        ESP_LOGI(TAG, "Color Temperature: %d, LUX: %d", temp, lux);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{   
    xTaskCreate(&TCS_task, "TCS_task", 2048, NULL, 5, NULL);
}
