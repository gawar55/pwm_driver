#include "iostream"
#include <qmd.hpp>
#include "driver/gpio.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#define LOG "MAIN"

int pwmPins[] = {GPIO_NUM_2,GPIO_NUM_4};
int dirPins[] = {GPIO_NUM_15,GPIO_NUM_13};

extern "C" void app_main(void)
{
    qmd* handler = new qmd(pwmPins, dirPins, 1);
    handler->setRange(19900,0); 
    // handler->setInvertingMode(true);
    handler->speeds[0] = 0.0f;
    handler->update();
    while(1)
    {
        vTaskDelay(100);
    }
}