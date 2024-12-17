#include <stdio.h>
#include "urosHandler.hpp"
#include <shooting_sub.hpp>
#include <qmd.hpp>
#include <driver/gpio.h>
#include <esp_log.h>

#define LOG "MAIN"
uros_master_node* master_node = 0;

int pwmPins[] = {GPIO_NUM_2,GPIO_NUM_4};
int dirPins[] = {GPIO_NUM_15,GPIO_NUM_13};

extern "C" void app_main(void)
{
    
    qmd* handler = new qmd(pwmPins, dirPins, 2);
    handler->setRange(19900,0);

    master_node = new uros_master_node("master_node");

    master_node->add_urosElement({
        // new simPub(),
        new simSub(handler),
    });
    ESP_LOGI(LOG, "R2-Alpha is under development !!");

}