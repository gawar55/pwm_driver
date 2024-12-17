#include "shooting_sub.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
const char * sub_topic_name = "speed";

simSub* simSub::def=0; 

const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
std_msgs__msg__Int32 message;
simSub::simSub(qmd* drv): handler(drv) 
{
  def=this;
};

void simSub::init() {
    rclc_subscription_init_default(&subscriber, node, sub_type_support, sub_topic_name);
    std_msgs__msg__Int32__init(&message);
    rclc_executor_add_subscription(exec, &subscriber, &message, subscription_callback, ON_NEW_DATA);
}

void simSub::subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  float pwmVal = ((float)(msg->data - 0) / (100 - 0));
  ESP_LOGI("MAIN","%f", pwmVal);
  def->handler->speeds[0] = pwmVal;
  def->handler->speeds[1] = pwmVal;
  def->handler->update();  

}