#ifndef Shooting_SUB_HPP
#define Shooting_SUB_HPP

#include "urosElement.hpp"
#include "qmd.hpp"
#include <std_msgs/msg/int32.h>

class simSub : public urosElement{
public:
    static simSub* def;
    simSub(qmd* drv);
    // microros subscriber initialization
    void init();

    // subscription callback
    static void subscription_callback(const void * msgin);

    qmd *handler = 0;
private:
    rcl_subscription_t subscriber;

};

#endif //Shooting_SUB_HPP