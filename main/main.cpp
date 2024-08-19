#include <stdio.h>
#include <driver/mcpwm_prelude.h>
#include "driver/gpio.h"
#include <string.h>
#include <esp_log.h>
#define MOTOR_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define MOTOR_MAX_PULSEWIDTH_US 2500 // maximum pulse width in microseconds
#define MOTOR_TIMEBASE_RESOLUTION_HZ 1 * 1000 * 1000
#define MOTOR_TIMEBASE_PERIOD 20000 // PWM time period in clock ticks   ( 20 ms current)

#define MOTOR_COUNT_MAX 12 // maximum number of motor parallelly operated by this chipset
#define UNIT_CHANNEL_COUNT 6

mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = MOTOR_TIMEBASE_RESOLUTION_HZ,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = MOTOR_TIMEBASE_PERIOD,
    .flags = {}

};

mcpwm_operator_config_t operator_config = {
    .group_id = 0,
    .flags = {}};

mcpwm_comparator_config_t compartor_config = {
    .flags = {1, 0, 0}};

#define GET_BIT(PIN) (1ULL << PIN)

class motor_driver
{

public:
    float maxPwm = MOTOR_MAX_PULSEWIDTH_US, minPwm = MOTOR_MIN_PULSEWIDTH_US;

    int count = 0;
    int dirPins[MOTOR_COUNT_MAX] = {0};
    int pwmPins[MOTOR_COUNT_MAX] = {0};
    bool invertingMode = false;

    struct unitHandler
    {
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper[3] = {nullptr};
        mcpwm_cmpr_handle_t cmpr[6] = {nullptr};
        mcpwm_gen_handle_t gens[6] = {nullptr};

    } unit0, unit1;

    void setupTimer(unitHandler &unit, int id = 0)
    {
        timer_config.group_id = id;
        operator_config.group_id = id;

        mcpwm_new_timer(&timer_config, &unit.timer);

        mcpwm_new_operator(&operator_config, &unit.oper[0]);
        mcpwm_new_operator(&operator_config, &unit.oper[1]);
        mcpwm_new_operator(&operator_config, &unit.oper[2]);

        mcpwm_operator_connect_timer(unit.oper[0], unit.timer);
        mcpwm_operator_connect_timer(unit.oper[1], unit.timer);
        mcpwm_operator_connect_timer(unit.oper[2], unit.timer);

        mcpwm_new_comparator(unit.oper[0], &compartor_config, &unit.cmpr[0]);
        mcpwm_new_comparator(unit.oper[0], &compartor_config, &unit.cmpr[1]);
        mcpwm_new_comparator(unit.oper[1], &compartor_config, &unit.cmpr[2]);
        mcpwm_new_comparator(unit.oper[1], &compartor_config, &unit.cmpr[3]);
        mcpwm_new_comparator(unit.oper[2], &compartor_config, &unit.cmpr[4]);
        mcpwm_new_comparator(unit.oper[2], &compartor_config, &unit.cmpr[5]);

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = pwmPins[0],
            .flags = {}};
        ESP_LOGI("main", "done generator config");

        generator_config.gen_gpio_num = pwmPins[0];
        mcpwm_new_generator(unit.oper[0], &generator_config, &unit.gens[0]);
        generator_config.gen_gpio_num = pwmPins[1];
        mcpwm_new_generator(unit.oper[0], &generator_config, &unit.gens[1]);
        generator_config.gen_gpio_num = pwmPins[2];
        mcpwm_new_generator(unit.oper[1], &generator_config, &unit.gens[2]);
        generator_config.gen_gpio_num = pwmPins[3];
        mcpwm_new_generator(unit.oper[1], &generator_config, &unit.gens[3]);
        generator_config.gen_gpio_num = pwmPins[4];
        mcpwm_new_generator(unit.oper[2], &generator_config, &unit.gens[4]);
        generator_config.gen_gpio_num = pwmPins[5];
        mcpwm_new_generator(unit.oper[2], &generator_config, &unit.gens[5]);

        for (int i = 0; i < UNIT_CHANNEL_COUNT; i++)
        // for (int i = 0; i < 1; i++)
        {
            mcpwm_generator_set_action_on_timer_event(unit.gens[i],
                                                      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));

            // go low on compare threshold
            mcpwm_generator_set_action_on_compare_event(unit.gens[i],
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, unit.cmpr[i], MCPWM_GEN_ACTION_LOW));
            ESP_LOGI("main", "done compare");
        }
        mcpwm_timer_enable(unit.timer);
        mcpwm_timer_start_stop(unit.timer, MCPWM_TIMER_START_NO_STOP);
        ESP_LOGI("main", "done all ");
    }

    motor_driver(int pwmPins[], int dirPins[], int count) : count(count)
    {
        memcpy(this->pwmPins, pwmPins, sizeof(count));
        memcpy(this->dirPins, dirPins, sizeof(count));
        ESP_LOGI("main", " enter ");
        setupTimer(unit0);
        if (count > UNIT_CHANNEL_COUNT)
            setupTimer(unit1, 1);

        gpio_config_t gpio_cfg = {
            .pin_bit_mask = 0,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};

        for (int i = 0; i < count; i++)
        {
            gpio_cfg.pin_bit_mask |= GET_BIT(dirPins[i]);
        }

        gpio_config(&gpio_cfg);
        ESP_LOGI("main", "gpio config ");
    };

    int map(float speed, float maxPwm, float minPwm)
    {
        ESP_LOGI("main", " map %f ", (speed) * ((maxPwm - minPwm) / 255.0f) + minPwm);
        return (speed) * ((maxPwm - minPwm) / 255.0f) + minPwm;
    }

    void update(float val)
    {
        float speed = val;
        for (int i = 0; i < UNIT_CHANNEL_COUNT; i++)
        {
            gpio_set_level((gpio_num_t)dirPins[i], 1);
            mcpwm_comparator_set_compare_value(unit0.cmpr[i], (uint32_t)(map(speed, maxPwm, minPwm)));
        }
        ESP_LOGI("main", "mcpwm_comparator_set_compare_value ");
    }

    void setRange(float min_Pwm, float max_Pwm)
    {
        minPwm = min_Pwm;
        maxPwm = max_Pwm;
    }
};
#define A_DIR1 GPIO_NUM_18
#define A_PWM1 GPIO_NUM_4

// FR
#define A_DIR2 GPIO_NUM_7
#define A_PWM2 GPIO_NUM_15

// BL
#define A_DIR3 GPIO_NUM_16
#define A_PWM3 GPIO_NUM_17

// BR
#define A_DIR4 GPIO_NUM_2
#define A_PWM4 GPIO_NUM_5

// lift
#define LIFT_DIR GPIO_NUM_21
#define LIFT_PWM GPIO_NUM_35

// GRAB
#define GRAB_DIR GPIO_NUM_2
#define GRAB_PWM GPIO_NUM_19

int pwmPins[4] = {A_PWM1, A_PWM2, A_PWM3, A_PWM4};
int dirPins[4] = {A_DIR1, A_DIR2, A_DIR3, A_DIR4};

extern "C" void app_main(void)
{
    motor_driver *DriverHandler = new motor_driver(pwmPins, dirPins, 1);
    ESP_LOGI("main", " Intilialising");
    DriverHandler->setRange(0, 19990);
    DriverHandler->update(1);
}