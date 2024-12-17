// qmd - esp-idf quad motor driver 
// Copyright (C) 2023  akshay bansod <akshayb@gmx.com>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#include "qmd.hpp"
#include "driver/gpio.h"
#include <string.h>


// convert pin number to bit mask
#define GET_BIT(PIN)  (1ULL<<PIN) 

/// @brief timer configuration
mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = MOTOR_TIMEBASE_RESOLUTION_HZ,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = MOTOR_TIMEBASE_PERIOD,
    .flags = {}
};

/// @brief operator config
mcpwm_operator_config_t operator_config = {
    .group_id = 0, // operator must be in the same group to the timer
    .flags = {}
};


mcpwm_comparator_config_t comparator_config = {
    .flags = {1, 0, 0}
};


qmd::qmd( int pwmPins[], int dirPins[], int count) : count(count) {

    memcpy(this->dirPins, dirPins, sizeof(int) * count);
    memcpy(this->pwmPins, pwmPins, sizeof(int) * count);

    setupTimer(unit0);
    if(count > UNIT_CHANNEL_COUNT) setupTimer(unit1, 1);

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,               
        .pull_up_en = GPIO_PULLUP_DISABLE,      
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE 
    };

    for(int i = 0; i < count; i++){
        gpio_cfg.pin_bit_mask |= GET_BIT(dirPins[i]);
    }

    gpio_config(&gpio_cfg);
};

void qmd::setRange(float max_pwm, float min_pwm){
    minPwm = min_pwm;
    maxPwm = max_pwm;
}

void qmd::update()
{

    float speed = 0.0f;

    for( int  i = 0; i < UNIT_CHANNEL_COUNT; i++) {
        speed = speeds[i];
        if(speed < 0.0f) {
            gpio_set_level((gpio_num_t) dirPins[i], 1); 
            speed = (invertingMode) ?  speed + 1.0f : abs(speed);
        }
        else gpio_set_level((gpio_num_t) dirPins[i], 0);
        mcpwm_comparator_set_compare_value(unit0.cmps[i], map(speed, maxPwm, minPwm));
    };
}

void qmd::setupTimer(unitHandler &unit, int id)
{

    timer_config.group_id = id;
    operator_config.group_id = id;

    // common timer for both pwm drivers
    mcpwm_new_timer(&timer_config, &unit.timer);

    mcpwm_new_operator(&operator_config, &unit.ops[0]);
    mcpwm_new_operator(&operator_config, &unit.ops[1]);
    mcpwm_new_operator(&operator_config, &unit.ops[2]);

    mcpwm_operator_connect_timer(unit.ops[0], unit.timer);
    mcpwm_operator_connect_timer(unit.ops[1], unit.timer);
    mcpwm_operator_connect_timer(unit.ops[2], unit.timer);


    mcpwm_new_comparator(unit.ops[0], &comparator_config, &unit.cmps[0]);
    mcpwm_new_comparator(unit.ops[0], &comparator_config, &unit.cmps[1]);
    mcpwm_new_comparator(unit.ops[1], &comparator_config, &unit.cmps[2]);
    mcpwm_new_comparator(unit.ops[1], &comparator_config, &unit.cmps[3]);
    mcpwm_new_comparator(unit.ops[2], &comparator_config, &unit.cmps[4]);
    mcpwm_new_comparator(unit.ops[2], &comparator_config, &unit.cmps[5]);


    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pwmPins[0],
        .flags = {}
    };

    int offset = (id) ? UNIT_CHANNEL_COUNT : 0;

    generator_config.gen_gpio_num = pwmPins[offset + 0];
    mcpwm_new_generator(unit.ops[0], &generator_config, &unit.gens[0]);
    generator_config.gen_gpio_num = pwmPins[offset + 1];
    mcpwm_new_generator(unit.ops[0], &generator_config, &unit.gens[1]);
    generator_config.gen_gpio_num = pwmPins[offset + 2];
    mcpwm_new_generator(unit.ops[1], &generator_config, &unit.gens[2]);
    generator_config.gen_gpio_num = pwmPins[offset + 3];
    mcpwm_new_generator(unit.ops[1], &generator_config, &unit.gens[3]);
    generator_config.gen_gpio_num = pwmPins[offset + 4];
    mcpwm_new_generator(unit.ops[2], &generator_config, &unit.gens[4]);
    generator_config.gen_gpio_num = pwmPins[offset + 5];
    mcpwm_new_generator(unit.ops[2], &generator_config, &unit.gens[5]);




    // left comparator operations
    // go high on counter empty
    for(int i = 0; i < UNIT_CHANNEL_COUNT; i++){

        mcpwm_generator_set_action_on_timer_event(unit.gens[i],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    
        // go low on compare threshold
        mcpwm_generator_set_action_on_compare_event(unit.gens[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, unit.cmps[i], MCPWM_GEN_ACTION_LOW));
    }

    mcpwm_timer_enable(unit.timer);
    mcpwm_timer_start_stop(unit.timer, MCPWM_TIMER_START_NO_STOP);
};