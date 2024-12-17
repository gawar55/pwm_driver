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
#ifndef DRIVER_HPP
#define DRIVER_HPP


#include <driver/mcpwm_prelude.h>


/**
 * @brief motor specific configurations
 */
#define MOTOR_MIN_PULSEWIDTH_US 500      // Minimum pulse width in microseconds 
#define MOTOR_MAX_PULSEWIDTH_US 2500   // maximum pulse width in microseconds 
#define MOTOR_TIMEBASE_RESOLUTION_HZ 1 * 1000 * 1000   // clock frequency in Hertz  ( 1 MHz current) 
#define MOTOR_TIMEBASE_PERIOD        20000      // PWM time period in clock ticks   ( 20 ms current) 


#define MOTOR_COUNT_MAX 12  // maximum number of motor parallelly operated by this chipset
#define UNIT_CHANNEL_COUNT 6  // maximum number of motor parallelly operated a unit


/**
 * @brief qmd - quad channel motor driver
 * 
 * @details Quad channel pwm generator optimized such that it uses single mcpwm esp32 peripheral
 *  to compare and driver quad pwm signal simulataneously.
 * 
 * to driver a motor, the module generates one pwm signal and one direction signal.
 */
class qmd{

public:
    /**
     * @brief configures peripherals to generate pwm at mentioned GPIOs.
     * @param count number of channels to operate in parallel (max 12)
     * @param pwmPins  array of GPIO pin numbers for pwm output
     * @param dirPins  array of GPIO pin numbers for direction output
     */
    qmd(int pwmPins[], int dirPins[], int count = 0);


    /**
     * @brief mapping function to map normalized input (0 to 1) to ( MIN - MAX).
     * 
     * @param norm normalized float input strictly between 0.0f to 1.0f 
     * @return int - pwm uptime duration in terms of clock ticks.
     */
    static int map(float norm, float MAX_PWM = MOTOR_MAX_PULSEWIDTH_US, float MIN_PWM = MOTOR_MIN_PULSEWIDTH_US){
        return (norm) * (MAX_PWM - MIN_PWM) + MIN_PWM; 
    }


    /**
     * @brief Set the Range of PWM output
     * 
     * @param maxPwm maximum pwm uptime in MICROSECONDS
     * @param minPwm minimum pwm uptime in MICROSECONDS
     */
    void setRange(float maxPwm, float minPwm);


    /**
     * @brief Set the Inverting Mode for reverse drive of motor
     * 
     * @param flipLogic true - set whether uptime is replaced with downtime when reverse drive 
     *                  false - uptime stays same for same magitude whether in forward or backward drive
     */
    inline void setInvertingMode(bool flipLogic = true) { invertingMode = flipLogic; };

    /**
     * @brief speeds of the motors to set must be strictly between -1 and 1
     * 
     */
    float speeds[MOTOR_COUNT_MAX] = {0.0f};

    void update();

private:
    
    float maxPwm = MOTOR_MAX_PULSEWIDTH_US, minPwm = MOTOR_MIN_PULSEWIDTH_US;
    
    int count = 0;
    int dirPins[MOTOR_COUNT_MAX] = {0};
    int pwmPins[MOTOR_COUNT_MAX] = {0};
    bool invertingMode = false;

    // mcpwm handlers for internal use  


    struct unitHandler
    {
        /// @brief  pcmwm timers
        mcpwm_timer_handle_t timer;

        /// @brief mcpwm operators
        mcpwm_oper_handle_t ops[3] = {nullptr};
        /// @brief mcpwm comparators
        mcpwm_cmpr_handle_t cmps[MOTOR_COUNT_MAX] = {nullptr};
        /// @brief mcpwm generators
        mcpwm_gen_handle_t gens[MOTOR_COUNT_MAX] = {nullptr};
    } unit0, unit1;


    void setupTimer(unitHandler& , int id = 0);
};



#endif// DRIVER_HPP
