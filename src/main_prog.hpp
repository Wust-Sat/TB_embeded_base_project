#pragma once
#include "main.hpp"
#include "stmepic.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
#include "gpio.hpp"

namespace se = stmepic;

void main_prog();


extern se::SimpleTask blink_task;
extern se::GpioPin gpio_user_led_1;
extern se::GpioPin gpio_user_led_2;
extern se::GpioPin gpio_status_led;
extern se::GpioPin gpio_transiver_can_cs;
extern se::FDCAN fdcan1;
