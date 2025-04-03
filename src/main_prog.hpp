#pragma once
#include "main.hpp"
#include "stmepic.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
#include "gpio.hpp"

namespace se = stmepic;

void main_prog();


extern std::shared_ptr<se::SimpleTask> blink_task;
extern std::shared_ptr<se::GpioPin> gpio_user_led_1;
extern std::shared_ptr<se::GpioPin> gpio_user_led_2;
extern std::shared_ptr<se::GpioPin> gpio_status_led;
extern std::shared_ptr<se::GpioPin> gpio_transiver_can_cs;
extern std::shared_ptr<se::FDCAN> fdcan1;
