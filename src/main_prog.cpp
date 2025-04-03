
// creat file src/main_prog.cpp with content:
#include "main_prog.hpp"
#include "stm32u5xx_it.h"

#include "main.hpp"

#include "fdcan.hpp"
#include "gpio.hpp"
#include "simple_task.hpp"
#include "stmepic.hpp"
#include "status.hpp"


// ###############################################################################
// HARDWARE

se::GpioPin gpio_user_led_1(*USER_LED_1_GPIO_Port, USER_LED_1_Pin);
se::GpioPin gpio_user_led_2(*USER_LED_2_GPIO_Port, USER_LED_2_Pin);
se::GpioPin gpio_status_led(*STATUS_LED_GPIO_Port, STATUS_LED_Pin);
se::GpioPin gpio_transiver_can_cs(*CAN_TRANSIVER_CS_GPIO_Port, CAN_TRANSIVER_CS_Pin);

std::shared_ptr<se::FDCAN> fdcan1;


// ###############################################################################
// TASKS
se::SimpleTask blink_task;


void task_f_blink(se::SimpleTask &task, void *args) {
  (void)args;
  gpio_status_led.toggle();
}


void main_prog() {

  // Initialize FDCAN
  FDCAN_FilterTypeDef fdfilter = {};
  STMEPIC_ASSING_TO_OR_HRESET(fdcan1, se::FDCAN::Make(hfdcan1, fdfilter, nullptr, nullptr));
  STMEPIC_NONE_OR_HRESET(fdcan1->hardware_start());

  STMEPIC_NONE_OR_HRESET(blink_task.task_init(task_f_blink, nullptr, 500, nullptr, 300));
  STMEPIC_NONE_OR_HRESET(blink_task.task_run());

  // the scheduler will start after exiting main_prog
}
