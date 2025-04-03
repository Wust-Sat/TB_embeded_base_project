 
//creat file src/main_prog.cpp with content:
#include "main_prog.hpp"
#include "stm32u5xx_it.h"

#include "main.hpp"

#include "stmepic.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
#include "gpio.hpp"


std::shared_ptr<se::SimpleTask> blink_task;
std::shared_ptr<se::GpioPin> gpio_user_led_1;
std::shared_ptr<se::GpioPin> gpio_user_led_2;
std::shared_ptr<se::GpioPin> gpio_status_led;
std::shared_ptr<se::GpioPin> gpio_transiver_can_cs;

std::shared_ptr<se::FDCAN> fdcan1;



void task_f_blink(se::SimpleTask& task, void* args){
  (void)args;
  gpio_status_led->toggle();
}


void main_prog(){

  blink_task = std::make_shared<se::SimpleTask>();

  // Initialize GPIO pins

  gpio_user_led_1 = std::make_shared<se::GpioPin>(*USER_LED_1_GPIO_Port, USER_LED_1_Pin);
  gpio_user_led_2 = std::make_shared<se::GpioPin>(*USER_LED_2_GPIO_Port, USER_LED_2_Pin);
  gpio_status_led = std::make_shared<se::GpioPin>(*STATUS_LED_GPIO_Port, STATUS_LED_Pin);
  gpio_transiver_can_cs = std::make_shared<se::GpioPin>(*CAN_TRANSIVER_CS_GPIO_Port, CAN_TRANSIVER_CS_Pin);

  // Initialize FDCAN
  FDCAN_FilterTypeDef fdfilter={};

  STMEPIC_ASSING_TO_OR_HRESET(fdcan1,se::FDCAN::Make(hfdcan1,fdfilter,nullptr,nullptr));


  // Your code here like your tasks, drivers, etc.
  // Do not start FreeRTOS kernel here since it will
  STMEPIC_NONE_OR_HARD_FAULT(blink_task->task_init(task_f_blink, nullptr, 1000, nullptr,300));
  STMEPIC_NONE_OR_HARD_FAULT(blink_task->task_run());

  // the scheduler will start after exiting main_prog
}
