
// creat file src/main_prog.cpp with content:
#include "main_prog.hpp"
#include "stm32u5xx_it.h"

#include "main.hpp"

#include "fdcan.hpp"
#include "gpio.hpp"
#include "simple_task.hpp"
#include "stmepic.hpp"
#include "status.hpp"
#include "logger.hpp"

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


// ###############################################################################
// Timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  if(htim->Instance == TIM6) {
    se::Ticker::get_instance().irq_update_ticker();
  }

  /* USER CODE END Callback 0 */
  if(htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


// Example tasks for a blinking LED

// This function is called before the main loop task starts and main loop will not start until this function returns
void task_f_blink_before(se::SimpleTask &task, void *args) {
  // here we will ignore the args and task reference explicitly
  (void)task;
  (void)args;

  // we set gptio to 1 to make sure the LED starts from ON state;
  gpio_user_led_1.write(1);

  // we wait 1 second for fun.
  vTaskDelay(1000);
}


// This function is called in the INFINITE LOOP of the blink_task with the given period
// the period can also be set to 0 for no delay.
// first arg is the reference to the task itself, second arg is the user args
void task_f_blink(se::SimpleTask &task, void *args) {
  (void)task;

  // we will get the fdcan1 from argument
  // std::shared_ptr<se::FDCAN> fdcan = *static_cast<std::shared_ptr<se::FDCAN> *>(args);

  // // we toggle the user LED
  gpio_status_led.toggle();

  // // here we perform a simple CAN write operation FOR example this would be a Heartbeat
  // se::CanDataFrame frame;
  // frame.frame_id  = 0x123;
  // frame.data[0]   = 0x01;
  // frame.data_size = 1;
  // (void)fdcan->write(frame);
}


void callback_can_some_frame(se::CanBase &can, se::CanDataFrame &frame, void *args) {
  // this function is called when the specific CAN frame is received
  (void)can;

  // here we can do something with the received frame
  // do some data converion frame.data ?

  // for example we can toggle the user LED 2 according to the received frame data
  gpio_user_led_2.write(frame.data[0] & 0x01);
}


void main_prog() {

  se::Ticker::get_instance().init(&htim6);

  // Initialize Logger
  se::Logger::get_instance().init(se::LOG_LEVEL::LOG_LEVEL_DEBUG, true, nullptr, true);

  // now we can print log messages to debugger console

  // Initialize FDCAN
  FDCAN_FilterTypeDef sFilterConfig = {};
  sFilterConfig.IdType              = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex         = 0;
  sFilterConfig.FilterType          = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig        = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1           = 0; // 0x915;
  sFilterConfig.FilterID2           = 0; // 0x1FFFFFFF; // all have to match
  se::FDcanFilterConfig filter_config;
  filter_config.filters.push_back(sFilterConfig);
  filter_config.fifo_number                  = se::FDCAN_FIFO::FDCAN_FIFO0;
  filter_config.globalFilter_NonMatchingStd  = FDCAN_REJECT;
  filter_config.globalFilter_NonMatchingExt  = FDCAN_REJECT;
  filter_config.globalFilter_RejectRemoteStd = FDCAN_FILTER_REMOTE;
  filter_config.globalFilter_RejectRemoteExt = FDCAN_FILTER_REMOTE;

  STMEPIC_ASSING_TO_OR_HRESET(fdcan1, se::FDCAN::Make(hfdcan1, filter_config, nullptr, nullptr));


  // We add a callback for the CAN frame with ID 0x200
  STMEPIC_NONE_OR_HRESET(fdcan1->add_callback(0x200, callback_can_some_frame, nullptr));

  // Start the FDCAN hardware
  STMEPIC_NONE_OR_HRESET(fdcan1->hardware_start());

  // Initialize task
  // the task will run in the main loop with a period of 500ms with fdcan1 passed as argument for the task
  STMEPIC_NONE_OR_HRESET(blink_task.task_init(task_f_blink, static_cast<void *>(&fdcan1), 500, task_f_blink_before, 300));
  STMEPIC_NONE_OR_HRESET(blink_task.task_run());

  // the scheduler will start after exiting main_prog
}
