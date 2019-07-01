// https://docs.espressif.com/projects/esp-idf/en/stable/index.html

// #include <stdio.h>
// #include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"


static const char* TAG = "Ultrasonic LED";

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

//the LED pin numbers
#define LEDC_HS_CH0_GPIO 18
#define LEDC_CH_NUM 1

//other LED settings
#define LEDC_PWM_DUTY_RES LEDC_TIMER_13_BIT //13-bit pwm resolution
#define MIN_LEDC_FREQ_HZ 0
#define MAX_LEDC_FREQ_HZ 6000
#define MAX_DISTANCE_CM  50.0 //the max distance for the sensor to detect

#define BLINK_THRESHOLD 0.80 //blinking threshold for the LED
#define HISTORY_NUM 4 //number of points to average for LED brightness smoothing

//the ultrasonic sensor pin numbers
#define TRIGGER_GPIO 4 //the output pin
#define ECHO_GPIO 5 //the input pin

#define SPEED_OF_SOUND 343.0 //in m/s
#define CM_PER_MICROSECOND (SPEED_OF_SOUND / 10000.0)
#define MAX_MICROSECONDS  (MAX_DISTANCE_CM / CM_PER_MICROSECOND * 2.0) //the sensor timeout

#define milliseconds(ms) (ms / portTICK_PERIOD_MS)


// LED configuration
// ----------------------------------------------------------------------------
ledc_channel_config_t ledc_channel[LEDC_CH_NUM];

//configure and initialize the LEDs
static void config_leds(){

  // configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_PWM_DUTY_RES, // resolution of PWM duty
      .freq_hz = MAX_LEDC_FREQ_HZ, // frequency of PWM signal
      .speed_mode = LEDC_HIGH_SPEED_MODE, // timer mode
      .timer_num = LEDC_TIMER_0 // timer index
  };
  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);

  ledc_channel[0].channel = LEDC_CHANNEL_0;
  ledc_channel[0].duty = 0;
  ledc_channel[0].gpio_num = LEDC_HS_CH0_GPIO;
  ledc_channel[0].speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_channel[0].timer_sel = LEDC_TIMER_0;

  // set LED Controller with defined configuration
  for (int ch = 0; ch < LEDC_CH_NUM; ch++) {
      ledc_channel_config(&ledc_channel[ch]);
  }

  // install the fade service
  ledc_fade_func_install(0);

}

static void set_ledc_pwm(uint32_t duty){
  for (int ch = 0; ch < LEDC_CH_NUM; ch++) {
    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, duty);
    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
  }
}

// ultrasonic GPIO configuration
// ----------------------------------------------------------------------------
static void config_gpio(){

  gpio_config_t gpio_conf;

  gpio_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
  gpio_conf.mode = GPIO_MODE_OUTPUT; //output pin
  gpio_conf.pin_bit_mask = (1 << TRIGGER_GPIO);
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&gpio_conf);

  gpio_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_conf.pin_bit_mask = (1 << ECHO_GPIO);
  gpio_conf.mode = GPIO_MODE_INPUT; //input pin
  gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; //pull the pin down to low
  gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&gpio_conf);

}

// trigger the ultrasonic sensor
static void ultrasonic_trigger(){

  gpio_set_level(TRIGGER_GPIO, 0);
  usleep(5);
  gpio_set_level(TRIGGER_GPIO, 1);
  usleep(10);
  gpio_set_level(TRIGGER_GPIO, 0);

}

// a struct for LED/sensor information
struct LEDI
{
  int64_t brt0; //the blink "reset" timer
  int64_t bt0; //the blink timer
  int blinks; //how many blinks have occurred
  int blink_state; //whether or not the LED is OFF or ON
  uint32_t read_duty; //current duty as "read" by the sensor
  uint32_t duty; //the duty of th LED
  double cm; //distance determined by the sensor
  int64_t ping_time; //sensor ping time
  double brightness_scale; //the brightness scaled from 0 to 1
};


// the main function for setting the brightness of the LEDs and blinking them
static void control_leds(struct LEDI *ledi){

  uint32_t threshold = (double)(MAX_LEDC_FREQ_HZ - MIN_LEDC_FREQ_HZ) * BLINK_THRESHOLD + MIN_LEDC_FREQ_HZ;

  if (ledi->read_duty > threshold){

    if(ledi->blinks >= 5){

      //reset the blink timer after 5 seconds
      if(esp_timer_get_time() - ledi->brt0 >= 1000000*5){
        ledi->bt0 = esp_timer_get_time();
        ledi->brt0 = esp_timer_get_time();
        ledi->blinks = 0;
        ledi->blink_state = 0;
      }

      set_ledc_pwm(ledi->duty);
      return;
    }

    //blink the leds in 50ms intervals
    if(esp_timer_get_time() - ledi->bt0 >= 50000){

      if(ledi->blink_state == 1){
        set_ledc_pwm(ledi->duty);
        ledi->blink_state = 0;
        ledi->blinks += 1;
      } else {
        set_ledc_pwm(0);
        ledi->blink_state = 1;
      }

      //update the start time for the next blink
      ledi->bt0 = esp_timer_get_time();
      return;

    }
    return;
  }

  //if the led duty is less than the threshold
  set_ledc_pwm(ledi->duty);
  ledi->blink_state = 0;
  ledi->blinks = 0;
  return;

}

// calculate the distance detected from the sensor and the resulting brightness of the LEDs
static void calc_distance(struct LEDI *ledi){

  //the ultrasonic process:
  //trigger the device
  //wait for the echo pin to get pulled up to HIGH
  //as soon as the echo pin is HIGH, start timing
  //once the echo pin is pulled down to LOW, stop timing
  //the measured time (ping time) is how long the ultrasonic sound wave took to travel from the device, bounce off of something, and received back
  //multiply that time by the speed of sound, and divide by 2 to get the detected distance

  int64_t t0; //the start time after the device was triggered
  int64_t ping_time = 0; //measure the time interval in microseconds

  t0 = esp_timer_get_time();
  while (gpio_get_level(ECHO_GPIO) != 1)
  {
    control_leds(ledi);
    if ((ping_time = esp_timer_get_time() - t0) >= MAX_MICROSECONDS) {
      break;
    }
  }

  t0 = esp_timer_get_time();

  if(ping_time < MAX_MICROSECONDS){
    while (gpio_get_level(ECHO_GPIO) == 1)
    {
      control_leds(ledi);
      if ((ping_time = esp_timer_get_time() - t0) >= MAX_MICROSECONDS) {
        break;
      }
    }
  }

  ledi->ping_time = MIN(ping_time, MAX_MICROSECONDS);
  ledi->cm = (double)ledi->ping_time * CM_PER_MICROSECOND / 2.0;
  ledi->brightness_scale = (MAX_DISTANCE_CM - ledi->cm) / MAX_DISTANCE_CM;
  ledi->read_duty = (double)(MAX_LEDC_FREQ_HZ - MIN_LEDC_FREQ_HZ) * ledi->brightness_scale + MIN_LEDC_FREQ_HZ;

}


// the main task
static void ultrasonic_task(void* arg){

  //subscribe the task to the watchdog timer
  esp_task_wdt_add(NULL);

  int64_t pt0 = esp_timer_get_time(); //print info timer

  struct LEDI ledi;
  ledi.brt0 = esp_timer_get_time();
  ledi.bt0 = esp_timer_get_time();
  ledi.blinks = 0;
  ledi.blink_state = 0;
  ledi.read_duty = 0;
  ledi.duty = 0;
  ledi.cm = 0.0;
  ledi.ping_time = 0;
  ledi.brightness_scale = 0.0;

  set_ledc_pwm(ledi.duty);

  uint32_t duty_history[HISTORY_NUM] = { ledi.duty };
  int history_index = 0;

  for(;;) {

    int64_t d0 = esp_timer_get_time();

    //trigger the ultrasonic sensor
    ultrasonic_trigger();

    //calculate the distance and the resuting LED duty
    calc_distance(&ledi);

    // update and average the last HISTORY_NUM duty values
    duty_history[history_index++ % HISTORY_NUM] = ledi.read_duty;
    ledi.duty = 0;
    for(int i = 0; i < HISTORY_NUM; i++)
      ledi.duty += duty_history[i];
    ledi.duty /= HISTORY_NUM;

    control_leds(&ledi);

    //small time adjustment for the task delay to "guarantee" each loop of the task takes 100ms
    int dlay = MAX(100 - (esp_timer_get_time() - d0) / 1000, 0);

    vTaskDelay(milliseconds(dlay));

    control_leds(&ledi);

    if(esp_timer_get_time() -  pt0 >= 1000000){
      ESP_LOGD(TAG, "Distance: %.2f | Brightness: %.2f | Duty: %u | Read Duty: %u", ledi.cm, ledi.brightness_scale, ledi.duty, ledi.read_duty);
      pt0 = esp_timer_get_time();
    }

    // reset the watchdog timer
    esp_task_wdt_reset();

  }

}



void app_main(){

  ESP_LOGI(TAG, "Initializing LEDs.");
  config_leds();

  ESP_LOGI(TAG, "Initializing GPIO pins.");
  config_gpio();

  ESP_LOGI(TAG, "Creating the ultrasonic task.");
  xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 10, NULL);

}
