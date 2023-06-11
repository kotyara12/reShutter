/* 
   EN: Class for controlling a faucet or window with a reverse step by step
   RU: Класс для управления краном или форточкой с реверсом по шагам
   --------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Страница проекта: https://github.com/kotyara12/reShutter
*/

#ifndef __RE_SHUTTER_H__
#define __RE_SHUTTER_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include "project_config.h"
#include "def_consts.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

class rShutter;

typedef bool (*cb_shutter_publish_t) (rShutter *shutter, char* topic, char* payload, bool free_topic, bool free_payload);
typedef void (*cb_shutter_change_t) (rShutter *shutter, uint8_t from_step, uint8_t to_step, uint8_t max_steps);
typedef void (*cb_shutter_timer_t) (rShutter *shutter, uint8_t pin, bool state);
typedef void (*cb_shutter_gpio_wrap_t) (rShutter *shutter, uint8_t pin);
typedef bool (*cb_shutter_gpio_init_t) (rShutter *shutter, uint8_t pin, bool level_active);
typedef bool (*cb_shutter_gpio_change_t) (rShutter *shutter, uint8_t pin, bool physical_level);

class rShutter {
  public:
    rShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
      int8_t min_steps, int8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
      cb_shutter_gpio_wrap_t cb_gpio_before, cb_shutter_gpio_wrap_t cb_gpio_after, cb_shutter_timer_t cb_timer, 
      cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish);
    ~rShutter();

    // Current state
    uint8_t getState();
    uint8_t getMaxSteps();
    time_t getLastChange();
    float getPercent();
    bool isFullOpen();
    bool isFullClose();

    // Generate JSON
    char* getStateJSON(uint8_t state);
    char* getTimestampsJSON();
    char* getJSON();

    bool Init();

    // Open or close the shutter by a specified number of steps
    bool Change(int8_t steps, bool publish);
    bool OpenFull(bool publish);
    bool CloseFullEx(bool forced, bool call_cb, bool publish);
    bool CloseFull(bool forced, bool publish);
    bool isBusy();
    bool Break();

    // Limits
    int8_t checkLimits(int8_t steps);
    bool setMinLimit(uint8_t limit, bool publish);
    bool setMaxLimit(uint8_t limit, bool publish);
    bool clearMinLimit(bool publish);
    bool clearMaxLimit(bool publish);

    // MQTT
    void mqttSetCallback(cb_shutter_publish_t cb_publish);
    char* mqttTopicGet();
    bool mqttTopicSet(char* topic);
    bool mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3);
    void mqttTopicFree();
    bool mqttPublish();

    // !!! Only for the timer callback function! Don't use it directly
    bool StopAll();
  protected:
    uint8_t     _pin_open = 0;                    // Pin number to open
    bool        _level_open = true;               // Logic level to open
    uint8_t     _pin_close = 0;                   // Pin number to close
    bool        _level_close = true;              // Logic level to close

    virtual bool gpioInit() = 0;
    virtual bool gpioSetLevel(uint8_t pin, bool physical_level) = 0; 
  private:
    uint32_t              _full_time = 15000;       // Full closing time in milliseconds
    int8_t                _min_steps = 0;           // Number of steps to fully closed
    int8_t                _max_steps = 10;          // Number of steps to fully open
    uint32_t              _step_time = 1000;        // Time delay by one step in milliseconds
    float                 _step_time_adj = 1.00;    // Delay adjustment factor for each next step
    uint32_t              _step_time_fin = 0;       // Finishing time
    int8_t                _state = 0;               // Current state
    uint8_t               _pin_open_state = 0;      // Current state of open GPIO
    uint8_t               _pin_close_state = 0;     // Current state of close GPIO
    int8_t                _limit_min = INT8_MIN;    // Minimum opening limit
    int8_t                _limit_max = INT8_MAX;    // Maximum opening limit
    time_t                _last_changed = 0;        // Time of last state change
    time_t                _last_open = 0;           // Time of last open
    time_t                _last_close = 0;          // Time of last close
    int8_t                _last_max_state = 0;      // Last maximum opening
    esp_timer_handle_t    _timer = nullptr;         // Step timer
    char*                 _mqtt_topic = nullptr;    // MQTT topic

    cb_shutter_change_t     _on_changed = nullptr;   // Pointer to the callback function to be called after load switching
    cb_shutter_gpio_wrap_t  _on_before = nullptr;    // Pointer to the callback function to be called before set physical level to GPIO
    cb_shutter_gpio_wrap_t  _on_after = nullptr;     // Pointer to the callback function to be called after set physical level to GPIO
    cb_shutter_timer_t      _on_timer = nullptr;     // Pointer to the callback function to be called before start timer and after end timer
    cb_shutter_publish_t    _mqtt_publish = nullptr; // Pointer to the publish callback function

    uint32_t calcStepTimeout(int8_t step);
    bool gpioSetLevelPriv(uint8_t pin, bool physical_level);
    bool DoChange(int8_t steps, bool call_cb, bool publish);

    // Timer
    bool timerCreate();
    bool timerFree();
    bool timerActivate(uint8_t pin, bool level, uint32_t duration_ms);
    bool timerIsActive();
    bool timerStop();
};

class rGpioShutter: public rShutter {
  public:
    rGpioShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
      int8_t min_steps, int8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
      cb_shutter_gpio_wrap_t cb_gpio_before, cb_shutter_gpio_wrap_t cb_gpio_after, cb_shutter_timer_t cb_timer, 
      cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish);
  protected:
    bool gpioInit() override;
    bool gpioSetLevel(uint8_t pin, bool physical_level) override; 
};

class rIoExpShutter: public rShutter {
  public:
    rIoExpShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
      int8_t min_steps, int8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
      cb_shutter_gpio_init_t cb_gpio_init, cb_shutter_gpio_change_t cb_gpio_change,
      cb_shutter_gpio_wrap_t cb_gpio_before, cb_shutter_gpio_wrap_t cb_gpio_after, cb_shutter_timer_t cb_timer, 
      cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish);
  protected:
    bool gpioInit() override;
    bool gpioSetLevel(uint8_t pin, bool physical_level) override; 
  private:
    cb_shutter_gpio_init_t _gpio_init = nullptr;
    cb_shutter_gpio_change_t _gpio_change = nullptr;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_SHUTTER_H__