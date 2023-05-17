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
typedef bool (*cb_shutter_gpio_init_t) (rShutter *shutter, uint8_t pin, bool level_active);
typedef bool (*cb_shutter_gpio_change_t) (rShutter *shutter, uint8_t pin, bool physical_level);

class rShutter {
  public:
    rShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
      uint8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
      cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish);
    ~rShutter();

    // Current state
    uint8_t getState();
    char* getJSON();

    bool Init();
    // Open or close the shutter by a specified number of steps
    bool Open(uint8_t steps);
    bool Close(uint8_t steps);
    // Forced close on initialization or crash
    bool CloseForced();

    // For the timer handler
    bool StopAndQueueProcessing();

    // MQTT
    void mqttSetCallback(cb_shutter_publish_t cb_publish);
    char* mqttTopicGet();
    bool mqttTopicSet(char* topic);
    bool mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3);
    void mqttTopicFree();
    bool mqttPublish();
  protected:
    uint8_t     _pin_open = 0;                    // Pin number to open
    bool        _level_open = true;               // Logic level to open
    uint8_t     _pin_close = 0;                   // Pin number to close
    bool        _level_close = true;              // Logic level to close

    virtual bool gpioInit() = 0;
    virtual bool gpioSetLevel(uint8_t pin, bool physical_level) = 0; 
  private:
    uint32_t            _full_time = 15000;       // Full closing time in milliseconds
    uint8_t             _max_steps = 10;          // Number of steps to fully open
    uint32_t            _step_time = 1000;        // Time delay by one step in milliseconds
    float               _step_time_adj = 1.00;    // Delay adjustment factor for each next step
    uint32_t            _step_time_fin = 0;       // Finishing time
    uint8_t             _state = 0;               // Current state
    uint8_t             _queue_open = 0;          // Number of scheduled steps if the timer is active
    uint8_t             _queue_close = 0;         // Number of scheduled steps if the timer is active
    time_t              _changed = 0;             // Time of last state change
    esp_timer_handle_t  _timer = nullptr;         // Step timer
    char*               _mqtt_topic = nullptr;    // MQTT topic

    cb_shutter_change_t  _on_changed = nullptr;   // Pointer to the callback function to be called after load switching
    cb_shutter_publish_t _mqtt_publish = nullptr; // Pointer to the publish callback function

    uint32_t calcStepTimeout(uint8_t step);

    // Disable all drives
    bool StopAll();

    // Timer
    bool timerCreate();
    bool timerFree();
    bool timerActivate(uint8_t pin, bool level, uint32_t duration_ms);
    bool timerIsActive();
    bool timerStop();
};

#ifdef __cplusplus
}
#endif

#endif // __RE_SHUTTER_H__