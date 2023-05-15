#include "reShutter.h"
#include <string.h>
#include "reEvents.h"
#include "reMqtt.h"
#include "reEsp32.h"
#include "rLog.h"
#include "rStrings.h"

static const char* logTAG = "SHTR";

#define ERR_SHUTTER_CHECK(err, str) if (err != ESP_OK) { rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err)); return false; };
#define ERR_GPIO_SET_LEVEL "Failed to change GPIO level"
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ rShutter -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rShutter::rShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
  uint8_t max_steps, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
  cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish)
{
  _pin_open = pin_open;
  _level_open = level_open;
  _pin_close = pin_close;
  _level_close = level_close;
  _max_steps = max_steps;
  _step_time = step_time;
  _step_time_adj = step_time_adj;
  _step_time_fin = step_time_fin;
  _on_changed = cb_state_changed;
  _mqtt_publish = cb_mqtt_publish;

  _state = 0;
  _changed = 0;
  _mqtt_topic = nullptr;
  _timer = nullptr;
}

rShutter::~rShutter()
{
  timerStop();
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- GPIO --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rShutter::Init()
{
  return gpioInit() 
      && gpioSetLevel(_pin_open, !level_open) 
      && gpioSetLevel(_pin_close, !level_close)
      && Close(0, true);
}

uint8_t rShutter::getState()
{
  return _state;
}

bool rShutter::Open(uint8_t steps)
{

}

bool rShutter::Close(uint8_t steps, bool forced)
{
  
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Timer --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void shutterTimerEnd(void* arg)
{
  if (arg) {
    rShutter* shutter = (rShutter*)arg;
    shutter->gpioSetLevel(_pin_open, !level_open);
    shutter->gpioSetLevel(_pin_close, !level_close);
    shutter->timerStop();
  };
}

bool rShutter::timerActivate(uint8_t pin, bool level, uint32_t duration_ms)
{
  if (_timer == nullptr) {
    esp_timer_create_args_t cfg;
    memset(&cfg, 0, sizeof(esp_timer_create_args_t));
    cfg.name = "shutter";
    cfg.callback = shutterTimerEnd;
    cfg.arg = this;
    RE_OK_CHECK(esp_timer_create(&cfg, &_timer), return false);
    if (_timer != nullptr) {
      RE_OK_CHECK(esp_timer_start_once(_timer, duration_ms*1000), return false);
      if (gpioSetLevel(pin, level)) {
        return true;
      } else {
        esp_timer_stop(_timer);
        esp_timer_delete(_timer);
        _timer = nullptr;
      };
    };
  };
  return false;
}

bool rShutter::timerIsActive()
{
  return (_timer != nullptr) && esp_timer_is_active(_timer);
}

bool rShutter::timerStop()
{
  if (_timer != nullptr) {
    if (esp_timer_is_active(_timer)) {
      esp_timer_stop(_timer);
    };
    RE_OK_CHECK(esp_timer_delete(_timer), return false);
    _timer = nullptr;
  };
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- MQTT ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rShutter::mqttSetCallback(cb_load_publish_t cb_publish)
{
  _mqtt_publish = cb_publish;
}

char* rShutter::mqttTopicGet()
{
  return _mqtt_topic;
}

bool rShutter::mqttTopicSet(char* topic)
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = topic;
  return (_mqtt_topic != nullptr);
}

bool rShutter::mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3)
{
  return mqttTopicSet(mqttGetTopicDevice(primary, local, topic1, topic2, topic3));
}

void rShutter::mqttTopicFree()
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

bool rShutter::mqttPublish()
{
  if ((_mqtt_topic) && (_mqtt_publish)) {
    return _mqtt_publish(this, _mqtt_topic, getJSON(), false, true);
  };
  return false;
}

char* rShutter::getJSON()
{
  return nullptr;
}
