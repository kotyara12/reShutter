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
  uint8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
  cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish)
{
  _pin_open = pin_open;
  _level_open = level_open;
  _pin_close = pin_close;
  _level_close = level_close;
  _full_time = full_time;
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

  timerCreate();
}

rShutter::~rShutter()
{
  timerFree();
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- GPIO --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rShutter::Init()
{
  return gpioInit() 
      && StopAll()
      && CloseForced();
}

// Disable all drives
bool rShutter::StopAll()
{
  return gpioSetLevel(_pin_open, !_level_open) && gpioSetLevel(_pin_close, !_level_close);
}

// At the end of the timer, we execute the next steps, if any
bool rShutter::StopAndQueueProcessing()
{
  bool ret = StopAll();
  if (ret) {
    if (_queue_open > 0) {
      ret = Open(_queue_open);
    } else if (_queue_close > 0) {
      ret = Close(_queue_close);
    };
    _queue_open = 0;
    _queue_close = 0;
  };
  return ret;
}


// Current state
uint8_t rShutter::getState()
{
  return _state;
}

uint32_t rShutter::calcStepTimeout(uint8_t step)
{
  uint32_t ret = _step_time;
  if (step > 1) {
    for (uint8_t i = 2; i <= step; i++) {
      ret = ret * _step_time_adj;
    };
  };
  return ret;
}

// Open the shutter by a specified number of steps
bool rShutter::Open(uint8_t steps)
{
  uint8_t _steps = steps;
  if (_steps + _state > _max_steps) {
    _steps = _max_steps - _state;
    rlog_w(logTAG, "Requested %d steps, actually %d steps will be completed", steps, _steps);
  };
  if (_steps > 0) {
    if (timerIsActive()) {
      _queue_open += _steps;
      rlog_w(logTAG, "An operation is currently in progress, the requested steps are queued");
    } else {
      // Calculate time
      uint32_t _duration = 0;
      for (uint8_t i = 1; i <= _steps; i++) {
        _duration += calcStepTimeout(_state + _steps);
      };
      // Turn on the drive for the сalculated time
      if (timerActivate(_pin_open, _level_open, _duration)) {
        rlog_i(logTAG, "Open shutter %d steps ( %d milliseconds )", _steps, _duration);
        if (_on_changed) {
          _on_changed(this, _state, _state + _steps, _max_steps);
        };
        _state += _steps;
        return true;
      };
      rlog_e(logTAG, "Failed to activate shutter");
    };
  };
  return false;
}

// Close the shutter by a specified number of steps
bool rShutter::Close(uint8_t steps)
{
  uint8_t _steps = steps;
  if (_steps > _state) {
    _steps = _state;
    rlog_w(logTAG, "Requested %d steps, actually %d steps will be completed", steps, _steps);
  };
  if (_steps > 0) {
    if (timerIsActive()) {
      _queue_close += _steps;
      rlog_w(logTAG, "An operation is currently in progress, the requested steps are queued");
    } else {
      // Calculate time
      uint32_t _duration = 0;
      for (uint8_t i = _steps; i > 0 ; i--) {
        _duration += calcStepTimeout(_state - _steps + 1);
      };
      // Turn on the drive for the сalculated time
      if (timerActivate(_pin_close, _level_close, _duration)) {
        rlog_i(logTAG, "Close shutter %d steps ( %d milliseconds )", _steps, _duration);
        if (_on_changed) {
          _on_changed(this, _state, _state - _steps, _max_steps);
        };
        _state -= _steps;
        return true;
      };
      rlog_e(logTAG, "Failed to activate shutter");
    };
  };
  return false;
}

// Forced close on initialization or crash
bool rShutter::CloseForced()
{
  timerStop();
  if (timerActivate(_pin_close, _level_close, _full_time)) {
    rlog_i(logTAG, "Сlose shutter completely");
    if (_on_changed) {
      _on_changed(this, _state, 0, _max_steps);
    };
    _state = 0;
    return true;
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Timer --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void shutterTimerEnd(void* arg)
{
  if (arg) {
    rShutter* shutter = (rShutter*)arg;
    shutter->StopAndQueueProcessing();
  };
}

bool rShutter::timerCreate()
{
  if (_timer == nullptr) {
    esp_timer_create_args_t cfg;
    memset(&cfg, 0, sizeof(esp_timer_create_args_t));
    cfg.name = "shutter";
    cfg.callback = shutterTimerEnd;
    cfg.arg = this;
    RE_OK_CHECK(esp_timer_create(&cfg, &_timer), return false);
  };
  return true;
}

bool rShutter::timerFree()
{
  if (_timer != nullptr) {
    timerStop();
    RE_OK_CHECK(esp_timer_delete(_timer), return false);
  };
  return true;
}

bool rShutter::timerActivate(uint8_t pin, bool level, uint32_t duration_ms)
{
  if (_timer == nullptr) {
    timerCreate();
  };
  if (_timer != nullptr) {
    RE_OK_CHECK(esp_timer_start_once(_timer, duration_ms*1000), return false);
    if (gpioSetLevel(pin, level)) {
      return true;
    } else {
      timerStop();
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
      StopAll();
      RE_OK_CHECK(esp_timer_stop(_timer), return false);
    };
  };
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- MQTT ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rShutter::mqttSetCallback(cb_shutter_publish_t cb_publish)
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
