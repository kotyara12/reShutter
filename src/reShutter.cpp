#include "reShutter.h"
#include <string.h>
#include "reEvents.h"
#include "reMqtt.h"
#include "reEsp32.h"
#include "rLog.h"
#include "rStrings.h"

#if CONFIG_RLOG_PROJECT_LEVEL > RLOG_LEVEL_NONE
static const char* logTAG = "SHTR";
#endif // CONFIG_RLOG_PROJECT_LEVEL

#define ERR_SHUTTER_CHECK(err, str) if (err != ESP_OK) { rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err)); return false; };
#define ERR_GPIO_SET_LEVEL "Failed to change GPIO level"
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ rShutter -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rShutter::rShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
  uint8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
  cb_shutter_gpio_wrap_t cb_gpio_before, cb_shutter_gpio_wrap_t cb_gpio_after, cb_shutter_timer_t cb_timer, 
  cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish)
{
  _pin_open = pin_open;
  _level_open = level_open;
  _pin_close = pin_close;
  _level_close = level_close;
  _full_time = full_time;
  _max_steps = max_steps;
  _limit_min = 0;
  _limit_max = _max_steps;
  _step_time = step_time;
  _step_time_adj = step_time_adj;
  _step_time_fin = step_time_fin;
  _on_before = cb_gpio_before;
  _on_after = cb_gpio_after;
  _on_timer = cb_timer;
  _on_changed = cb_state_changed;
  _mqtt_publish = cb_mqtt_publish;

  _state = 0;
  _last_changed = 0;
  _last_open = 0;
  _last_close = 0;
  _last_max_state = 0;
  _mqtt_topic = nullptr;
  _timer = nullptr;
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
  _last_changed = 0;
  _last_open = 0;
  _last_close = 0;
  _last_max_state = 0;
  _pin_open_state = 0;
  _pin_close_state = 0;
  return gpioInit() && timerCreate() && StopAll();
}

bool rShutter::gpioSetLevelPriv(uint8_t pin, bool physical_level)
{
  if (physical_level) {
    if (pin == _pin_open) {
      _pin_open_state = true;
    } else if (pin == _pin_close) {
      _pin_close_state = true;
    };
    if (_on_timer) _on_timer(this, pin, true);
  };
  if (_on_before) _on_before(this, pin);
  bool ret = gpioSetLevel(pin, physical_level);
  if (_on_after) _on_after(this, pin);
  if (ret && !physical_level) {
    if (pin == _pin_open) {
      _pin_open_state = false;
    } else if (pin == _pin_close) {
      _pin_close_state = false;
    };
    if (_on_timer) _on_timer(this, pin, false);
  };
  return ret;
}

// Disable all drives
bool rShutter::StopAll()
{
  bool ret = true;
  if (_pin_open_state) {
    ret = gpioSetLevelPriv(_pin_open, !_level_open);
  };
  if (ret && _pin_close_state) {
    ret = gpioSetLevelPriv(_pin_close, !_level_close);
  };
  return ret;
}

// Current state
uint8_t rShutter::getState()
{
  return _state;
}

uint8_t rShutter::getMaxSteps()
{
  return _max_steps;
}

float rShutter::getPercent()
{
  return (float)_state / _max_steps * 100.0;
}

bool rShutter::isFullOpen()
{
  if (_limit_max < _max_steps) {
    return _state >= _limit_max;
  } else {
    return _state >= _max_steps;
  };
}

bool rShutter::isFullClose()
{
  if (_limit_min > 0) {
    return _state <= _limit_min;
  } else {
    return _state == 0;
  };
}

time_t rShutter::getLastChange()
{
  return _last_changed;
}

uint32_t rShutter::calcStepTimeout(uint8_t step)
{
  float ret = (float)_step_time;
  if (step > 1) {
    for (uint8_t i = 2; i <= step; i++) {
      ret = ret * _step_time_adj;
    };
  };
  return (uint32_t)ret;
}

// Open the shutter by a specified number of steps
bool rShutter::OpenPriv(uint8_t steps)
{
  if (steps > 0) {
    if (timerIsActive()) {
      rlog_w(logTAG, "Drive busy, operation canceled");
    } else {
      // Calculate time
      uint32_t _duration = 0;
      for (uint8_t i = 1; i <= steps; i++) {
        _duration += calcStepTimeout(_state + i);
      };
      // Turn on the drive for the сalculated time
      if (timerActivate(_pin_open, _level_open, _duration)) {
        rlog_i(logTAG, "Open shutter %d steps ( %d milliseconds )", steps, _duration);
        _last_changed = time(nullptr);
        if (_state == 0) {
          _last_max_state = 0;
          _last_open = time(nullptr);
        };
        _state += steps;
        if (_state > _last_max_state) {
          _last_max_state = _state;
        };
        if (_on_changed) {
          _on_changed(this, _state - steps, _state, _max_steps);
        };
        return true;
      };
      rlog_e(logTAG, "Failed to activate shutter");
    };
  };
  return false;
}

// Close the shutter by a specified number of steps
bool rShutter::ClosePriv(uint8_t steps)
{
  if (steps > 0) {
    if (timerIsActive()) {
      rlog_w(logTAG, "Drive busy, operation canceled");
    } else {
      // Calculate time
      uint32_t _duration = 0;
      for (uint8_t i = steps; i > 0 ; i--) {
        _duration += calcStepTimeout(_state - i + 1);
        if (_state - i == 0) {
          _duration += _step_time_fin;
        }
      };
      // Turn on the drive for the сalculated time
      if (timerActivate(_pin_close, _level_close, _duration)) {
        rlog_i(logTAG, "Close shutter %d steps ( %d milliseconds )", steps, _duration);
        _last_changed = time(nullptr);
        _state -= steps;
        if (_state == 0) {
          _last_close = time(nullptr);
        };
        if (_on_changed) {
          _on_changed(this, _state + steps, _state, _max_steps);
        };
        return true;
      };
      rlog_e(logTAG, "Failed to activate shutter");
    };
  };
  return false;
}

bool rShutter::Open(uint8_t steps)
{
  int8_t _steps = checkLimits((int8_t)steps);
  if (_steps > 0) {
    return OpenPriv((uint8_t)_steps);
  } else if (_steps < 0) {
    return ClosePriv((uint8_t)(-_steps));
  };
  return false;
}

bool rShutter::Close(uint8_t steps)
{
  int8_t _steps = checkLimits(-(int8_t)steps);
  if (_steps < 0) {
    return ClosePriv((uint8_t)(-_steps));
  } else if (_steps > 0) {
    return OpenPriv((uint8_t)_steps);
  };
  return false;
}


bool rShutter::OpenFull()
{
  if (_state < _max_steps) {
    return Open(_max_steps - _state);
  };
  return false;
}

// Full closure without regard to steps (until the limit switches are activated)
bool rShutter::CloseFullEx(bool forced, bool call_cb)
{
  if ((_state > _limit_min) || forced) {
    if (_limit_min == 0) {
      Break();
      if (timerActivate(_pin_close, _level_close, _full_time)) {
        rlog_i(logTAG, "Сlose shutter completely");
        _last_changed = time(nullptr);
        _last_close = time(nullptr);
        if (call_cb && _on_changed) {
          _on_changed(this, _state, _limit_min, _max_steps);
        };
        _state = 0;
        return true;
      };
    } else {
      Close(_state - _limit_min);
    };
  };
  return false;
}

bool rShutter::CloseFull(bool forced)
{
  return CloseFullEx(forced, true);
}

bool rShutter::isBusy()
{
  return timerIsActive();
}

bool rShutter::Break()
{
  if (timerIsActive()) {
    return timerStop();
  };
  return false;
}

int8_t rShutter::checkLimits(int8_t steps)
{
  int8_t ret = steps;
  // Checking permanent limits
  if ((_state + ret) < 0) {
    ret = - _state;
  };
  if ((_state + ret) > _max_steps) {
    ret = _max_steps - _state;
  };
  // Checking non-permanent limits
  if ((_state + ret) < _limit_min) {
    ret = _limit_min - _state;
  };
  if ((_state + ret) > _limit_max) {
    ret = _limit_max - _state;
  };
  if (steps != ret) {
    rlog_w(logTAG, "Requested %d steps, actually %d steps will be completed", steps, ret);
  };
  return ret;
}

bool rShutter::setMinLimit(uint8_t limit)
{
  if (limit != _limit_min) {
    _limit_min = limit;
    if (_state < _limit_min) {
      return Open(_limit_min - _state);
    };
  };
  return false;
}

bool rShutter::setMaxLimit(uint8_t limit)
{
  if (limit != _limit_max) {
    if (limit <= _max_steps) {
      _limit_max = limit;
    } else {
      _limit_max = _max_steps;
    };
    if (_state > _limit_max) {
      return Close(_state - _limit_max);
    };
  };
  return false;
}

bool rShutter::clearMinLimit()
{
  return setMinLimit(0);
}

bool rShutter::clearMaxLimit()
{
  return setMaxLimit(_max_steps);
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Timer --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void shutterTimerEnd(void* arg)
{
  if (arg) {
    rShutter* shutter = (rShutter*)arg;
    shutter->StopAll();
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
    if (gpioSetLevelPriv(pin, level)) {
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
      RE_OK_CHECK(esp_timer_stop(_timer), return false);
    };
  };
  return StopAll();
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

char* rShutter::getStateJSON(uint8_t state)
{
  return malloc_stringf("{\"" CONFIG_SHUTTER_VALUE "\":%d,\"" CONFIG_SHUTTER_PERCENT "\":%.1f}", state, (float)state / _max_steps * 100.0);
}

char* rShutter::getTimestampsJSON()
{
  char _time_changed[CONFIG_SHUTTER_TIMESTAMP_BUF_SIZE];
  char _time_open[CONFIG_SHUTTER_TIMESTAMP_BUF_SIZE];
  char _time_close[CONFIG_SHUTTER_TIMESTAMP_BUF_SIZE];

  time2str_empty( CONFIG_SHUTTER_TIMESTAMP_FORMAT, &_last_changed, &_time_changed[0], sizeof(_time_changed));
  time2str_empty( CONFIG_SHUTTER_TIMESTAMP_FORMAT, &_last_open, &_time_open[0], sizeof(_time_open));
  time2str_empty( CONFIG_SHUTTER_TIMESTAMP_FORMAT, &_last_close, &_time_close[0], sizeof(_time_close));

  return malloc_stringf("{\"" CONFIG_SHUTTER_CHANGED "\":\"%s\",\"" CONFIG_SHUTTER_OPEN "\":\"%s\",\"" CONFIG_SHUTTER_CLOSE "\":\"%s\"}", _time_changed, _time_open, _time_close);
}

char* rShutter::getJSON()
{
  char* _json = nullptr;
  
  char* _json_stat = getStateJSON(_state);
  char* _json_smax = getStateJSON(_last_max_state);
  char* _json_time = getTimestampsJSON();

  if (_json_stat && _json_smax && _json_time) {
    _json = malloc_stringf("{\"" CONFIG_SHUTTER_STATUS "\":%s,\"" CONFIG_SHUTTER_TIMESTAMP "\":%s,\"" CONFIG_SHUTTER_MAXIMUM "\":%s}", _json_stat, _json_time, _json_smax);
  };

  if (_json_stat) free(_json_stat);
  if (_json_smax) free(_json_smax);
  if (_json_time) free(_json_time);

  return _json;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- rGpioShutter -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rGpioShutter::rGpioShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
  uint8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
  cb_shutter_gpio_wrap_t cb_gpio_before, cb_shutter_gpio_wrap_t cb_gpio_after, cb_shutter_timer_t cb_timer, 
  cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish)
:rShutter(pin_open, level_open, pin_close, level_close, 
  max_steps, full_time, step_time, step_time_adj, step_time_fin,
  cb_gpio_before, cb_gpio_after, cb_timer, cb_state_changed, cb_mqtt_publish)
{
}

bool rGpioShutter::gpioInit()
{
  // Configure internal GPIO to output
  gpio_reset_pin((gpio_num_t)_pin_open);
  gpio_reset_pin((gpio_num_t)_pin_close);
  ERR_SHUTTER_CHECK(gpio_set_direction((gpio_num_t)_pin_open, GPIO_MODE_OUTPUT), ERR_GPIO_SET_MODE);
  ERR_SHUTTER_CHECK(gpio_set_direction((gpio_num_t)_pin_close, GPIO_MODE_OUTPUT), ERR_GPIO_SET_MODE);
  return true;
}

bool rGpioShutter::gpioSetLevel(uint8_t pin, bool physical_level)
{
  ERR_SHUTTER_CHECK(gpio_set_level((gpio_num_t)pin, (uint32_t)physical_level), ERR_GPIO_SET_LEVEL);
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- rIoExtShutter -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rIoExpShutter::rIoExpShutter(uint8_t pin_open, bool level_open, uint8_t pin_close, bool level_close, 
  uint8_t max_steps, uint32_t full_time, uint32_t step_time, float step_time_adj, uint32_t step_time_fin,
  cb_shutter_gpio_init_t cb_gpio_init, cb_shutter_gpio_change_t cb_gpio_change,
  cb_shutter_gpio_wrap_t cb_gpio_before, cb_shutter_gpio_wrap_t cb_gpio_after, cb_shutter_timer_t cb_timer, 
  cb_shutter_change_t cb_state_changed, cb_shutter_publish_t cb_mqtt_publish)
:rShutter(pin_open, level_open, pin_close, level_close, 
  max_steps, full_time, step_time, step_time_adj, step_time_fin,
  cb_gpio_before, cb_gpio_after, cb_timer, cb_state_changed, cb_mqtt_publish)
{
  _gpio_init = cb_gpio_init;
  _gpio_change = cb_gpio_change;
}

bool rIoExpShutter::gpioInit()
{
  if (_gpio_init) {
    return _gpio_init(this, _pin_open, _level_open) && _gpio_init(this, _pin_close, _level_close);
  };
  return true;
}

bool rIoExpShutter::gpioSetLevel(uint8_t pin, bool physical_level)
{
  if (_gpio_change) {
    return _gpio_change(this, pin, physical_level);
  };
  return true;
}
