// Lesson 38 (module 3): refactor of the lesson's mini-project.
// Light sensor (LDR) -> servo angle, with a simple moving average (SMA) filter.
#include <Arduino.h>
#include <driver/ledc.h>

struct Config {
    static constexpr uint8_t LDR_PIN   = 9;    // ADC1_CH8
    static constexpr uint8_t SERVO_PIN = 14;

    // Calibration
    static constexpr int LIGHT_MIN_MV  = 50;
    static constexpr int LIGHT_MAX_MV  = 3000;
    static constexpr int SERVO_MIN_DEG = 10;
    static constexpr int SERVO_MAX_DEG = 170;
    static constexpr int SERVO_MIN_US  = 500;
    static constexpr int SERVO_MAX_US  = 2500;
    static constexpr uint32_t SERVO_FREQ_HZ = 50;

    static constexpr uint8_t  SMA_WINDOW   = 8;     // samples
    static constexpr uint32_t SAMPLE_MS    = 100;   // superloop period
    static constexpr uint32_t SERIAL_BAUD  = 115200;
};

static constexpr int clampValue(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static constexpr int mapRange(int v, int inMin, int inMax, int outMin, int outMax) {
    return inMin == inMax
        ? outMax
        : outMin + (clampValue(v, inMin, inMax) - inMin) * (outMax - outMin) / (inMax - inMin);
}

// Simple moving average over the last SMA_WINDOW samples (fixed buffer, no heap).
class MovingAverage {
public:
    int add(int sample) {
        _sum += sample - _buf[_idx];
        _buf[_idx] = sample;
        _idx = (_idx + 1) % Config::SMA_WINDOW;
        if (_count < Config::SMA_WINDOW) ++_count;
        return static_cast<int>(_sum / _count);
    }

private:
    int     _buf[Config::SMA_WINDOW] = {};
    int32_t _sum   = 0;
    uint8_t _idx   = 0;
    uint8_t _count = 0;
};

// LDR on an ADC pin; analogReadMilliVolts() returns the calibrated voltage.
class LightSensor {
public:
    explicit LightSensor(uint8_t pin) : _pin(pin) {}

    void init() {
        analogReadResolution(12);
        analogSetPinAttenuation(_pin, ADC_11db);  // full 0..3.1 V range
    }

    int readMv() const { return static_cast<int>(analogReadMilliVolts(_pin)); }

private:
    const uint8_t _pin;
};

// Servo on an LEDC channel: angle -> pulse width -> duty.
class Servo {
public:
    static constexpr ledc_timer_bit_t RESOLUTION = LEDC_TIMER_12_BIT;
    static constexpr uint32_t MAX_DUTY  = (1U << RESOLUTION) - 1;
    static constexpr uint32_t PERIOD_US = 1000000UL / Config::SERVO_FREQ_HZ;

    Servo(uint8_t pin, ledc_timer_t timer, ledc_channel_t channel)
        : _pin(pin), _timer(timer), _channel(channel) {}

    void init() {
        ledc_timer_config_t timerCfg = {};
        timerCfg.speed_mode      = LEDC_LOW_SPEED_MODE;
        timerCfg.timer_num       = _timer;
        timerCfg.duty_resolution = RESOLUTION;
        timerCfg.freq_hz         = Config::SERVO_FREQ_HZ;
        timerCfg.clk_cfg         = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timerCfg));

        ledc_channel_config_t chCfg = {};
        chCfg.speed_mode = LEDC_LOW_SPEED_MODE;
        chCfg.channel    = _channel;
        chCfg.timer_sel  = _timer;
        chCfg.intr_type  = LEDC_INTR_DISABLE;
        chCfg.gpio_num   = _pin;
        chCfg.duty       = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&chCfg));
    }

    // Returns the duty that was written, for logging.
    uint32_t setAngle(int deg) {
        _deg = clampValue(deg, Config::SERVO_MIN_DEG, Config::SERVO_MAX_DEG);
        const int us = mapRange(_deg, Config::SERVO_MIN_DEG, Config::SERVO_MAX_DEG,
                                Config::SERVO_MIN_US, Config::SERVO_MAX_US);
        const uint32_t duty = static_cast<uint32_t>(us) * MAX_DUTY / PERIOD_US;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel));
        return duty;
    }

private:
    const uint8_t        _pin;
    const ledc_timer_t   _timer;
    const ledc_channel_t _channel;
    int _deg = 0;
};

static LightSensor   ldr(Config::LDR_PIN);
static MovingAverage lightFilter;
static Servo         servo(Config::SERVO_PIN, LEDC_TIMER_0, LEDC_CHANNEL_0);

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    ldr.init();
    servo.init();
    Serial.println("=== LDR -> servo (lesson 38 refactor) ===");
}

void loop() {
    static uint32_t lastSample = 0;
    const uint32_t now = millis();
    if (now - lastSample < Config::SAMPLE_MS) {
        delay(1);  // idle instead of busy-spinning
        return;
    }
    lastSample = now;

    const int rawMv = ldr.readMv();
    const int avgMv = lightFilter.add(rawMv);
    const int deg   = mapRange(avgMv, Config::LIGHT_MIN_MV, Config::LIGHT_MAX_MV,
                               Config::SERVO_MIN_DEG, Config::SERVO_MAX_DEG);
    const uint32_t duty = servo.setAngle(deg);

    Serial.printf("light %4d mV  sma %4d mV  ->  %3d deg  duty %3lu\n",
                  rawMv, avgMv, deg, static_cast<unsigned long>(duty));
}
