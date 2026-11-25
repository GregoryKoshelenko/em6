// Lesson 35 (module 3.5): servo follows a potentiometer 1:1.
//  - potentiometer: ~270 deg mechanical travel, servo: 0..180 deg
//  - 1:1 means "turn the knob by X deg -> the shaft turns by X deg", so only the
//    overlapping 0..180 deg of the knob is used; the rest of the knob travel is cut off
//  - the angle from the leftmost position is logged when it changes
// Servo PWM: 50 Hz, pulse 500..2500 us for 0..180 deg (LEDC, 14-bit duty).
#include <Arduino.h>
#include <driver/ledc.h>

struct Config {
    static constexpr uint8_t  POT_PIN        = 1;      // ADC1_CH0
    static constexpr uint8_t  SERVO_PIN      = 14;
    static constexpr int      POT_TRAVEL_DEG = 270;    // full knob rotation
    static constexpr int      POT_FULL_MV    = 3300;   // knob at the right end
    static constexpr int      SERVO_MAX_DEG  = 180;    // usable overlap: 0..180
    static constexpr int      SERVO_MIN_US   = 500;
    static constexpr int      SERVO_MAX_US   = 2500;
    static constexpr uint32_t SERVO_HZ       = 50;     // 20 ms period
    static constexpr ledc_timer_bit_t RES    = LEDC_TIMER_14_BIT;
    static constexpr uint32_t DUTY_MAX       = (1U << 14) - 1;
    static constexpr uint32_t PERIOD_US      = 1000000UL / SERVO_HZ;
    static constexpr uint8_t  SMA_WINDOW     = 8;
    static constexpr uint32_t SAMPLE_MS      = 20;
};

static int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------- Potentiometer -> knob angle ----------
class Knob {
public:
    // Angle of the knob from its leftmost position, 0..270 deg (filtered).
    int angleDeg() {
        const int mv = static_cast<int>(analogReadMilliVolts(Config::POT_PIN));  // calibrated mV
        _sum += mv - _buf[_idx];
        _buf[_idx] = mv;
        _idx = (_idx + 1) % Config::SMA_WINDOW;
        if (_count < Config::SMA_WINDOW) ++_count;
        const int avgMv = _sum / _count;
        return clampInt(avgMv * Config::POT_TRAVEL_DEG / Config::POT_FULL_MV, 0, Config::POT_TRAVEL_DEG);
    }
private:
    int     _buf[Config::SMA_WINDOW] = {};
    int32_t _sum = 0;
    uint8_t _idx = 0, _count = 0;
};

// ---------- Servo on LEDC ----------
class Servo {
public:
    void init() {
        ledc_timer_config_t t = {};
        t.speed_mode      = LEDC_LOW_SPEED_MODE;
        t.timer_num       = LEDC_TIMER_0;
        t.duty_resolution = Config::RES;      // 50 Hz * 2^14 = 0.8 MHz, well under 80 MHz
        t.freq_hz         = Config::SERVO_HZ;
        t.clk_cfg         = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));

        ledc_channel_config_t c = {};
        c.speed_mode = LEDC_LOW_SPEED_MODE;
        c.channel    = LEDC_CHANNEL_0;
        c.timer_sel  = LEDC_TIMER_0;
        c.gpio_num   = Config::SERVO_PIN;
        c.duty       = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }

    // deg 0..180 -> pulse 500..2500 us -> duty
    uint32_t setAngle(int deg) {
        deg = clampInt(deg, 0, Config::SERVO_MAX_DEG);
        const int us = Config::SERVO_MIN_US
                     + deg * (Config::SERVO_MAX_US - Config::SERVO_MIN_US) / Config::SERVO_MAX_DEG;
        const uint32_t duty = static_cast<uint32_t>(us) * Config::DUTY_MAX / Config::PERIOD_US;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        return duty;
    }
};

static Knob  knob;
static Servo servo;

void setup() {
    Serial.begin(115200);
    analogSetPinAttenuation(Config::POT_PIN, ADC_11db);
    servo.init();
    Serial.printf("Servo follows the knob 1:1. Knob %d deg, servo 0..%d deg; knob above %d deg is cut.\n",
                  Config::POT_TRAVEL_DEG, Config::SERVO_MAX_DEG, Config::SERVO_MAX_DEG);
}

void loop() {
    static uint32_t lastSample = 0;
    static int lastDeg = -1;
    const uint32_t now = millis();
    if (now - lastSample < Config::SAMPLE_MS) { delay(1); return; }
    lastSample = now;

    const int knobDeg  = knob.angleDeg();
    const int servoDeg = clampInt(knobDeg, 0, Config::SERVO_MAX_DEG);   // 1:1, cut above 180
    if (servoDeg == lastDeg) return;                                     // log only changes
    lastDeg = servoDeg;

    const uint32_t duty = servo.setAngle(servoDeg);
    Serial.printf("angle from left: %3d deg (knob %3d deg%s)  duty %lu\n",
                  servoDeg, knobDeg, knobDeg > Config::SERVO_MAX_DEG ? ", cut" : "",
                  (unsigned long)duty);
}
