// Lesson 33 (module 3.3): PWM. Two potentiometers, two independent PWM outputs.
//  - pot 1 -> LED brightness   (LEDC timer 0,  5 kHz, 10 bit, channel 0)
//  - pot 2 -> DC motor speed   (LEDC timer 1, 20 kHz, 10 bit, channel 1) via N-MOSFET
// Separate timers = separate frequencies; separate channels = separate duty.
// Changing one never touches the other.
//
// Motor wiring (real board, see docs/hw33-pwm-wiring.md):
//   +V_motor -> motor -> MOSFET drain; source -> GND; gate <- 1 kOhm <- GPIO5
//   1N4007 across the motor (cathode to +V) against back-EMF; common GND with the ESP32.
#include <Arduino.h>
#include <driver/ledc.h>

struct Config {
    static constexpr uint8_t  LED_POT_PIN   = 1;     // ADC1_CH0
    static constexpr uint8_t  MOTOR_POT_PIN = 2;     // ADC1_CH1
    static constexpr uint8_t  LED_PWM_PIN   = 4;
    static constexpr uint8_t  MOTOR_PWM_PIN = 5;     // -> MOSFET gate
    static constexpr uint32_t LED_FREQ_HZ   = 5000;  // no visible flicker
    static constexpr uint32_t MOTOR_FREQ_HZ = 20000; // above hearing, no whine
    static constexpr ledc_timer_bit_t RES   = LEDC_TIMER_10_BIT;  // 0..1023
    static constexpr uint32_t DUTY_MAX      = (1U << 10) - 1;
    static constexpr uint32_t UPDATE_MS     = 50;
    static constexpr uint32_t LOG_MS        = 500;
    static constexpr uint32_t DEADBAND      = 4;     // ignore tiny pot jitter
};

// One PWM output = its own LEDC timer + channel.
class PwmOutput {
public:
    PwmOutput(uint8_t pin, ledc_timer_t timer, ledc_channel_t channel, uint32_t freqHz)
        : _pin(pin), _timer(timer), _channel(channel), _freq(freqHz) {}

    void init() {
        ledc_timer_config_t t = {};
        t.speed_mode      = LEDC_LOW_SPEED_MODE;
        t.timer_num       = _timer;
        t.duty_resolution = Config::RES;
        t.freq_hz         = _freq;            // freq * 2^res must stay <= 80 MHz
        t.clk_cfg         = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));

        ledc_channel_config_t c = {};
        c.speed_mode = LEDC_LOW_SPEED_MODE;
        c.channel    = _channel;
        c.timer_sel  = _timer;
        c.gpio_num   = _pin;
        c.duty       = 0;
        c.hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }

    // Writes only when the duty really changed.
    void setDuty(uint32_t duty) {
        if (duty > Config::DUTY_MAX) duty = Config::DUTY_MAX;
        const uint32_t diff = duty > _duty ? duty - _duty : _duty - duty;
        if (diff < Config::DEADBAND && duty != 0 && duty != Config::DUTY_MAX) return;
        _duty = duty;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, _duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel));
    }

    uint32_t duty() const { return _duty; }
    uint32_t percent() const { return _duty * 100 / Config::DUTY_MAX; }

private:
    const uint8_t        _pin;
    const ledc_timer_t   _timer;
    const ledc_channel_t _channel;
    const uint32_t       _freq;
    uint32_t             _duty = 0;
};

static PwmOutput led(Config::LED_PWM_PIN, LEDC_TIMER_0, LEDC_CHANNEL_0, Config::LED_FREQ_HZ);
static PwmOutput motor(Config::MOTOR_PWM_PIN, LEDC_TIMER_1, LEDC_CHANNEL_1, Config::MOTOR_FREQ_HZ);

// ADC 0..4095 -> duty 0..1023
static uint32_t potToDuty(uint8_t pin) {
    return map(analogRead(pin), 0, 4095, 0, Config::DUTY_MAX);
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    led.init();
    motor.init();
    Serial.printf("LED: %lu Hz, motor: %lu Hz, 10-bit duty\n",
                  (unsigned long)Config::LED_FREQ_HZ, (unsigned long)Config::MOTOR_FREQ_HZ);
}

void loop() {
    static uint32_t lastUpdate = 0, lastLog = 0;
    const uint32_t now = millis();

    if (now - lastUpdate >= Config::UPDATE_MS) {
        lastUpdate = now;
        led.setDuty(potToDuty(Config::LED_POT_PIN));
        motor.setDuty(potToDuty(Config::MOTOR_POT_PIN));
    }

    if (now - lastLog >= Config::LOG_MS) {
        lastLog = now;
        Serial.printf("LED %4lu (%3lu%%)   motor %4lu (%3lu%%)\n",
                      (unsigned long)led.duty(), (unsigned long)led.percent(),
                      (unsigned long)motor.duty(), (unsigned long)motor.percent());
    }

    delay(1);
}
