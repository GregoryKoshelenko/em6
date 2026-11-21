// Lesson 37 (module 3) mini-project: servo controlled by a rotary encoder.
// Refactors the lesson's LDR -> servo code (LEDC driver, clamp/map, duty math)
// and replaces the LDR input with a KY-040 encoder.
#include <Arduino.h>
#include <driver/ledc.h>

struct Config {
    static constexpr uint8_t ENC_CLK_PIN = 4;
    static constexpr uint8_t ENC_DT_PIN  = 5;
    static constexpr uint8_t ENC_SW_PIN  = 6;
    static constexpr uint8_t SERVO_PIN   = 14;
    static constexpr uint8_t BUZZER_PIN  = 7;

    static constexpr int SERVO_MIN_DEG = 0;
    static constexpr int SERVO_MAX_DEG = 180;
    static constexpr int SERVO_MIN_US  = 500;
    static constexpr int SERVO_MAX_US  = 2500;
    static constexpr uint32_t SERVO_FREQ_HZ = 50;

    static constexpr int STEP_COARSE_DEG = 10;
    static constexpr int STEP_FINE_DEG   = STEP_COARSE_DEG / 2;

    static constexpr uint32_t BEEP_FREQ_HZ = 2000;
    static constexpr uint32_t BEEP_MS      = 80;

    static constexpr uint32_t DEBOUNCE_MS   = 30;
    static constexpr uint32_t LONG_PRESS_MS = 1000;
    static constexpr uint32_t LOOP_IDLE_MS  = 5;
    static constexpr uint32_t SERIAL_BAUD   = 115200;
};

static constexpr int clampValue(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static constexpr int mapRange(int v, int inMin, int inMax, int outMin, int outMax) {
    return inMin == inMax
        ? outMax
        : outMin + (clampValue(v, inMin, inMax) - inMin) * (outMax - outMin) / (inMax - inMin);
}

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

    void setAngle(int deg) {
        _deg = clampValue(deg, Config::SERVO_MIN_DEG, Config::SERVO_MAX_DEG);
        const int us = mapRange(_deg, Config::SERVO_MIN_DEG, Config::SERVO_MAX_DEG,
                                Config::SERVO_MIN_US, Config::SERVO_MAX_US);
        const uint32_t duty = static_cast<uint32_t>(us) * MAX_DUTY / PERIOD_US;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel));
    }

    int angle() const { return _deg; }

private:
    const uint8_t        _pin;
    const ledc_timer_t   _timer;
    const ledc_channel_t _channel;
    int _deg = 0;
};

// Passive buzzer on its own LEDC timer; beep() is non-blocking.
class Buzzer {
public:
    Buzzer(uint8_t pin, ledc_timer_t timer, ledc_channel_t channel)
        : _pin(pin), _timer(timer), _channel(channel) {}

    void init() {
        ledc_timer_config_t timerCfg = {};
        timerCfg.speed_mode      = LEDC_LOW_SPEED_MODE;
        timerCfg.timer_num       = _timer;
        timerCfg.duty_resolution = LEDC_TIMER_10_BIT;
        timerCfg.freq_hz         = Config::BEEP_FREQ_HZ;
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

    void beep(uint32_t now) {
        setDuty(HALF_DUTY);
        _until = now + Config::BEEP_MS;
        _on = true;
    }

    void tick(uint32_t now) {
        if (_on && static_cast<int32_t>(now - _until) >= 0) {
            setDuty(0);
            _on = false;
        }
    }

private:
    static constexpr uint32_t HALF_DUTY = 512;  // 50 % of 10-bit

    void setDuty(uint32_t duty) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

    const uint8_t        _pin;
    const ledc_timer_t   _timer;
    const ledc_channel_t _channel;
    uint32_t _until = 0;
    bool     _on    = false;
};

// KY-040: ISR on CLK falling edge counts ticks; DT gives the direction.
static volatile int32_t gEncoderTicks = 0;

static void IRAM_ATTR onEncoderClk() {
    gEncoderTicks += digitalRead(Config::ENC_DT_PIN) == HIGH ? 1 : -1;
}

static int32_t takeEncoderTicks() {
    noInterrupts();
    const int32_t ticks = gEncoderTicks;
    gEncoderTicks = 0;
    interrupts();
    return ticks;
}

// Debounced button with short-press and long-press events.
class Button {
public:
    enum class Event : uint8_t { None, Short, Long };

    explicit Button(uint8_t pin) : _pin(pin) {}
    void init() { pinMode(_pin, INPUT_PULLUP); }

    Event poll(uint32_t now) {
        const bool raw = digitalRead(_pin) == LOW;
        if (raw != _lastRaw) { _lastRaw = raw; _changedAt = now; }
        if (now - _changedAt < Config::DEBOUNCE_MS) return Event::None;

        Event ev = Event::None;
        if (raw && !_pressed) {
            _pressed = true; _pressedAt = now; _longFired = false;
        } else if (raw && _pressed && !_longFired && now - _pressedAt >= Config::LONG_PRESS_MS) {
            _longFired = true; ev = Event::Long;
        } else if (!raw && _pressed) {
            _pressed = false;
            if (!_longFired) ev = Event::Short;
        }
        return ev;
    }

private:
    const uint8_t _pin;
    bool     _lastRaw   = false;
    bool     _pressed   = false;
    bool     _longFired = false;
    uint32_t _changedAt = 0;
    uint32_t _pressedAt = 0;
};

static Servo  servo(Config::SERVO_PIN, LEDC_TIMER_0, LEDC_CHANNEL_0);
static Buzzer buzzer(Config::BUZZER_PIN, LEDC_TIMER_1, LEDC_CHANNEL_1);
static Button encButton(Config::ENC_SW_PIN);
static int    stepDeg = Config::STEP_COARSE_DEG;

static void logState(const char* what) {
    Serial.printf("%-8s angle=%3d deg  step=%2d deg\n", what, servo.angle(), stepDeg);
}

void setup() {
    Serial.begin(Config::SERIAL_BAUD);

    pinMode(Config::ENC_CLK_PIN, INPUT_PULLUP);
    pinMode(Config::ENC_DT_PIN,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Config::ENC_CLK_PIN), onEncoderClk, FALLING);
    encButton.init();

    servo.init();
    buzzer.init();

    constexpr int center = (Config::SERVO_MIN_DEG + Config::SERVO_MAX_DEG) / 2;
    servo.setAngle(center);

    Serial.println("=== Servo + encoder (lesson 37) ===");
    Serial.println("Rotate: move | Click: coarse/fine step | Hold 1 s: center");
    logState("start");
}

void loop() {
    const uint32_t now = millis();

    const int32_t ticks = takeEncoderTicks();
    if (ticks != 0) {
        const int target = servo.angle() + static_cast<int>(ticks) * stepDeg;
        if (target < Config::SERVO_MIN_DEG || target > Config::SERVO_MAX_DEG) {
            buzzer.beep(now);
            servo.setAngle(target);  // clamps to the limit
            logState("LIMIT");
        } else {
            servo.setAngle(target);
            logState(ticks > 0 ? "CW" : "CCW");
        }
    }

    switch (encButton.poll(now)) {
        case Button::Event::Short:
            stepDeg = stepDeg == Config::STEP_COARSE_DEG ? Config::STEP_FINE_DEG
                                                         : Config::STEP_COARSE_DEG;
            logState("STEP");
            break;
        case Button::Event::Long:
            servo.setAngle((Config::SERVO_MIN_DEG + Config::SERVO_MAX_DEG) / 2);
            logState("CENTER");
            break;
        case Button::Event::None:
            break;
    }

    buzzer.tick(now);
    delay(Config::LOOP_IDLE_MS);
}
