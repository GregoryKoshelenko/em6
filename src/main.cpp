#include <Arduino.h>
#include <esp_sleep.h>

// ─────────────────────────────────────────────────────────────────────────────
// Config
// ─────────────────────────────────────────────────────────────────────────────

struct Config {
    static constexpr uint8_t  LED1_PIN         = 2;
    static constexpr uint8_t  LED2_PIN         = 21;
    static constexpr uint8_t  LED3_PIN         = 19;

    static constexpr unsigned long LED1_PERIOD_MS  = 200UL;
    static constexpr unsigned long LED2_PERIOD_MS  = 500UL;
    static constexpr unsigned long LED3_PERIOD_MS  = 1000UL;

    static constexpr unsigned long REPORT_EVERY_MS = 5000UL;
    static constexpr unsigned long MIN_SLEEP_MS    = 2UL;

    static constexpr float NORMAL_MW = 240.0f;
    static constexpr float SLEEP_MW  = 0.8f;

    static constexpr uint32_t SERIAL_BAUD = 115200UL;
};

// ─────────────────────────────────────────────────────────────────────────────
// Led
// ─────────────────────────────────────────────────────────────────────────────

enum class LedState : uint8_t { Off = LOW, On = HIGH };

class Led {
public:
    constexpr explicit Led(uint8_t pin)
        : _pin(pin), _state(LedState::Off) {}

    void init() {
        pinMode(_pin, OUTPUT);
        set(LedState::Off);
    }

    void set(LedState state) {
        _state = state;
        digitalWrite(_pin, static_cast<uint8_t>(state));
    }

    void toggle() {
        set(_state == LedState::On ? LedState::Off : LedState::On);
    }

    LedState state() const { return _state; }

private:
    const uint8_t _pin;
    LedState      _state;
};

// ─────────────────────────────────────────────────────────────────────────────
// Blinker
// ─────────────────────────────────────────────────────────────────────────────

class Blinker {
public:
    Blinker(Led& led, unsigned long periodMs)
        : _led(led), _halfPeriod(periodMs / 2), _last(0) {}

    void tick(unsigned long now) {
        if (now - _last >= _halfPeriod) {
            _led.toggle();
            _last = now;
        }
    }

    unsigned long timeUntilNext(unsigned long now) const {
        const unsigned long elapsed = now - _last;
        if (elapsed >= _halfPeriod) return 0;
        return _halfPeriod - elapsed;
    }

private:
    Led&          _led;
    unsigned long _halfPeriod;
    unsigned long _last;
};

// ─────────────────────────────────────────────────────────────────────────────
// PowerSleep — estimates next event, sleeps, tracks power stats
// ─────────────────────────────────────────────────────────────────────────────

class PowerSleep {
public:
    static constexpr uint8_t MAX_SOURCES = 4;

    PowerSleep()
        : _count(0), _totalActiveUs(0), _totalSleepMs(0), _lastReport(0) {}

    void add(Blinker& b) {
        if (_count < MAX_SOURCES) _sources[_count++] = &b;
    }

    void tick(unsigned long now, unsigned long activeStartUs) {
        _totalActiveUs += micros() - activeStartUs;

        const unsigned long sleepMs = _nextEvent(now);

        if (sleepMs >= Config::MIN_SLEEP_MS) {
            _totalSleepMs += sleepMs;
            Serial.flush();
            esp_sleep_enable_timer_wakeup(sleepMs * 1000UL);
            esp_light_sleep_start();
        }
    }

    void report(unsigned long now) {
        if (now - _lastReport < Config::REPORT_EVERY_MS) return;

        const float totalMs     = static_cast<float>(_totalSleepMs)
                                + static_cast<float>(_totalActiveUs) / 1000.0f;
        const float activeRatio = (_totalActiveUs / 1000.0f) / totalMs;
        const float sleepRatio  = static_cast<float>(_totalSleepMs) / totalMs;
        const float avgPower    = (activeRatio * Config::NORMAL_MW)
                                + (sleepRatio  * Config::SLEEP_MW);

        Serial.println("---------------------");
        Serial.print("Active: "); Serial.print(activeRatio * 100.0f, 2); Serial.println(" %");
        Serial.print("Sleep:  "); Serial.print(sleepRatio  * 100.0f, 2); Serial.println(" %");
        Serial.print("~Power: "); Serial.print(avgPower, 3);             Serial.println(" mW");

        _totalActiveUs = 0;
        _totalSleepMs  = 0;
        _lastReport    = now;
    }

private:
    unsigned long _nextEvent(unsigned long now) const {
        unsigned long minMs = 0xFFFFFFFFUL;
        for (uint8_t i = 0; i < _count; ++i) {
            const unsigned long t = _sources[i]->timeUntilNext(now);
            if (t < minMs) minMs = t;
        }
        return minMs;
    }

    Blinker*      _sources[MAX_SOURCES];
    uint8_t       _count;
    unsigned long _totalActiveUs;
    unsigned long _totalSleepMs;
    unsigned long _lastReport;
};

// ─────────────────────────────────────────────────────────────────────────────
// Module objects
// ─────────────────────────────────────────────────────────────────────────────

static Led led1(Config::LED1_PIN);
static Led led2(Config::LED2_PIN);
static Led led3(Config::LED3_PIN);

static Blinker   blinker1(led1, Config::LED1_PERIOD_MS);
static Blinker   blinker2(led2, Config::LED2_PERIOD_MS);
static Blinker   blinker3(led3, Config::LED3_PERIOD_MS);
static PowerSleep sleeper;

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    led1.init();
    led2.init();
    led3.init();

    sleeper.add(blinker1);
    sleeper.add(blinker2);
    sleeper.add(blinker3);

    Serial.println("=== Power-Save Superloop ===");
    Serial.println("LED1: 200ms | LED2: 500ms | LED3: 1000ms");
}

void loop() {
    const unsigned long activeStartUs = micros();
    const unsigned long now           = millis();

    blinker1.tick(now);
    blinker2.tick(now);
    blinker3.tick(now);

    sleeper.tick(now, activeStartUs);
    sleeper.report(now);
}
