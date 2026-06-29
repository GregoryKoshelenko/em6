#include <Arduino.h>

enum class LedState  : uint8_t { Off = LOW, On = HIGH };
enum class BlinkMode : uint8_t { Blinking, AlwaysOn, AlwaysOff };

struct Config {
    static constexpr uint8_t  RED_LED_PIN      = 2;
    static constexpr uint8_t  BLUE_LED_PIN     = 21;
    static constexpr uint8_t  BUTTON_PIN       = 7;
    static constexpr uint8_t  BOOT_BUTTON_PIN  = 0;

    static constexpr unsigned long FAST_BLINK_MS = 200UL;
    static constexpr unsigned long SLOW_BLINK_MS = 800UL;
    static constexpr unsigned long DEBOUNCE_MS   = 50UL;
    static constexpr uint32_t     SERIAL_BAUD    = 115200UL;

    static const uint16_t REPORT_EVERY; 
    static const uint8_t  SHORT_PRESS_BLINKS;
};

const uint16_t Config::REPORT_EVERY       = 1000U;
const uint8_t  Config::SHORT_PRESS_BLINKS = 3U;


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


class Blinker {
public:
    Blinker(Led& led, unsigned long periodMs)
        : _led(led), _halfPeriod(periodMs / 2), _last(0) {}

    void setPeriod(unsigned long periodMs) {
        _halfPeriod = periodMs / 2;
    }

    void tick(unsigned long now) {
        if (now - _last >= _halfPeriod) {
            _led.toggle();
            _last = now;
        }
    }

private:
    Led&          _led;
    unsigned long _halfPeriod;
    unsigned long _last;
};


static volatile bool gModeButtonEvent  = false;
static volatile bool gSpeedButtonEvent = false;

void IRAM_ATTR onModeButton()  { gModeButtonEvent  = true; }
void IRAM_ATTR onSpeedButton() { gSpeedButtonEvent = true; }


static Led     redLed(Config::RED_LED_PIN);
static Led     blueLed(Config::BLUE_LED_PIN);
static Blinker redBlinker(redLed,  Config::SLOW_BLINK_MS);
static Blinker blueBlinker(blueLed, Config::SLOW_BLINK_MS);
static BlinkMode currentMode = BlinkMode::Blinking;

static void applyMode(BlinkMode mode) {
    switch (mode) {
        case BlinkMode::AlwaysOn:
            redLed.set(LedState::On);
            blueLed.set(LedState::On);
            break;
        case BlinkMode::AlwaysOff:
            redLed.set(LedState::Off);
            blueLed.set(LedState::Off);
            break;
        case BlinkMode::Blinking:
            break;
    }
}

static const char* modeLabel(BlinkMode mode) {
    switch (mode) {
        case BlinkMode::Blinking:  return "Blinking";
        case BlinkMode::AlwaysOn:  return "Always ON";
        case BlinkMode::AlwaysOff: return "Always OFF";
    }
    return "";
}

void setup() {
    Serial.begin(Config::SERIAL_BAUD);

    redLed.init();
    blueLed.init();

    pinMode(Config::BUTTON_PIN,      INPUT_PULLUP);
    pinMode(Config::BOOT_BUTTON_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Config::BUTTON_PIN),
                    onModeButton,  RISING);
    attachInterrupt(digitalPinToInterrupt(Config::BOOT_BUTTON_PIN),
                    onSpeedButton, RISING);

    Serial.println("=== Embedded C++ Blinker ===");
    Serial.print("Mode: ");
    Serial.println(modeLabel(currentMode));
    Serial.println("GPIO7 = cycle mode | BOOT = fast/slow");
}

void loop() {
    static unsigned long loopCount       = 0;
    static unsigned long reportStart     = 0;
    static unsigned long lastModeDebounce  = 0;
    static unsigned long lastSpeedDebounce = 0;
    static bool          fastBlink       = false;

    const unsigned long now = millis();

    if (gModeButtonEvent && (now - lastModeDebounce >= Config::DEBOUNCE_MS)) {
        gModeButtonEvent   = false;
        lastModeDebounce   = now;

        switch (currentMode) {
            case BlinkMode::Blinking:  currentMode = BlinkMode::AlwaysOn;  break;
            case BlinkMode::AlwaysOn:  currentMode = BlinkMode::AlwaysOff; break;
            case BlinkMode::AlwaysOff: currentMode = BlinkMode::Blinking;  break;
        }
        applyMode(currentMode);

        Serial.print("Mode → ");
        Serial.println(modeLabel(currentMode));
    }

    if (gSpeedButtonEvent && (now - lastSpeedDebounce >= Config::DEBOUNCE_MS)) {
        gSpeedButtonEvent  = false;
        lastSpeedDebounce  = now;

        fastBlink = !fastBlink;
        const unsigned long period = fastBlink ? Config::FAST_BLINK_MS
                                               : Config::SLOW_BLINK_MS;
        redBlinker.setPeriod(period);
        blueBlinker.setPeriod(period);

        Serial.print("Speed → ");
        Serial.println(fastBlink ? "fast (200ms)" : "slow (800ms)");
    }

    if (currentMode == BlinkMode::Blinking) {
        redBlinker.tick(now);
        blueBlinker.tick(now);
    }

    if (++loopCount % Config::REPORT_EVERY == 0) {
        const unsigned long elapsed = now - reportStart;
        Serial.print("1000 iter = ");
        Serial.print(elapsed);
        Serial.println(" ms");
        reportStart = now;
    }
}
