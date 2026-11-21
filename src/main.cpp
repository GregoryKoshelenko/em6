#include <Arduino.h>

struct Config {
    static constexpr uint8_t RED_PIN    = 2;
    static constexpr uint8_t YELLOW_PIN = 21;
    static constexpr uint8_t GREEN_PIN  = 19;
    static constexpr uint8_t BUTTON_PIN = 0;   // BOOT button, active LOW

    static constexpr uint32_t GREEN_MS        = 10000;
    static constexpr uint32_t GREEN_BLINK_MS  = 3000;
    static constexpr uint32_t YELLOW_MS       = 3000;
    static constexpr uint32_t RED_MS          = 10000;
    static constexpr uint32_t RED_YELLOW_MS   = 2000;
    static constexpr uint32_t BLINK_HALF_MS   = 500;   // 1 Hz blinking
    static constexpr uint32_t DEBOUNCE_MS     = 50;

    static constexpr uint32_t SERIAL_BAUD = 115200;
};

class Led {
public:
    constexpr explicit Led(uint8_t pin) : _pin(pin) {}
    void init() const { pinMode(_pin, OUTPUT); set(false); }
    void set(bool on) const { digitalWrite(_pin, on ? HIGH : LOW); }
private:
    const uint8_t _pin;
};

class Button {
public:
    constexpr explicit Button(uint8_t pin) : _pin(pin) {}

    void init() { pinMode(_pin, INPUT_PULLUP); }

    // Returns true once per press, after the signal is stable for DEBOUNCE_MS.
    bool pressed(uint32_t now) {
        const bool raw = digitalRead(_pin) == LOW;
        if (raw != _lastRaw) {
            _lastRaw    = raw;
            _changedAt  = now;
        }
        if (now - _changedAt >= Config::DEBOUNCE_MS && raw != _stable) {
            _stable = raw;
            return _stable;
        }
        return false;
    }

private:
    const uint8_t _pin;
    bool     _lastRaw   = false;
    bool     _stable    = false;
    uint32_t _changedAt = 0;
};

enum class State : uint8_t {
    Green,
    GreenBlink,
    Yellow,
    Red,
    RedYellow,
    YellowBlink,   // unregulated crossing (service mode)
};

class TrafficLight {
public:
    TrafficLight(const Led& red, const Led& yellow, const Led& green)
        : _red(red), _yellow(yellow), _green(green) {}

    void begin(uint32_t now) { enter(State::Green, now); }

    void toggleService(uint32_t now) {
        enter(_state == State::YellowBlink ? State::Red : State::YellowBlink, now);
    }

    void tick(uint32_t now) {
        const uint32_t elapsed = now - _enteredAt;
        const bool blinkOn = (elapsed / Config::BLINK_HALF_MS) % 2 == 0;

        switch (_state) {
            case State::Green:
                if (elapsed >= Config::GREEN_MS) enter(State::GreenBlink, now);
                break;
            case State::GreenBlink:
                _green.set(blinkOn);
                if (elapsed >= Config::GREEN_BLINK_MS) enter(State::Yellow, now);
                break;
            case State::Yellow:
                if (elapsed >= Config::YELLOW_MS) enter(State::Red, now);
                break;
            case State::Red:
                if (elapsed >= Config::RED_MS) enter(State::RedYellow, now);
                break;
            case State::RedYellow:
                if (elapsed >= Config::RED_YELLOW_MS) enter(State::Green, now);
                break;
            case State::YellowBlink:
                _yellow.set(blinkOn);
                break;
        }
    }

private:
    void enter(State next, uint32_t now) {
        _state     = next;
        _enteredAt = now;
        _red.set(next == State::Red || next == State::RedYellow);
        _yellow.set(next == State::Yellow || next == State::RedYellow
                    || next == State::YellowBlink);
        _green.set(next == State::Green || next == State::GreenBlink);
        Serial.printf("[%lu ms] -> %s\n", static_cast<unsigned long>(now), name(next));
    }

    static const char* name(State s) {
        switch (s) {
            case State::Green:       return "GREEN";
            case State::GreenBlink:  return "GREEN BLINK";
            case State::Yellow:      return "YELLOW";
            case State::Red:         return "RED";
            case State::RedYellow:   return "RED + YELLOW";
            case State::YellowBlink: return "YELLOW BLINK (unregulated)";
        }
        return "?";
    }

    const Led& _red;
    const Led& _yellow;
    const Led& _green;
    State    _state     = State::Green;
    uint32_t _enteredAt = 0;
};

static const Led    redLed(Config::RED_PIN);
static const Led    yellowLed(Config::YELLOW_PIN);
static const Led    greenLed(Config::GREEN_PIN);
static Button       modeButton(Config::BUTTON_PIN);
static TrafficLight light(redLed, yellowLed, greenLed);

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    redLed.init();
    yellowLed.init();
    greenLed.init();
    modeButton.init();

    Serial.println("=== Traffic light (PDR Ukraine) ===");
    Serial.println("BOOT button: toggle yellow blinking mode");
    light.begin(millis());
}

void loop() {
    const uint32_t now = millis();
    if (modeButton.pressed(now)) light.toggleService(now);
    light.tick(now);
}
