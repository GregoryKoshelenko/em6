// Lesson 40 (module 4.1): UART link ESP32-S3 <-> STM32 (Nucleo-L031K6).
//  - ESP32 button press -> "T" over UART -> STM32 toggles its LED (done in class)
//  - STM32 button press -> "T" over UART -> ESP32 toggles its LED (homework)
//
// Line protocol, 8N1, every message ends with '\n':
//   "T"        toggle your LED
//   "L0"/"L1"  acknowledge: my LED is now off/on
//   "#..."     log line, the receiver ignores it
//
// Wiring: ESP GPIO17 (TX1) -> STM32 RX, ESP GPIO18 (RX1) <- STM32 TX, GND <-> GND.
// Wokwi: the Serial Monitor is wired to the link, so it plays the STM32 side
// (type T + Enter to toggle the ESP32 LED; ESP32 button presses show up as "T").
#include <Arduino.h>

struct Config {
    static constexpr uint8_t  LED_PIN     = 2;
    static constexpr uint8_t  BUTTON_PIN  = 4;      // to GND, internal pull-up
    static constexpr uint8_t  LINK_RX_PIN = 18;
    static constexpr uint8_t  LINK_TX_PIN = 17;
    static constexpr uint32_t LINK_BAUD   = 115200; // lesson starts with 9600; both sides must match
    static constexpr uint32_t DEBOUNCE_MS = 50;
    static constexpr size_t   LINE_LEN    = 16;
};

class Led {
public:
    explicit Led(uint8_t pin) : _pin(pin) {}
    void init() { pinMode(_pin, OUTPUT); set(false); }
    void set(bool on) { _on = on; digitalWrite(_pin, on ? HIGH : LOW); }
    void toggle() { set(!_on); }
    bool isOn() const { return _on; }
private:
    const uint8_t _pin;
    bool _on = false;
};

// Polled debounce: one event per press, after the level is stable for DEBOUNCE_MS.
class Button {
public:
    explicit Button(uint8_t pin) : _pin(pin) {}
    void init() { pinMode(_pin, INPUT_PULLUP); }
    bool pressed(uint32_t now) {
        const bool raw = digitalRead(_pin) == LOW;
        if (raw != _lastRaw) { _lastRaw = raw; _changedAt = now; }
        if (now - _changedAt >= Config::DEBOUNCE_MS && raw != _stable) {
            _stable = raw;
            return _stable;
        }
        return false;
    }
private:
    const uint8_t _pin;
    bool _lastRaw = false, _stable = false;
    uint32_t _changedAt = 0;
};

// Collects bytes into a line without blocking; returns true when a full line is ready.
class LineReader {
public:
    bool feed(char c) {
        if (c == '\r') return false;
        if (c == '\n') {
            _buf[_len] = '\0';
            const bool ready = _len > 0 && !_overflow;
            _len = 0;
            _overflow = false;
            return ready;
        }
        if (_len < Config::LINE_LEN - 1) _buf[_len++] = c;
        else _overflow = true;               // drop the over-long line
        return false;
    }
    const char* line() const { return _buf; }
private:
    char   _buf[Config::LINE_LEN] = {};
    size_t _len = 0;
    bool   _overflow = false;
};

static HardwareSerial peer(1);
static Led        led(Config::LED_PIN);
static Button     button(Config::BUTTON_PIN);
static LineReader reader;

static void handleLine(const char* msg) {
    if (msg[0] == '#') return;                               // peer log
    if (strcmp(msg, "T") == 0) {
        led.toggle();
        peer.println(led.isOn() ? "L1" : "L0");              // ack with the new state
    } else if (msg[0] == 'L' && (msg[1] == '0' || msg[1] == '1') && msg[2] == '\0') {
        peer.printf("# peer LED is %s\n", msg[1] == '1' ? "ON" : "OFF");
    } else {
        peer.printf("# unknown command: %s\n", msg);
    }
}

void setup() {
    peer.begin(Config::LINK_BAUD, SERIAL_8N1, Config::LINK_RX_PIN, Config::LINK_TX_PIN);
    led.init();
    button.init();
    peer.println("# ESP32 ready: button -> T, T -> toggle LED");
}

void loop() {
    if (button.pressed(millis())) peer.println("T");

    while (peer.available()) {
        if (reader.feed(static_cast<char>(peer.read()))) handleLine(reader.line());
    }

    delay(1);  // idle; keeps the Wokwi simulation fast
}
