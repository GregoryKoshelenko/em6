#include <Arduino.h>

struct Config {
    // Task 1: Relay
    static constexpr uint8_t       RELAY_CTRL_PIN    = 5;
    static constexpr uint8_t       RELAY_FB_PIN      = 18;
    static constexpr uint8_t       MEAS_COUNT        = 10;
    static constexpr unsigned long MEAS_INTERVAL_MS  = 500UL;

    // Task 2: Soft PWM
    static constexpr uint8_t       POT_PIN           = 34;
    static constexpr uint8_t       PWM_PIN           = 19;
    static constexpr unsigned long PWM_PERIOD_MS     = 20UL;
    static constexpr uint32_t      ADC_MAX           = 4095UL;

    static constexpr uint32_t      SERIAL_BAUD       = 115200UL;
};

static volatile bool          gContactFired = false;
static volatile unsigned long gContactTime  = 0;

void IRAM_ATTR onRelayContact() {
    gContactTime  = millis();
    gContactFired = true;
}

class RelayTimer {
public:
    RelayTimer()
        : _triggerTime(0), _lastMeas(0), _sum(0), _count(0),
          _state(State::Triggering) {}

    void begin() {
        pinMode(Config::RELAY_CTRL_PIN, OUTPUT);
        pinMode(Config::RELAY_FB_PIN,   INPUT_PULLUP);
        digitalWrite(Config::RELAY_CTRL_PIN, LOW);
        attachInterrupt(digitalPinToInterrupt(Config::RELAY_FB_PIN),
                        onRelayContact, FALLING);
    }

    bool done() const { return _state == State::Finished; }

    void tick(unsigned long now) {
        switch (_state) {
            case State::Triggering:
                gContactFired = false;
                _triggerTime  = now;
                digitalWrite(Config::RELAY_CTRL_PIN, HIGH);
                _state = State::WaitContact;
                break;

            case State::WaitContact:
                if (gContactFired) {
                    const unsigned long elapsed = gContactTime - _triggerTime;
                    _sum += elapsed;
                    _count++;
                    Serial.print("  [");
                    Serial.print(_count);
                    Serial.print("/");
                    Serial.print(Config::MEAS_COUNT);
                    Serial.print("] ");
                    Serial.print(elapsed);
                    Serial.println(" ms");
                    digitalWrite(Config::RELAY_CTRL_PIN, LOW);
                    _lastMeas = now;
                    _state = (_count < Config::MEAS_COUNT) ? State::Cooldown
                                                           : State::PrintResult;
                }
                break;

            case State::Cooldown:
                if (now - _lastMeas >= Config::MEAS_INTERVAL_MS) {
                    _state = State::Triggering;
                }
                break;

            case State::PrintResult:
                Serial.println("---------------------");
                Serial.print("Average: ");
                Serial.print(_sum / _count);
                Serial.println(" ms");
                _state = State::Finished;
                break;

            case State::Finished:
                break;
        }
    }

private:
    enum class State : uint8_t { Triggering, WaitContact, Cooldown, PrintResult, Finished };

    unsigned long _triggerTime;
    unsigned long _lastMeas;
    unsigned long _sum;
    uint8_t       _count;
    State         _state;
};


class SoftPwm {
public:
    explicit SoftPwm(uint8_t pin) : _pin(pin), _duty(0) {}

    void begin() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
    }

    void setDuty(uint8_t duty) {
        _duty = (duty > 100) ? 100 : duty;
    }

    uint8_t duty() const { return _duty; }

    void tick(unsigned long now) {
        const unsigned long phase  = now % Config::PWM_PERIOD_MS;
        const unsigned long onTime = (Config::PWM_PERIOD_MS * _duty) / 100UL;
        digitalWrite(_pin, phase < onTime ? HIGH : LOW);
    }

private:
    const uint8_t _pin;
    uint8_t       _duty;
};


static RelayTimer relayTimer;
static SoftPwm    motorPwm(Config::PWM_PIN);


void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    relayTimer.begin();
    motorPwm.begin();
    Serial.println("=== Relay Timer + Soft PWM ===");
    Serial.println("Relay measurements starting...");
}

void loop() {
    const unsigned long now = millis();

    relayTimer.tick(now);

    const uint32_t adcVal = analogRead(Config::POT_PIN);
    const uint8_t  duty   = static_cast<uint8_t>((adcVal * 100UL) / Config::ADC_MAX);
    motorPwm.setDuty(duty);
    motorPwm.tick(now);

    static unsigned long lastPrint = 0;
    static uint8_t       lastDuty  = 255;
    if (duty != lastDuty && now - lastPrint >= 200UL) {
        Serial.print("PWM duty: ");
        Serial.print(duty);
        Serial.println("%");
        lastDuty  = duty;
        lastPrint = now;
    }
}
