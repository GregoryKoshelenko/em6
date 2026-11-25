// Lesson 34 (module 3.4), task 1: non-blocking audio player for a buzzer.
//  - sound via PWM (LEDC): frequency = note pitch, duty 50% = sound, 0% = silence
//  - fixed 50 ms tick: every note lasts N ticks, the player advances one tick at a time
//  - nothing blocks: no delay(), no FreeRTOS task; a heartbeat LED keeps blinking meanwhile
// Melody: "Twinkle Twinkle Little Star" (traditional), first two phrases, then repeats.
#include <Arduino.h>
#include <driver/ledc.h>

struct Config {
    static constexpr uint8_t  BUZZER_PIN   = 7;
    static constexpr uint8_t  LED_PIN      = 2;
    static constexpr uint32_t TICK_MS      = 50;     // fixed player tick
    static constexpr uint32_t BLINK_MS     = 250;    // heartbeat, independent of the player
    static constexpr ledc_timer_t   TIMER   = LEDC_TIMER_0;
    static constexpr ledc_channel_t CHANNEL = LEDC_CHANNEL_0;
    static constexpr ledc_timer_bit_t RES   = LEDC_TIMER_10_BIT;
    static constexpr uint32_t DUTY_ON      = 512;    // 50% of 1023: square wave
};

// A note: pitch in Hz (0 = rest) and length in ticks (1 tick = 50 ms).
struct Note {
    uint16_t freqHz;
    uint8_t  ticks;
};

// NOTE_ prefix: plain A4 / C4 clash with Arduino's analog pin names.
enum Pitch : uint16_t {
    REST = 0, NOTE_C4 = 262, NOTE_D4 = 294, NOTE_E4 = 330,
    NOTE_F4 = 349, NOTE_G4 = 392, NOTE_A4 = 440
};

static constexpr uint8_t Q = 8;    // quarter note = 8 ticks = 400 ms
static constexpr uint8_t H = 16;   // half note

static const Note MELODY[] = {
    {NOTE_C4, Q}, {NOTE_C4, Q}, {NOTE_G4, Q}, {NOTE_G4, Q}, {NOTE_A4, Q}, {NOTE_A4, Q}, {NOTE_G4, H},
    {NOTE_F4, Q}, {NOTE_F4, Q}, {NOTE_E4, Q}, {NOTE_E4, Q}, {NOTE_D4, Q}, {NOTE_D4, Q}, {NOTE_C4, H},
    {REST, H},
};
static constexpr size_t MELODY_LEN = sizeof(MELODY) / sizeof(MELODY[0]);

// ---------- Buzzer on LEDC ----------
class Buzzer {
public:
    void init() {
        ledc_timer_config_t t = {};
        t.speed_mode      = LEDC_LOW_SPEED_MODE;
        t.timer_num       = Config::TIMER;
        t.duty_resolution = Config::RES;
        t.freq_hz         = 440;
        t.clk_cfg         = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));

        ledc_channel_config_t c = {};
        c.speed_mode = LEDC_LOW_SPEED_MODE;
        c.channel    = Config::CHANNEL;
        c.timer_sel  = Config::TIMER;
        c.gpio_num   = Config::BUZZER_PIN;
        c.duty       = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }

    void tone(uint16_t freqHz) {
        ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, Config::TIMER, freqHz));
        setDuty(Config::DUTY_ON);
    }

    void silence() { setDuty(0); }

private:
    void setDuty(uint32_t duty) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, Config::CHANNEL, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, Config::CHANNEL));
    }
};

// ---------- Player: a tiny state machine driven by the 50 ms tick ----------
class Player {
public:
    explicit Player(Buzzer& buzzer) : _buzzer(buzzer) {}

    // Called exactly once per tick.
    void tick() {
        if (_left == 0) startNote();                 // previous note finished
        else if (_left == 1) _buzzer.silence();      // last tick of a note: short gap,
                                                     // so equal notes are heard apart
        --_left;
    }

private:
    void startNote() {
        const Note& n = MELODY[_index];
        _left = n.ticks;
        if (n.freqHz == REST) _buzzer.silence();
        else                  _buzzer.tone(n.freqHz);
        Serial.printf("[%6lu ms] note %2u: %s %3u Hz, %u ticks\n",
                      (unsigned long)millis(), unsigned(_index),
                      n.freqHz ? "tone" : "rest", n.freqHz, n.ticks);
        _index = (_index + 1) % MELODY_LEN;          // loop the melody
    }

    Buzzer& _buzzer;
    size_t  _index = 0;
    uint8_t _left  = 0;
};

static Buzzer buzzer;
static Player player(buzzer);

void setup() {
    Serial.begin(115200);
    pinMode(Config::LED_PIN, OUTPUT);
    buzzer.init();
    Serial.println("Non-blocking buzzer player: 50 ms tick, no delay(), no tasks");
}

void loop() {
    static uint32_t lastTick = 0, lastBlink = 0;
    static bool ledOn = false;
    const uint32_t now = millis();

    // Fixed tick: catch up if the loop was late, so the tempo stays exact.
    while (now - lastTick >= Config::TICK_MS) {
        lastTick += Config::TICK_MS;
        player.tick();
    }

    // Independent work that keeps running while the melody plays.
    if (now - lastBlink >= Config::BLINK_MS) {
        lastBlink = now;
        ledOn = !ledOn;
        digitalWrite(Config::LED_PIN, ledOn ? HIGH : LOW);
    }
}
