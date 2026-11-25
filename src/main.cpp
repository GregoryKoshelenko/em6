// Lesson 31 (module 3.1): analog signals. LDR divider -> ADC1 oneshot -> SMA -> LED with hysteresis.
//  - ADC read with the ESP-IDF oneshot driver + curve-fitting calibration (mV)
//  - Simple Moving Average over the last SMA_WINDOW samples (ring buffer, running sum)
//  - two thresholds: LED turns ON above DARK_ON_MV, OFF below LIGHT_OFF_MV (no flicker in between)
// LDR module: brighter light -> lower output voltage, so "dark" = high mV.
#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

struct Config {
    static constexpr adc_unit_t    ADC_UNIT    = ADC_UNIT_1;
    static constexpr adc_channel_t ADC_CHANNEL = ADC_CHANNEL_8;   // GPIO9 on ESP32-S3
    static constexpr adc_atten_t   ADC_ATTEN   = ADC_ATTEN_DB_12; // 0..~3.1 V
    static constexpr uint8_t  LED_PIN       = 2;
    static constexpr uint8_t  SMA_WINDOW    = 16;
    static constexpr uint32_t SAMPLE_MS     = 50;    // 20 samples/s
    static constexpr uint32_t LOG_MS        = 500;
    static constexpr int      DARK_ON_MV    = 2000;  // above -> dark -> LED on
    static constexpr int      LIGHT_OFF_MV  = 1500;  // below -> light -> LED off
};

// ---------- ADC (oneshot) ----------
static adc_oneshot_unit_handle_t adcHandle;
static adc_cali_handle_t         caliHandle;

static void adcInit() {
    adc_oneshot_unit_init_cfg_t unitCfg = {};
    unitCfg.unit_id = Config::ADC_UNIT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitCfg, &adcHandle));

    adc_oneshot_chan_cfg_t chCfg = {};
    chCfg.atten    = Config::ADC_ATTEN;
    chCfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, Config::ADC_CHANNEL, &chCfg));

    adc_cali_curve_fitting_config_t caliCfg = {};
    caliCfg.unit_id  = Config::ADC_UNIT;
    caliCfg.chan     = Config::ADC_CHANNEL;
    caliCfg.atten    = Config::ADC_ATTEN;
    caliCfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&caliCfg, &caliHandle));
}

// One conversion: raw 0..4095 and calibrated millivolts.
static bool adcReadMv(int& raw, int& mv) {
    if (adc_oneshot_read(adcHandle, Config::ADC_CHANNEL, &raw) != ESP_OK) return false;
    return adc_cali_raw_to_voltage(caliHandle, raw, &mv) == ESP_OK;
}

// ---------- Filter ----------
class MovingAverage {
public:
    int add(int sample) {
        _sum += sample - _buf[_idx];          // drop the oldest, add the newest
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

// ---------- Decision with hysteresis ----------
// Between the two thresholds the LED keeps its previous state.
static bool updateLed(bool ledOn, int filteredMv) {
    if (!ledOn && filteredMv > Config::DARK_ON_MV)   return true;
    if (ledOn  && filteredMv < Config::LIGHT_OFF_MV) return false;
    return ledOn;
}

static MovingAverage sma;
static bool ledOn = false;

void setup() {
    Serial.begin(115200);
    pinMode(Config::LED_PIN, OUTPUT);
    digitalWrite(Config::LED_PIN, LOW);
    adcInit();
    Serial.printf("LDR -> SMA(%u) -> LED, ON > %d mV, OFF < %d mV\n",
                  Config::SMA_WINDOW, Config::DARK_ON_MV, Config::LIGHT_OFF_MV);
}

void loop() {
    static uint32_t lastSample = 0, lastLog = 0;
    static int raw = 0, mv = 0, avg = 0;
    const uint32_t now = millis();

    if (now - lastSample >= Config::SAMPLE_MS) {
        lastSample = now;
        if (adcReadMv(raw, mv)) {
            avg = sma.add(mv);
            const bool next = updateLed(ledOn, avg);
            if (next != ledOn) {
                ledOn = next;
                digitalWrite(Config::LED_PIN, ledOn ? HIGH : LOW);
                Serial.printf("LED %s (sma %d mV)\n", ledOn ? "ON  - dark" : "OFF - light", avg);
            }
        }
    }

    if (now - lastLog >= Config::LOG_MS) {
        lastLog = now;
        Serial.printf("raw %4d  %4d mV  sma %4d mV  LED %s\n", raw, mv, avg, ledOn ? "on" : "off");
    }

    delay(1);  // idle; keeps the Wokwi simulation fast
}
