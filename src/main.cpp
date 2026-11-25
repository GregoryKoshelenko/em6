// Lesson 32 (module 3.2): ADC calibration. Potentiometer divider -> ADC1, every 100 ms:
//   RAW -> U_manual = RAW * VREF / ADC_MAX   (naive linear formula)
//   RAW -> U_cali   = esp_adc calibration    (curve fitting, uses eFuse data)
//   Error(%) = (U_manual - U_cali) / U_cali * 100
// A table row is printed when RAW changes noticeably (turn the potentiometer).
#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

struct Config {
    static constexpr adc_unit_t     ADC_UNIT    = ADC_UNIT_1;
    static constexpr adc_channel_t  ADC_CHANNEL = ADC_CHANNEL_0;    // GPIO1 on ESP32-S3
    static constexpr adc_atten_t    ADC_ATTEN   = ADC_ATTEN_DB_12;  // input range ~0..3100 mV
    static constexpr adc_bitwidth_t ADC_BITS    = ADC_BITWIDTH_12;  // 0..4095
    static constexpr int      ADC_MAX      = 4095;
    static constexpr int      VREF_MV      = 3300;  // naive full scale = supply voltage
    static constexpr uint32_t SAMPLE_MS    = 100;
    static constexpr int      RAW_STEP     = 40;    // print a new row after this change
    static constexpr uint8_t  HEADER_EVERY = 20;    // repeat the header every N rows
};

// Declared before any function: the .ino preprocessor puts prototypes above the first function.
struct Measurement {
    int   raw;
    float manualMv;
    int   caliMv;
    float errorPct;
};

static adc_oneshot_unit_handle_t adcHandle;
static adc_cali_handle_t         caliHandle;

static void adcInit() {
    adc_oneshot_unit_init_cfg_t unitCfg = {};
    unitCfg.unit_id = Config::ADC_UNIT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitCfg, &adcHandle));

    adc_oneshot_chan_cfg_t chCfg = {};
    chCfg.atten    = Config::ADC_ATTEN;
    chCfg.bitwidth = Config::ADC_BITS;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, Config::ADC_CHANNEL, &chCfg));

    adc_cali_curve_fitting_config_t caliCfg = {};
    caliCfg.unit_id  = Config::ADC_UNIT;
    caliCfg.chan     = Config::ADC_CHANNEL;
    caliCfg.atten    = Config::ADC_ATTEN;
    caliCfg.bitwidth = Config::ADC_BITS;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&caliCfg, &caliHandle));
}

static bool measure(Measurement& m) {
    if (adc_oneshot_read(adcHandle, Config::ADC_CHANNEL, &m.raw) != ESP_OK) return false;
    if (adc_cali_raw_to_voltage(caliHandle, m.raw, &m.caliMv) != ESP_OK) return false;
    m.manualMv = m.raw * float(Config::VREF_MV) / Config::ADC_MAX;
    m.errorPct = m.caliMv > 0 ? (m.manualMv - m.caliMv) * 100.0f / m.caliMv : 0.0f;
    return true;
}

static void printHeader() {
    Serial.println();
    Serial.println(" RAW   U_manual(mV)   U_cali(mV)   Error(%)");
    Serial.println("------------------------------------------");
}

static void printRow(const Measurement& m) {
    Serial.printf("%4d   %12.1f   %10d   %8.2f\n", m.raw, m.manualMv, m.caliMv, m.errorPct);
}

void setup() {
    Serial.begin(115200);
    adcInit();
    Serial.println("ADC calibration check: ADC1_CH0 (GPIO1), atten 12 dB (~0..3100 mV),");
    Serial.printf("12-bit (0..%d), U_manual = RAW * %d / %d. Turn the potentiometer.\n",
                  Config::ADC_MAX, Config::VREF_MV, Config::ADC_MAX);
    printHeader();
}

void loop() {
    static uint32_t lastSample = 0;
    static int      lastPrintedRaw = -1000;
    static uint8_t  rows = 0;

    const uint32_t now = millis();
    if (now - lastSample < Config::SAMPLE_MS) { delay(1); return; }
    lastSample = now;

    Measurement m;
    if (!measure(m)) { Serial.println("ADC read error"); return; }
    if (abs(m.raw - lastPrintedRaw) < Config::RAW_STEP) return;
    lastPrintedRaw = m.raw;

    if (rows > 0 && rows % Config::HEADER_EVERY == 0) printHeader();
    printRow(m);
    ++rows;
}
