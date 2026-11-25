// Lesson 43 (module 4.4): SPI. BME280 over SPI added to the lesson 42 clock.
//  - I2C bus: SSD1306 (0x3C) + DS1307 RTC (0x68), as in lesson 42
//  - SPI bus: BME280 (own register-level driver, datasheet compensation)
//  - screen: date, time, then T / RH / P; BME280 read every 500 ms, no delay()
//  - every value shown on the screen is duplicated to the log
// In Wokwi the BME280 is a custom chip (bme280.chip.c), Wokwi has no built-in one.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

struct Config {
    // I2C
    static constexpr uint8_t  SDA_PIN   = 8;
    static constexpr uint8_t  SCL_PIN   = 9;
    static constexpr uint32_t I2C_HZ    = 400000;
    static constexpr uint8_t  OLED_ADDR = 0x3C;
    static constexpr uint8_t  RTC_ADDR  = 0x68;
    // SPI (BME280)
    static constexpr uint8_t  SPI_SCK   = 12;
    static constexpr uint8_t  SPI_MOSI  = 11;   // -> BME280 SDI
    static constexpr uint8_t  SPI_MISO  = 13;   // <- BME280 SDO
    static constexpr uint8_t  BME_CS    = 10;   // -> BME280 CSB
    static constexpr uint32_t SPI_HZ    = 1000000;
    // Display
    static constexpr uint8_t  WIDTH     = 128;
    static constexpr uint8_t  HEIGHT    = 64;
    // Timing
    static constexpr uint32_t RTC_POLL_MS = 100;
    static constexpr uint32_t BME_POLL_MS = 500;   // "not less than once per second"
};

// ---------- Log: level + timestamp + tag, one line per event ----------
#define LOGI(fmt, ...) Serial.printf("[%8lu][I][app] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#define LOGE(fmt, ...) Serial.printf("[%8lu][E][app] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)

// Declared before any function (the .ino preprocessor puts prototypes above the first function).
struct DateTime {
    uint8_t  sec, min, hour;
    uint8_t  day, month;
    uint16_t year;
};

struct Climate {
    float tempC;
    float humidity;     // %RH
    float pressureHpa;
    bool  valid;
};

// Text lines built once, so the screen and the log show the same values.
struct ScreenText {
    char date[20];
    char tempHum[24];
    char press[24];
};

struct Bme280Calib {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;
};

static Adafruit_SSD1306 oled(Config::WIDTH, Config::HEIGHT, &Wire, -1);

// ======================= BME280 over SPI =======================
// SPI frame: CS low -> address byte (bit 7: 1 = read, 0 = write) -> data -> CS high.
class Bme280 {
public:
    static constexpr uint8_t REG_ID = 0xD0, REG_RESET = 0xE0, REG_CTRL_HUM = 0xF2,
                             REG_CTRL_MEAS = 0xF4, REG_CONFIG = 0xF5, REG_DATA = 0xF7,
                             REG_CALIB_TP = 0x88, REG_CALIB_H = 0xE1, CHIP_ID = 0x60;

    void readRegs(uint8_t reg, uint8_t* buf, size_t len) {
        SPI.beginTransaction(SETTINGS);
        digitalWrite(Config::BME_CS, LOW);
        SPI.transfer(reg | 0x80);                       // read
        for (size_t i = 0; i < len; ++i) buf[i] = SPI.transfer(0x00);
        digitalWrite(Config::BME_CS, HIGH);
        SPI.endTransaction();
    }

    void writeReg(uint8_t reg, uint8_t value) {
        SPI.beginTransaction(SETTINGS);
        digitalWrite(Config::BME_CS, LOW);
        SPI.transfer(reg & 0x7F);                       // write
        SPI.transfer(value);
        digitalWrite(Config::BME_CS, HIGH);
        SPI.endTransaction();
    }

    uint16_t u16(const uint8_t* b) { return b[0] | (b[1] << 8); }   // little endian
    int16_t  s16(const uint8_t* b) { return (int16_t)u16(b); }

    bool init() {
        pinMode(Config::BME_CS, OUTPUT);
        digitalWrite(Config::BME_CS, HIGH);
        SPI.begin(Config::SPI_SCK, Config::SPI_MISO, Config::SPI_MOSI, Config::BME_CS);

        uint8_t id = 0;
        readRegs(REG_ID, &id, 1);
        if (id != CHIP_ID) { LOGE("BME280 not found: id 0x%02X (expected 0x60)", id); return false; }

        uint8_t b[26];
        readRegs(REG_CALIB_TP, b, 26);
        cal.T1 = u16(b + 0);  cal.T2 = s16(b + 2);  cal.T3 = s16(b + 4);
        cal.P1 = u16(b + 6);  cal.P2 = s16(b + 8);  cal.P3 = s16(b + 10); cal.P4 = s16(b + 12);
        cal.P5 = s16(b + 14); cal.P6 = s16(b + 16); cal.P7 = s16(b + 18); cal.P8 = s16(b + 20);
        cal.P9 = s16(b + 22); cal.H1 = b[25];
        readRegs(REG_CALIB_H, b, 7);
        cal.H2 = s16(b + 0);  cal.H3 = b[2];
        cal.H4 = (int16_t)((b[3] << 4) | (b[4] & 0x0F));
        cal.H5 = (int16_t)((b[5] << 4) | (b[4] >> 4));
        cal.H6 = (int8_t)b[6];

        writeReg(REG_CTRL_HUM, 0x01);     // humidity x1 (must be written before ctrl_meas)
        writeReg(REG_CTRL_MEAS, 0x27);    // temp x1, press x1, normal mode
        writeReg(REG_CONFIG, 0xA0);       // standby 1000 ms, filter off
        LOGI("BME280 found on SPI, id 0x%02X", id);
        return true;
    }

    // Datasheet section 8.1 (double precision compensation).
    Climate read() {
        uint8_t d[8];
        readRegs(REG_DATA, d, 8);                       // one burst: press, temp, hum
        const int32_t adcP = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
        const int32_t adcT = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
        const int32_t adcH = (d[6] << 8) | d[7];

        double v1 = (adcT / 16384.0 - cal.T1 / 1024.0) * cal.T2;
        double v2 = (adcT / 131072.0 - cal.T1 / 8192.0) * (adcT / 131072.0 - cal.T1 / 8192.0) * cal.T3;
        const double tFine = v1 + v2;
        const double t = tFine / 5120.0;

        v1 = tFine / 2.0 - 64000.0;
        v2 = v1 * v1 * cal.P6 / 32768.0;
        v2 = v2 + v1 * cal.P5 * 2.0;
        v2 = v2 / 4.0 + cal.P4 * 65536.0;
        v1 = (cal.P3 * v1 * v1 / 524288.0 + cal.P2 * v1) / 524288.0;
        v1 = (1.0 + v1 / 32768.0) * cal.P1;
        double p = 0;
        if (v1 != 0.0) {
            p = 1048576.0 - adcP;
            p = (p - v2 / 4096.0) * 6250.0 / v1;
            const double w1 = cal.P9 * p * p / 2147483648.0;
            const double w2 = p * cal.P8 / 32768.0;
            p = p + (w1 + w2 + cal.P7) / 16.0;
        }

        double h = tFine - 76800.0;
        h = (adcH - (cal.H4 * 64.0 + cal.H5 / 16384.0 * h)) *
            (cal.H2 / 65536.0 * (1.0 + cal.H6 / 67108864.0 * h * (1.0 + cal.H3 / 67108864.0 * h)));
        h = h * (1.0 - cal.H1 * h / 524288.0);
        h = h > 100.0 ? 100.0 : (h < 0.0 ? 0.0 : h);

        Climate c;
        c.tempC = t; c.humidity = h; c.pressureHpa = p / 100.0;
        c.valid = (adcT != 0x80000) && t > -40.5 && t < 85.5 && c.pressureHpa > 290 && c.pressureHpa < 1110;
        return c;
    }
private:
    const SPISettings SETTINGS{Config::SPI_HZ, MSBFIRST, SPI_MODE0};
    Bme280Calib cal{};
};

// ======================= DS1307 (I2C, BCD registers) =======================
static uint8_t bcdToDec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t decToBcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

static bool rtcRead(DateTime& t, bool& halted) {
    Wire.beginTransmission(Config::RTC_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;   // repeated START
    if (Wire.requestFrom(Config::RTC_ADDR, uint8_t(7)) != 7) return false;
    const uint8_t s = Wire.read();
    halted  = s & 0x80;
    t.sec   = bcdToDec(s & 0x7F);
    t.min   = bcdToDec(Wire.read() & 0x7F);
    t.hour  = bcdToDec(Wire.read() & 0x3F);
    Wire.read();
    t.day   = bcdToDec(Wire.read() & 0x3F);
    t.month = bcdToDec(Wire.read() & 0x1F);
    t.year  = 2000 + bcdToDec(Wire.read());
    return t.sec < 60 && t.min < 60 && t.hour < 24 && t.day >= 1 && t.day <= 31
        && t.month >= 1 && t.month <= 12;
}

static bool rtcWrite(const DateTime& t) {
    Wire.beginTransmission(Config::RTC_ADDR);
    Wire.write(0x00);
    Wire.write(decToBcd(t.sec) & 0x7F);
    Wire.write(decToBcd(t.min));
    Wire.write(decToBcd(t.hour));
    Wire.write(1);
    Wire.write(decToBcd(t.day));
    Wire.write(decToBcd(t.month));
    Wire.write(decToBcd(t.year % 100));
    return Wire.endTransmission() == 0;
}

static DateTime compileTime() {
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char* d = __DATE__;
    const char* tm = __TIME__;
    DateTime t{};
    char mon[4] = { d[0], d[1], d[2], 0 };
    t.month = (strstr(months, mon) - months) / 3 + 1;
    t.day   = atoi(d + 4);
    t.year  = atoi(d + 7);
    t.hour  = atoi(tm);
    t.min   = atoi(tm + 3);
    t.sec   = atoi(tm + 6);
    return t;
}

static uint8_t dayOfWeek(uint16_t y, uint8_t m, uint8_t d) {
    static const uint8_t k[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    if (m < 3) y -= 1;
    return (y + y / 4 - y / 100 + y / 400 + k[m - 1] + d) % 7;
}

// ======================= Display =======================
static const char* const DAYS[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

static bool displayInit() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR)) return false;
    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    oled.display();
    return true;
}

static ScreenText buildText(const DateTime& t, const Climate& c) {
    ScreenText s;
    snprintf(s.date, sizeof(s.date), "%s %02u.%02u.%04u",
             DAYS[dayOfWeek(t.year, t.month, t.day)], t.day, t.month, t.year);
    if (c.valid) {
        snprintf(s.tempHum, sizeof(s.tempHum), "T:%.1fC  RH:%.0f%%", c.tempC, c.humidity);
        snprintf(s.press, sizeof(s.press), "P:%.0f hPa", c.pressureHpa);
    } else {
        snprintf(s.tempHum, sizeof(s.tempHum), "BME280: no data");
        s.press[0] = '\0';
    }
    return s;
}

static void displayAll(const DateTime& t, const ScreenText& s) {
    char buf[8];
    oled.clearDisplay();

    oled.setTextSize(1);
    oled.setCursor((Config::WIDTH - strlen(s.date) * 6) / 2, 0);
    oled.print(s.date);

    snprintf(buf, sizeof(buf), "%02u:%02u", t.hour, t.min);   // large HH:MM
    oled.setTextSize(3);
    oled.setCursor(2, 11);
    oled.print(buf);
    snprintf(buf, sizeof(buf), ":%02u", t.sec);
    oled.setTextSize(2);
    oled.setCursor(92, 19);
    oled.print(buf);

    oled.setTextSize(1);
    oled.setCursor(2, 39);
    oled.print(s.tempHum);
    oled.setCursor(2, 49);
    oled.print(s.press);

    oled.drawRect(4, 58, 120, 5, SSD1306_WHITE);              // seconds bar
    oled.fillRect(5, 59, (118 * (t.sec + 1)) / 60, 3, SSD1306_WHITE);

    oled.display();
}

// ======================= App =======================
static Bme280  bme;
static Climate climate = { 0, 0, 0, false };
static bool bmeOk = false;

void setup() {
    Serial.begin(115200);
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN, Config::I2C_HZ);
    if (!displayInit()) LOGE("SSD1306 not found at 0x3C");

    DateTime t;
    bool halted = false;
    if (!rtcRead(t, halted) || halted) {
        LOGI("RTC stopped or invalid -> set from build time");
        rtcWrite(compileTime());
    }
    bmeOk = bme.init();
}

void loop() {
    static uint32_t lastRtc = 0, lastBme = 0;
    static int lastSec = -1;
    const uint32_t now = millis();

    // BME280: own timestamp, independent of the clock.
    if (now - lastBme >= Config::BME_POLL_MS) {
        lastBme = now;
        if (!bmeOk) bmeOk = bme.init();               // retry if it was missing
        if (bmeOk) climate = bme.read();
    }

    if (now - lastRtc < Config::RTC_POLL_MS) return;
    lastRtc = now;

    DateTime t;
    bool halted = false;
    if (!rtcRead(t, halted)) { LOGE("RTC read error (0x68)"); lastSec = -1; return; }
    if (t.sec == lastSec) return;                      // redraw once per second
    lastSec = t.sec;

    const ScreenText s = buildText(t, climate);
    displayAll(t, s);
    LOGI("%s %02u:%02u:%02u | %s | %s", s.date, t.hour, t.min, t.sec, s.tempHum, s.press);
}
