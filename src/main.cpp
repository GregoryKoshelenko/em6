// Lesson 42 (module 4.3): I2C clock. DS1307 RTC (read by registers) -> SSD1306 128x64.
//  - time HH:MM:SS (hours and minutes large, Casio style), date "EEE DD.MM.YYYY"
//  - screen refreshed when the second changes (RTC polled every 100 ms)
//  - optional: seconds progress bar
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

struct Config {
    static constexpr uint8_t  SDA_PIN   = 8;
    static constexpr uint8_t  SCL_PIN   = 9;
    static constexpr uint32_t I2C_HZ    = 400000;
    static constexpr uint8_t  OLED_ADDR = 0x3C;
    static constexpr uint8_t  RTC_ADDR  = 0x68;
    static constexpr uint8_t  WIDTH     = 128;
    static constexpr uint8_t  HEIGHT    = 64;
    static constexpr uint32_t POLL_MS   = 100;
};

struct DateTime {
    uint8_t  sec, min, hour;
    uint8_t  day, month;
    uint16_t year;
};

static Adafruit_SSD1306 oled(Config::WIDTH, Config::HEIGHT, &Wire, -1);

// ---------- DS1307 (registers 0x00-0x06, BCD) ----------
static uint8_t bcdToDec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t decToBcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

static bool rtcRead(DateTime& t, bool& halted) {
    Wire.beginTransmission(Config::RTC_ADDR);
    Wire.write(0x00);                                      // register pointer -> seconds
    if (Wire.endTransmission(false) != 0) return false;   // repeated START
    if (Wire.requestFrom(Config::RTC_ADDR, uint8_t(7)) != 7) return false;

    const uint8_t s = Wire.read();
    halted  = s & 0x80;                                    // CH bit: oscillator stopped
    t.sec   = bcdToDec(s & 0x7F);
    t.min   = bcdToDec(Wire.read() & 0x7F);
    t.hour  = bcdToDec(Wire.read() & 0x3F);                // 24h mode
    Wire.read();                                           // day of week: computed instead
    t.day   = bcdToDec(Wire.read() & 0x3F);
    t.month = bcdToDec(Wire.read() & 0x1F);
    t.year  = 2000 + bcdToDec(Wire.read());
    return t.sec < 60 && t.min < 60 && t.hour < 24 && t.day >= 1 && t.day <= 31
        && t.month >= 1 && t.month <= 12;
}

static bool rtcWrite(const DateTime& t) {
    Wire.beginTransmission(Config::RTC_ADDR);
    Wire.write(0x00);
    Wire.write(decToBcd(t.sec) & 0x7F);                    // CH = 0 -> clock runs
    Wire.write(decToBcd(t.min));
    Wire.write(decToBcd(t.hour));                          // bit 6 = 0 -> 24h
    Wire.write(1);
    Wire.write(decToBcd(t.day));
    Wire.write(decToBcd(t.month));
    Wire.write(decToBcd(t.year % 100));
    return Wire.endTransmission() == 0;
}

// Build time of the firmware, e.g. "Nov 22 2026" + "15:33:59".
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

// 0 = Sunday ... 6 = Saturday (Sakamoto's method).
static uint8_t dayOfWeek(uint16_t y, uint8_t m, uint8_t d) {
    static const uint8_t k[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    if (m < 3) y -= 1;
    return (y + y / 4 - y / 100 + y / 400 + k[m - 1] + d) % 7;
}

// ---------- Display ----------
static bool displayInit() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR)) return false;
    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    oled.display();
    return true;
}

static void displayTime(const DateTime& t) {
    static const char* const DAYS[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    char buf[20];

    oled.clearDisplay();

    // Date: "Sat 14.02.2026", centered on top.
    snprintf(buf, sizeof(buf), "%s %02u.%02u.%04u",
             DAYS[dayOfWeek(t.year, t.month, t.day)], t.day, t.month, t.year);
    oled.setTextSize(1);
    oled.setCursor((Config::WIDTH - strlen(buf) * 6) / 2, 2);
    oled.print(buf);

    // Time: HH:MM large (size 3 = 18x24 px per char), :SS small next to it.
    snprintf(buf, sizeof(buf), "%02u:%02u", t.hour, t.min);
    oled.setTextSize(3);
    oled.setCursor(2, 18);
    oled.print(buf);
    snprintf(buf, sizeof(buf), ":%02u", t.sec);
    oled.setTextSize(2);
    oled.setCursor(92, 26);
    oled.print(buf);

    // Seconds progress bar.
    oled.drawRect(4, 52, 120, 8, SSD1306_WHITE);
    oled.fillRect(6, 54, (116 * (t.sec + 1)) / 60, 4, SSD1306_WHITE);

    oled.display();
}

static void displayError(const char* msg) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 28);
    oled.print(msg);
    oled.display();
}

// ---------- App ----------
void setup() {
    Serial.begin(115200);
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN, Config::I2C_HZ);

    if (!displayInit()) Serial.println("SSD1306 not found at 0x3C");

    DateTime t;
    bool halted = false;
    if (!rtcRead(t, halted) || halted) {
        Serial.println("RTC stopped or invalid -> set from build time");
        rtcWrite(compileTime());
    }
}

void loop() {
    static uint32_t lastPoll = 0;
    static int lastSec = -1;

    const uint32_t now = millis();
    if (now - lastPoll < Config::POLL_MS) { delay(1); return; }
    lastPoll = now;

    DateTime t;
    bool halted = false;
    if (!rtcRead(t, halted)) {
        displayError("RTC read error (0x68)");
        lastSec = -1;
        return;
    }
    if (t.sec == lastSec) return;          // redraw once per second
    lastSec = t.sec;

    displayTime(t);
    Serial.printf("%02u:%02u:%02u %02u.%02u.%04u\n", t.hour, t.min, t.sec, t.day, t.month, t.year);
}
