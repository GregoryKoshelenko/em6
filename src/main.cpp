// Lesson 41 (module 4.2): I2C intro. SSD1306 init plan, checked on a real bus.
// No display library: the init sequence from docs/hw41-i2c-ssd1306-plan.md is sent
// with Wire only, then a test frame (border + diagonals + checker) is drawn.
// Also scans the bus: expects SSD1306 at 0x3C and DS1307 RTC at 0x68.
#include <Arduino.h>
#include <Wire.h>

struct Config {
    static constexpr uint8_t  SDA_PIN    = 8;
    static constexpr uint8_t  SCL_PIN    = 9;
    static constexpr uint32_t I2C_HZ     = 400000;
    static constexpr uint8_t  OLED_ADDR  = 0x3C;
    static constexpr uint8_t  WIDTH      = 128;
    static constexpr uint8_t  PAGES      = 8;     // 64 rows / 8
    static constexpr uint8_t  DATA_CHUNK = 16;    // bytes per data transaction
};

namespace ssd1306 {
constexpr uint8_t CTRL_CMD  = 0x00;
constexpr uint8_t CTRL_DATA = 0x40;

// Steps 1-17 of the plan.
constexpr uint8_t INIT[] = {
    0xAE,         // display off
    0xD5, 0x80,   // clock divide / oscillator
    0xA8, 0x3F,   // multiplex 64
    0xD3, 0x00,   // display offset 0
    0x40,         // start line 0
    0x8D, 0x14,   // charge pump on
    0x20, 0x00,   // horizontal addressing
    0xA1,         // segment remap
    0xC8,         // COM scan descending
    0xDA, 0x12,   // COM pins for 64 rows
    0x81, 0xCF,   // contrast
    0xD9, 0xF1,   // pre-charge
    0xDB, 0x40,   // VCOMH
    0xA4,         // show RAM
    0xA6,         // normal (not inverted)
    0x2E,         // scroll off
    0xAF,         // display on
};

// Returns Wire's status: 0 = OK, 2 = address NACK, 3 = data NACK, ...
uint8_t commands(const uint8_t* cmd, size_t len) {
    Wire.beginTransmission(Config::OLED_ADDR);
    Wire.write(CTRL_CMD);
    Wire.write(cmd, len);
    return Wire.endTransmission();
}

uint8_t data(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i += Config::DATA_CHUNK) {
        Wire.beginTransmission(Config::OLED_ADDR);
        Wire.write(CTRL_DATA);
        Wire.write(buf + i, min<size_t>(Config::DATA_CHUNK, len - i));
        const uint8_t err = Wire.endTransmission();
        if (err) return err;
    }
    return 0;
}
}  // namespace ssd1306

static uint8_t frame[Config::WIDTH * Config::PAGES];

static void setPixel(int x, int y) {
    if (x < 0 || x >= Config::WIDTH || y < 0 || y >= Config::PAGES * 8) return;
    frame[(y / 8) * Config::WIDTH + x] |= 1 << (y % 8);
}

static void drawTestFrame() {
    memset(frame, 0, sizeof(frame));
    for (int x = 0; x < 128; ++x) { setPixel(x, 0); setPixel(x, 63); setPixel(x, x / 2); setPixel(x, 63 - x / 2); }
    for (int y = 0; y < 64; ++y)  { setPixel(0, y); setPixel(127, y); }
    for (int y = 24; y < 40; ++y)                       // 16x16 checker in the middle
        for (int x = 56; x < 72; ++x)
            if (((x / 4) + (y / 4)) % 2) setPixel(x, y);
}

static void scanBus() {
    Serial.print("I2C scan:");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", addr);
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN, Config::I2C_HZ);
    scanBus();

    uint8_t err = ssd1306::commands(ssd1306::INIT, sizeof(ssd1306::INIT));
    Serial.printf("init: %s (%u bytes)\n", err ? "NACK" : "OK", unsigned(sizeof(ssd1306::INIT)));
    if (err) return;

    const uint8_t window[] = { 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07 };  // columns 0-127, pages 0-7
    drawTestFrame();
    err = ssd1306::commands(window, sizeof(window));
    if (!err) err = ssd1306::data(frame, sizeof(frame));
    Serial.printf("frame: %s (%u bytes)\n", err ? "NACK" : "OK", unsigned(sizeof(frame)));
}

void loop() {
    delay(100);  // nothing to do; the display keeps its RAM
}
