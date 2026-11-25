// Wokwi custom chip: BME280 over SPI (Wokwi has no built-in BME280).
// Behaves like the real sensor on the bus:
//  - SPI mode 0, first byte = register address, bit 7: 1 = read, 0 = write
//  - reads auto-increment the address; writes are (address, data) pairs
//  - chip id 0xD0 = 0x60, calibration at 0x88..0xA1 and 0xE1..0xE7,
//    raw data at 0xF7..0xFE (press[20], temp[20], hum[16])
// The raw ADC values are found by binary search over the datasheet compensation
// formulas, so the firmware's own compensation returns the slider values.
#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// Calibration (datasheet example values for T/P, typical values for H)
static const uint16_t T1 = 27504; static const int16_t T2 = 26435, T3 = -1000;
static const uint16_t P1 = 36477; static const int16_t P2 = -10685, P3 = 3024, P4 = 2855,
                                   P5 = 140, P6 = -7, P7 = 15500, P8 = -14600, P9 = 6000;
static const uint8_t  H1 = 75;    static const int16_t H2 = 362; static const uint8_t H3 = 0;
static const int16_t  H4 = 313, H5 = 50; static const int8_t H6 = 30;

typedef struct {
  pin_t cs;
  spi_dev_t spi;
  uint8_t buf[1];
  uint8_t regs[256];
  uint8_t addr;
  bool have_addr;
  bool reading;
  uint32_t attr_t, attr_h, attr_p;
} chip_state_t;

// ---------- datasheet compensation (double precision, section 8.1) ----------
static double comp_t(int32_t adc_t, double *t_fine) {
  double v1 = (adc_t / 16384.0 - T1 / 1024.0) * T2;
  double v2 = (adc_t / 131072.0 - T1 / 8192.0) * (adc_t / 131072.0 - T1 / 8192.0) * T3;
  *t_fine = v1 + v2;
  return (v1 + v2) / 5120.0;
}

static double comp_p(int32_t adc_p, double t_fine) {
  double v1 = t_fine / 2.0 - 64000.0;
  double v2 = v1 * v1 * P6 / 32768.0;
  v2 = v2 + v1 * P5 * 2.0;
  v2 = v2 / 4.0 + P4 * 65536.0;
  v1 = (P3 * v1 * v1 / 524288.0 + P2 * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * P1;
  if (v1 == 0.0) return 0;
  double p = 1048576.0 - adc_p;
  p = (p - v2 / 4096.0) * 6250.0 / v1;
  v1 = P9 * p * p / 2147483648.0;
  v2 = p * P8 / 32768.0;
  return p + (v1 + v2 + P7) / 16.0;            // Pa
}

static double comp_h(int32_t adc_h, double t_fine) {
  double h = t_fine - 76800.0;
  h = (adc_h - (H4 * 64.0 + H5 / 16384.0 * h)) *
      (H2 / 65536.0 * (1.0 + H6 / 67108864.0 * h * (1.0 + H3 / 67108864.0 * h)));
  h = h * (1.0 - H1 * h / 524288.0);
  if (h > 100.0) h = 100.0;
  if (h < 0.0) h = 0.0;
  return h;
}

// ---------- inverse: find raw values for the slider values ----------
static void update_raw(chip_state_t *c) {
  const double t = attr_read_float(c->attr_t);
  const double h = attr_read_float(c->attr_h);
  const double p = attr_read_float(c->attr_p) * 100.0;  // hPa -> Pa
  double tf;

  int32_t lo = 0, hi = 0xFFFFF;                 // temperature rises with adc_t
  while (lo < hi) { int32_t m = (lo + hi) / 2; if (comp_t(m, &tf) < t) lo = m + 1; else hi = m; }
  const int32_t adc_t = lo;
  comp_t(adc_t, &tf);

  lo = 0; hi = 0xFFFFF;                          // pressure falls as adc_p rises
  while (lo < hi) { int32_t m = (lo + hi) / 2; if (comp_p(m, tf) > p) lo = m + 1; else hi = m; }
  const int32_t adc_p = lo;

  lo = 0; hi = 0xFFFF;                           // humidity rises with adc_h
  while (lo < hi) { int32_t m = (lo + hi) / 2; if (comp_h(m, tf) < h) lo = m + 1; else hi = m; }
  const int32_t adc_h = lo;

  c->regs[0xF7] = adc_p >> 12; c->regs[0xF8] = (adc_p >> 4) & 0xFF; c->regs[0xF9] = (adc_p & 0x0F) << 4;
  c->regs[0xFA] = adc_t >> 12; c->regs[0xFB] = (adc_t >> 4) & 0xFF; c->regs[0xFC] = (adc_t & 0x0F) << 4;
  c->regs[0xFD] = adc_h >> 8;  c->regs[0xFE] = adc_h & 0xFF;
}

static void put16(uint8_t *r, int a, uint16_t v) { r[a] = v & 0xFF; r[a + 1] = v >> 8; }

static void init_regs(chip_state_t *c) {
  uint8_t *r = c->regs;
  memset(r, 0, sizeof(c->regs));
  r[0xD0] = 0x60;                                // chip id
  put16(r, 0x88, T1); put16(r, 0x8A, (uint16_t)T2); put16(r, 0x8C, (uint16_t)T3);
  put16(r, 0x8E, P1); put16(r, 0x90, (uint16_t)P2); put16(r, 0x92, (uint16_t)P3);
  put16(r, 0x94, (uint16_t)P4); put16(r, 0x96, (uint16_t)P5); put16(r, 0x98, (uint16_t)P6);
  put16(r, 0x9A, (uint16_t)P7); put16(r, 0x9C, (uint16_t)P8); put16(r, 0x9E, (uint16_t)P9);
  r[0xA1] = H1;
  put16(r, 0xE1, (uint16_t)H2);
  r[0xE3] = H3;
  r[0xE4] = (H4 >> 4) & 0xFF;
  r[0xE5] = (uint8_t)((H4 & 0x0F) | ((H5 & 0x0F) << 4));
  r[0xE6] = (H5 >> 4) & 0xFF;
  r[0xE7] = (uint8_t)H6;
}

// ---------- SPI ----------
static void spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *c = (chip_state_t *)user_data;
  if (!count) return;
  const uint8_t b = buffer[0];

  if (!c->have_addr) {                           // first byte of a frame (or a write pair)
    c->have_addr = true;
    c->reading = b & 0x80;
    c->addr = b | 0x80;                          // SPI uses 7 address bits
    if (c->reading) {
      buffer[0] = c->regs[c->addr];              // shifted out during the next byte
      c->addr++;
    } else {
      buffer[0] = 0;
    }
  } else if (c->reading) {
    buffer[0] = c->regs[c->addr];                // auto-increment
    c->addr++;
  } else {
    if (c->addr == 0xE0 && b == 0xB6) init_regs(c);       // soft reset
    else if (c->addr >= 0xF2 && c->addr <= 0xF5) c->regs[c->addr] = b;  // ctrl / config
    c->have_addr = false;                        // next byte is a new address
    buffer[0] = 0;
  }

  if (pin_read(c->cs) == LOW) spi_start(c->spi, c->buf, 1);
}

static void cs_changed(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *c = (chip_state_t *)user_data;
  if (value == LOW) {
    update_raw(c);                               // fresh sample for this frame
    c->have_addr = false;
    c->buf[0] = 0;
    spi_start(c->spi, c->buf, 1);
  } else {
    spi_stop(c->spi);
  }
}

void chip_init(void) {
  chip_state_t *c = malloc(sizeof(chip_state_t));
  memset(c, 0, sizeof(*c));
  c->attr_t = attr_init_float("temperature", 23.4);
  c->attr_h = attr_init_float("humidity", 45.0);
  c->attr_p = attr_init_float("pressure", 1013.0);
  init_regs(c);

  c->cs = pin_init("CSB", INPUT_PULLUP);
  const spi_config_t spi_cfg = {
    .sck = pin_init("SCK", INPUT),
    .mosi = pin_init("SDI", INPUT),
    .miso = pin_init("SDO", INPUT),
    .mode = 0,
    .done = spi_done,
    .user_data = c,
  };
  c->spi = spi_init(&spi_cfg);

  const pin_watch_config_t watch = { .edge = BOTH, .pin_change = cs_changed, .user_data = c };
  pin_watch(c->cs, &watch);
}
