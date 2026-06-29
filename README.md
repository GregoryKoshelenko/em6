# Embedded C++ — ESP32 Homework 21

## Task 1: Relay Response Time

Measures how long a relay takes to physically close after its coil is energized.
One GPIO triggers the coil, a second GPIO detects the contact closing via interrupt.
10 measurements are taken and the average is printed to Serial Monitor.

### How it works

```
loop() fires relay (GPIO5 HIGH)
    |
    +-- records triggerTime = millis()
    |
    +-- relay coil energizes (~5-15ms later) contact closes
    |
    ISR fires immediately on FALLING edge (GPIO18)
        gContactTime = millis()
        gContactFired = true
    |
    +-- loop() sees gContactFired == true
    +-- elapsed = gContactTime - triggerTime
    +-- prints result, fires relay OFF, waits 500ms, repeats
```

### ESP32 Wiring — Task 1

```
                    ESP32
                 ┌─────────┐
                 │   5V    ├──────────────────── Relay VCC
                 │   GND   ├──────────────────── Relay GND
                 │  GPIO5  ├──────────────────── Relay IN (coil control)
                 │         │
                 │  GPIO18 ├──── 10K ──── 3.3V   (external pullup)
                 │  GPIO18 ├──────────────────── Relay NO contact
                 │         │                     Relay COM ──── GND
                 └─────────┘

Relay module:
  IN  ──── GPIO5   (HIGH = relay ON)
  VCC ──── 5V
  GND ──── GND

Relay contact (feedback):
  NO  ──── GPIO18
  COM ──── GND

When relay activates: NO closes → GPIO18 pulled LOW → FALLING interrupt fires
```

### Expected Serial Output

```
=== Relay Timer + Soft PWM ===
Relay measurements starting...
  [1/10] 8 ms
  [2/10] 7 ms
  [3/10] 9 ms
  [4/10] 8 ms
  [5/10] 8 ms
  [6/10] 7 ms
  [7/10] 9 ms
  [8/10] 8 ms
  [9/10] 8 ms
  [10/10] 7 ms
---------------------
Average: 7 ms
```

---

## Task 2: Software PWM (Optional)

Reads a potentiometer via ADC and generates a software PWM signal on GPIO19.
PWM frequency: 50Hz (20ms period). Duty cycle: 0–100% controlled by potentiometer.

### How it works

```
now % 20ms  →  phase within current period (0..19)
duty = ADC / 4095 * 100%
onTime = 20ms * duty / 100

if phase < onTime  → GPIO19 HIGH
else               → GPIO19 LOW
```

No timer hardware used — the superloop runs fast enough (~3µs/iteration) that
1ms PWM resolution is achievable with millis().

### ESP32 Wiring — Task 2

```
                    ESP32
                 ┌─────────┐
                 │  3.3V   ├──── Pot pin 1 (left)
                 │  GPIO34 ├──── Pot wiper (middle)     ADC input
                 │  GND    ├──── Pot pin 3 (right)
                 │         │
                 │  GPIO19 ├──── Motor driver IN         PWM output
                 └─────────┘

Potentiometer (10K):
  Pin 1  ──── 3.3V
  Pin 2  ──── GPIO34 (ADC1_CH6)
  Pin 3  ──── GND

Motor driver (e.g. L298N, DRV8833):
  IN1    ──── GPIO19
  VCC    ──── external power supply
  GND    ──── common GND with ESP32
```

### Expected Serial Output

```
PWM duty: 0%
PWM duty: 12%
PWM duty: 35%
PWM duty: 67%
PWM duty: 89%
PWM duty: 100%
```

---

## Full Wiring Diagram

```
        3.3V ──┬──────────────────── Pot pin 1
               │        10K
        GND  ──┼──────────────────── Pot pin 3
               │
               │         ESP32
               │      ┌──────────┐
               └──────┤  3.3V    │
                      │  GND     ├──── common GND
                      │          │
                      │  GPIO5   ├──────────────── Relay IN
                      │  GPIO18  ├──────────────── Relay NO contact
                      │          │                 Relay COM ──── GND
                      │          │
                      │  GPIO34  ├──────────────── Pot wiper
                      │  GPIO19  ├──────────────── Motor driver IN
                      └──────────┘

Relay module:
  VCC ──── 5V (from ESP32 5V pin or external)
  GND ──── GND
  IN  ──── GPIO5
  NO  ──── GPIO18
  COM ──── GND

Potentiometer (10K):
  Left  ──── 3.3V
  Mid   ──── GPIO34
  Right ──── GND
```

## Pin Summary

| Pin    | Direction | Function              |
|--------|-----------|-----------------------|
| GPIO5  | OUTPUT    | Relay coil control    |
| GPIO18 | INPUT     | Relay contact feedback|
| GPIO34 | INPUT ADC | Potentiometer wiper   |
| GPIO19 | OUTPUT    | Software PWM output   |

## Notes

- GPIO34 is input-only on ESP32 — no internal pullup, ADC only.
- Use a relay module with built-in transistor driver (most 5V relay modules work).
- GPIO18 uses `INPUT_PULLUP` — contact must pull to GND when closed (NO + COM to GND).
- millis() resolution is 1ms, so relay timing accuracy is ±1ms.
