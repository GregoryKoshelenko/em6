#include <Arduino.h>

static const uint8_t  LDR_PIN = 4;
static const uint8_t  LED_PIN = 2;
// static const uint8_t  POT_PIN = 5;  // optional: potentiometer for max-brightness

static const uint8_t  LEDC_CH   = 0; // LEDC channel to use for PWM output
static const uint32_t LEDC_FREQ = 5000; // 5 kHz PWM frequency
static const uint8_t  LEDC_BITS = 8;    // PWM resolution
static const uint32_t LEDC_MAX  = (1U << LEDC_BITS) - 1U;

static const uint32_t SAMPLE_MS = 20;    // 50 Hz
static const uint32_t PRINT_MS  = 500;

static const float    EMA_ALPHA  = 0.1f; // 0 = heavy smoothing, 1 = raw
static const float    HYST       = 5.0f; // PWM deadzone — prevents flicker at boundary
static const float    FADE_STEP  = 2.0f; // PWM units per tick → ~2.5s full sweep at 50 Hz

static float    emaVal     = 0.0f;
static float    currentPWM = 0.0f;
static float    targetPWM  = 0.0f;
static uint32_t lastSample = 0;
static uint32_t lastPrint  = 0;
static int32_t  rawADC     = 0;
static int32_t  lightMin   = 4095;
static int32_t  lightMax   = 0;

/** Detects open-circuit (LDR disconnected) and short-circuit conditions. */
static bool validateSensor(int32_t raw) {
    if (raw <= 10)   { Serial.println("[WARN] LDR short or missing"); return false; }
    if (raw >= 4090) { Serial.println("[WARN] LDR open circuit");     return false; }
    return true;
}

/** Samples LDR for 3s to find ambient min/max light range for normalization. */
static void autoCalibrate() {
    Serial.println("Calibrating 3s — vary light conditions now...");
    uint32_t start = millis();
    while (millis() - start < 3000) {
        int v = analogRead(LDR_PIN);
        lightMin = min(lightMin, v);
        lightMax = max(lightMax, v);
    }
    if (lightMax - lightMin < 200) {
        lightMin = 0;
        lightMax = 4095;
        Serial.println("Range too narrow — using defaults (0-4095)");
    } else {
        Serial.printf("Calibration done: min=%d  max=%d\n", lightMin, lightMax);
    }
}

/** Applies EMA filter, maps ADC → inverted PWM target, updates hysteresis gate. */
static void updateTarget() {
    emaVal = EMA_ALPHA * rawADC + (1.0f - EMA_ALPHA) * emaVal;

    float clamped    = constrain(emaVal, (float)lightMin, (float)lightMax);
    float normalized = (clamped - lightMin) / (float)(lightMax - lightMin);
    float newTarget  = (1.0f - normalized) * LEDC_MAX; // dark → bright

    // float potScale = analogRead(POT_PIN) / 4095.0f;
    // newTarget *= potScale;

    if (fabsf(newTarget - targetPWM) > HYST) {
        targetPWM = newTarget;
    }
}

/** Steps currentPWM toward targetPWM by FADE_STEP, writes to LEDC. */
static void applyFade() {
    if (currentPWM < targetPWM) currentPWM = min(currentPWM + FADE_STEP, targetPWM);
    else if (currentPWM > targetPWM) currentPWM = max(currentPWM - FADE_STEP, targetPWM);
    ledcWrite(LEDC_CH, (uint32_t)currentPWM);
}

/** Prints raw ADC, EMA-filtered value, and PWM duty cycle to Serial. */
static void printDiagnostics() {
    Serial.printf("Raw=%4d  EMA=%6.1f  PWM=%5.1f%%\n",
                  rawADC, emaVal, (currentPWM / LEDC_MAX) * 100.0f);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    analogReadResolution(12);
    analogSetPinAttenuation(LDR_PIN, ADC_11db);

    ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_BITS);
    ledcAttachPin(LED_PIN, LEDC_CH);
    ledcWrite(LEDC_CH, 0);

    autoCalibrate();
    emaVal = analogRead(LDR_PIN);
    Serial.println("Ready.");
}

void loop() {
    uint32_t now = millis();

    if (now - lastSample >= SAMPLE_MS) {
        lastSample = now;
        rawADC = analogRead(LDR_PIN);
        if (!validateSensor(rawADC)) { 
            ledcWrite(LEDC_CH, 0); 
        }
        else {
            updateTarget();
            applyFade();
        }
        
    }

    if (now - lastPrint >= PRINT_MS) {
        lastPrint = now;
        printDiagnostics();
    }
}
