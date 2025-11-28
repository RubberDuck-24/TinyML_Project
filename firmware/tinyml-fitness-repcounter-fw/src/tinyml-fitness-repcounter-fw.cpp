#include "Particle.h"
#include "Adafruit_SSD1306_RK.h"

SYSTEM_THREAD(ENABLED);

// ======================
// OLED config (I2C)
// ======================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2C constructor: width, height, &Wire
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// ======================
// ADXL343 (I2C) config
// ======================
const uint8_t ADXL343_ADDR  = 0x53; // ADXL343 with SDO->GND
const uint8_t REG_DEVID     = 0x00;
const uint8_t REG_POWER_CTL = 0x2D;
const uint8_t REG_DATAX0    = 0x32;

// ======================
// Buttons & DIP config
// ======================

// Start/Stop recording button
const int BUTTON_PIN = D6;
bool isActive = false;               // false = paused/idle, true = logging/active
int lastButtonReading = HIGH;
int stableButtonState  = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 50;

// Rep marker button (ground truth reps)
const int REP_BUTTON_PIN = D5;
int lastRepReading = HIGH;
int stableRepState = HIGH;
unsigned long lastRepDebounceTime = 0;
unsigned long repGt = 0;            // ground-truth rep counter

// DIP switch pins for exercise selection
const int DIP_SW0_PIN = A0;         // LSB
const int DIP_SW1_PIN = A1;         // MSB

// ======================
// ADXL343 helpers
// ======================

void adxlWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADXL343_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t adxlRead(uint8_t reg) {
    Wire.beginTransmission(ADXL343_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); // repeated start
    Wire.requestFrom(ADXL343_ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

void adxlReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(ADXL343_ADDR);
    Wire.write(startReg);
    Wire.endTransmission(false); // repeated start
    Wire.requestFrom(ADXL343_ADDR, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) {
        buffer[i++] = Wire.read();
    }
}

// ======================
// Label handling
// ======================

// Label from DIP switches (exercise selection)
String exerciseLabel = "idle";   // "idle", "pushup", "squat", "pullup"

// Optional: serial buffer (still there if you want to debug via serial)
String serialBuffer = "";

void updateLabelFromDip() {
    // Active-low switches: ON = LOW = 1
    int b0 = (digitalRead(DIP_SW0_PIN) == LOW) ? 1 : 0;
    int b1 = (digitalRead(DIP_SW1_PIN) == LOW) ? 1 : 0;

    int mode = (b1 << 1) | b0;

    switch (mode) {
        case 0:
            exerciseLabel = "idle";
            break;
        case 1:
            exerciseLabel = "pushup";
            break;
        case 2:
            exerciseLabel = "squat";
            break;
        case 3:
            exerciseLabel = "pullup";
            break;
        default:
            exerciseLabel = "idle";
            break;
    }
}

// You can still use serial to print debug, but label now comes from DIP.
void handleSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                serialBuffer.trim();
                if (serialBuffer.length() > 0) {
                    // For now we just log the input, but DO NOT override exerciseLabel,
                    // since we use the DIP switches for labeling.
                    Serial.printlnf("# Serial command received: %s", serialBuffer.c_str());
                }
                serialBuffer = "";
            }
        } else {
            if (serialBuffer.length() < 32) {
                serialBuffer += c;
            }
        }
    }
}

// ======================
// OLED update
// ======================

void updateDisplay(float xG, float yG, float zG) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.setCursor(0, 0);
    display.println("TinyML Logger");

    display.setCursor(0, 16);
    if (!isActive) {
        display.println("Status: Paused");
    } else {
        display.print("REC: ");
        display.println(exerciseLabel);
    }

    display.setCursor(0, 32);
    display.print("X: ");
    display.println(xG, 2);
    display.setCursor(0, 40);
    display.print("Y: ");
    display.println(yG, 2);
    display.setCursor(0, 48);
    display.print("Z: ");
    display.println(zG, 2);

    // Optional: show rep GT on bottom right
    display.setCursor(80, 0);
    display.print("Rep:");
    display.println((int)repGt);

    display.display();
}

// ======================
// Button handling (debounced)
// ======================

void handleStartStopButton() {
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonReading) {
        lastDebounceTime = millis();
        lastButtonReading = reading;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
        if (reading != stableButtonState) {
            stableButtonState = reading;

            // Active-low button: pressed when reading == LOW
            if (stableButtonState == LOW) {
                isActive = !isActive;
                Serial.printlnf("# Start/Stop button pressed. isActive = %s", isActive ? "true" : "false");
            }
        }
    }
}

void handleRepButton() {
    int reading = digitalRead(REP_BUTTON_PIN);

    if (reading != lastRepReading) {
        lastRepDebounceTime = millis();
        lastRepReading = reading;
    }

    if ((millis() - lastRepDebounceTime) > DEBOUNCE_MS) {
        if (reading != stableRepState) {
            stableRepState = reading;

            // Active-low: increment rep counter on press
            if (stableRepState == LOW) {
                repGt++;
                Serial.printlnf("# Rep marker pressed. rep_gt = %lu", repGt);
            }
        }
    }
}

// ======================
// setup()
// ======================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("# TinyML fitness data logger starting...");

    // Buttons & DIP
    pinMode(BUTTON_PIN, INPUT_PULLUP);      // start/stop
    pinMode(REP_BUTTON_PIN, INPUT_PULLUP);  // rep marker
    pinMode(DIP_SW0_PIN, INPUT_PULLUP);
    pinMode(DIP_SW1_PIN, INPUT_PULLUP);

    Wire.begin(); // shared I2C bus: ADXL343 + OLED

    // --- ADXL343 init ---
    Serial.println("# Init ADXL343...");
    uint8_t devid = adxlRead(REG_DEVID);
    Serial.printlnf("# ADXL343 DEVID: 0x%02X", devid);
    if (devid != 0xE5) {
        Serial.println("# ERROR: ADXL343 not detected, check wiring.");
        while (true) {
            delay(1000);
        }
    }

    adxlWrite(REG_POWER_CTL, 0x08); // measurement mode
    delay(10);
    Serial.println("# ADXL343 OK.");

    // --- OLED init ---
    Serial.println("# Init OLED...");
    if (!display.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println("# ERROR: SSD1306 init failed");
        while (true) {
            delay(1000);
        }
    }
    display.clearDisplay();
    display.display();

    updateLabelFromDip();  // initial label from DIP
    updateDisplay(0.0f, 0.0f, 1.0f);
    Serial.println("# OLED OK.");

    // CSV header with rep_gt column
    Serial.println("timestamp_ms,ax,ay,az,label,rep_gt");
}

// ======================
// loop()
// ======================

void loop() {
    handleStartStopButton(); // may toggle isActive
    handleRepButton();       // may increment repGt
    handleSerialCommands();  // optional serial debug

    // Always read label from DIP (in case you flip it between sets)
    updateLabelFromDip();

    // --- Read accelerometer ---
    uint8_t rawData[6];
    adxlReadBytes(REG_DATAX0, rawData, 6);

    int16_t xRaw = (int16_t)((rawData[1] << 8) | rawData[0]);
    int16_t yRaw = (int16_t)((rawData[3] << 8) | rawData[2]);
    int16_t zRaw = (int16_t)((rawData[5] << 8) | rawData[4]);

    const float scale_g_per_lsb = 0.004f; // ±2g, 4 mg/LSB
    float xG = xRaw * scale_g_per_lsb;
    float yG = yRaw * scale_g_per_lsb;
    float zG = zRaw * scale_g_per_lsb;

    // Effective label: when paused, force "idle"
    String effectiveLabel = (isActive ? exerciseLabel : "idle");

    // --- CSV data output ---
    Serial.printlnf("%lu,%.4f,%.4f,%.4f,%s,%lu",
                    millis(),
                    xG, yG, zG,
                    effectiveLabel.c_str(),
                    repGt);

    // --- Update OLED ---
    updateDisplay(xG, yG, zG);

    delay(40); // ~25 Hz
}