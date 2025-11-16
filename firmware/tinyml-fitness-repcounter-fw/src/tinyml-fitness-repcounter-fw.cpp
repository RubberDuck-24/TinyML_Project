/* 
 * Project TinyML Fitness RepCounter
 * Author: Christian Rasmussen
 * Date: 02/10/2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

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

// Helper: write one byte to a register
void adxlWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADXL343_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Helper: read one byte from a register
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

// Helper: read multiple bytes starting at a register
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
// Label + serial commands
// ======================
String currentLabel = "idle";   // default
String serialBuffer = "";       // accumulate chars from Serial

void updateDisplay(float xG, float yG, float zG) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.setCursor(0, 0);
    display.println("TinyML Logger");

    display.setCursor(0, 16);
    if (currentLabel == "idle") {
        display.println("Status: Idle");
    } else {
        display.print("REC: ");
        display.println(currentLabel);
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

    display.display();
}

void handleSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                serialBuffer.trim();
                if (serialBuffer.length() > 0) {
                    currentLabel = serialBuffer;
                    Serial.printlnf("# Label changed to: %s", currentLabel.c_str());
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
// setup()
// ======================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("# TinyML fitness data logger starting...");

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
    updateDisplay(0.0f, 0.0f, 1.0f);
    Serial.println("# OLED OK.");

    // CSV header
    Serial.println("timestamp_ms,ax,ay,az,label");
}

// ======================
// loop()
// ======================
void loop() {
    handleSerialCommands(); // may change currentLabel

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

    // --- CSV data output ---
    Serial.printlnf("%lu,%.4f,%.4f,%.4f,%s",
                    millis(),
                    xG, yG, zG,
                    currentLabel.c_str());

    // --- Update OLED ---
    updateDisplay(xG, yG, zG);

    delay(40); // ~25 Hz
}