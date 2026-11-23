/*
  HW675 ESP32-C3: AS5600 wind vane + Pelco-D DRV8833 pan/tilt.
  Board: ESP32C3 Dev Module; USB CDC On Boot ENABLED.
  WiFi hotspot: WindVane / vane2026test. Open http://192.168.4.1/.
  Optional home-router connection: fill HOME_WIFI_SSID/PASSWORD below.
  WindVane hotspot stays available. Its page shows the home-router IP when joined.
  From home WiFi, open that home IP (not 192.168.4.1).
  No internet required; all page assets are stored in this sketch.
  /update: password-protected browser firmware upload (admin / vane2026update).
  Change OTA_PASSWORD below. One USB upload with an OTA partition layout is
  required first. Future uploads use Export Compiled Binary > application .ino.bin.
  Pan/tilt disarms and heater switches off during uploads. USB remains available.
  Change WIFI_AP_PASSWORD below before permanent installation.
  Web compass uses WEB_DIRECTION_OFFSET_DEG=-125 to match original receiver.
  Wired packets/OLED remain uncalibrated, preserving existing receiver behavior.
  Web server runs in a separate task; it never controls motor pins or I2C.
  Revision: higher-priority 8KB web task + 2ms main-loop yield for C3 scheduling.
  USB diagnostics every 5s report connected clients, server, requests and heap.
  /health serves a tiny plain-text response for connection troubleshooting.
  Shared read-only weather snapshots are protected by a short critical section.
  Library: U8g2. ESP32 Arduino core 2.x/3.x/4.x PWM APIs supported.

  GPIO5 SDA / GPIO6 SCL: existing OLED and AS5600, 3.3V I2C.
  OLED visible area: x=28..99, y=24..63; same working layout.
  GPIO10: weather TX -> existing MAX485 DI, 115200 8N1.
  Weather MAX485: VCC=5V, DE=HIGH, /RE=HIGH, RO unconnected.
  GPIO7: camera RX at 9600 8N1, via a 3.3V-compatible receiver/level shifter.
  DO NOT connect RS485 A/B or a 5V MAX485 RO directly to GPIO7.
  GPIO0 -> DRV8833 AIN1 (old Pro Mini D3)
  GPIO1 -> DRV8833 AIN2 (old D11)
  GPIO3 -> DRV8833 BIN1 (old D9)
  GPIO4 -> DRV8833 BIN2 (old D10)
  Keep motor power, motor outputs and nSLEEP arrangement as in working unit.
  Common grounds required. C3 controls logic inputs, not motor power.
  GPIO9 is unused UART1 RX: leave unconnected. GPIO21 is unused UART0 TX.

  SET PRESET 4 + 14 in either order: arm; SET 5 or 15: disarm/brake.
  SET 1/2/3: PWM 180/220/255. Starts disarmed; checksum and STOP retained.
  Web pointer: polls up to 10Hz, frame-rate animation with ~70ms smoothing;
  shortest-path wrap through north. Wired data remains unfiltered at 4Hz.
  Web camera status shows commanded pan/tilt outputs and heater request state.
  Zoom displays observed Pelco-D tele/wide/stop only, not lens feedback.
  If camera zoom is internal and not sent over serial, shows Not reported.
  Single-axis directions only, same as original; PWM about 31.37kHz.
  MOTION_WATCHDOG_MS remains 0, matching supplied working sketch.
  Heater: GPIO2 -> existing active-HIGH MOSFET gate/control input.
  SET 19 = heater ON; SET 20 = OFF; starts OFF after reset.
  Keep existing gate resistor/pull-down and common ground. MOSFET must accept
  3.3V gate drive. GPIO2 is also a strap pin; test cold boot after wiring.

  SHT3x on shared I2C: detects 0x44 or 0x45; reads once/second, checks CRC.
  Sends <roundedDirection,0,tempF,humidity> + CR/LF every 250ms to weather
  UART and USB. Until first valid SHT reading only, fallback=75F/50% RH.
  Thereafter retains last valid T/RH if SHT is disconnected; no extra serial text.
  OLED OFF by default; 2 net full turns CW or CCW within a rolling 15 seconds
  wakes it for exactly 2 minutes. Rocking is not accumulated as full turns.
  A new gesture is needed after timeout; spinning while ON does not extend it.
  AS5600 is sampled every 50ms even when OLED is off; sensor loss/gaps reset
  gesture tracking. Keep hand rotation moderate (less than half a turn/sample).
  Off uses SSD1306 power-save; all weather and camera tasks remain active.
  Existing receiver direction calibration remains there; none applied here.
  Last valid direction retained on sensor errors; no output before first valid
  read. No permanent AS5600 configuration writes. WiFi hotspot serves a local live dashboard; no Bluetooth.
*/
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Enable Tools > USB CDC On Boot before compiling."
#endif

// HOME ROUTER: enter your 2.4GHz WiFi details between the quotes.
// Leave HOME_WIFI_SSID empty to use only the existing WindVane hotspot.
constexpr char HOME_WIFI_SSID[] = "your router";
constexpr char HOME_WIFI_PASSWORD[] = "your password";

// Wireless firmware updates: dashboard link or http://<device-IP>/update.
// Change this password before the first USB upload; keep it in future sketches.
constexpr char OTA_USERNAME[] = "admin";
constexpr char OTA_PASSWORD[] = "vane2026update";
// First USB upload: choose a partition scheme WITH OTA (two app slots).
// Future updates: Sketch > Export Compiled Binary, upload sketch .ino.bin only.
// Keep this updater and your WiFi settings in every replacement sketch.
portMUX_TYPE otaMux = portMUX_INITIALIZER_UNLOCKED;
bool otaControlRequested = false;
bool otaControlStopped = false;

bool otaControlsLocked() {
  portENTER_CRITICAL(&otaMux);
  const bool locked = otaControlRequested;
  portEXIT_CRITICAL(&otaMux);
  return locked;
}

constexpr uint8_t SDA_PIN = 5;
constexpr uint8_t SCL_PIN = 6;
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr int SCREEN_LEFT = 28;
constexpr int SCREEN_WIDTH = 72;
constexpr int WEATHER_TX_PIN = 10;
int lastTempF = 75; // Startup fallback until first CRC-checked SHT3x reading.
int lastHumidity = 50;
bool haveTemperatureHumidity = false;
float lastTempC = 0;
float lastHumidityPct = 0;
uint32_t lastShtValidMs = 0;
uint32_t lastDirectionValidMs = 0;
constexpr unsigned long TX_INTERVAL_MS = 250;
bool haveDirection = false;
int lastDirectionDeg = 0;
unsigned long lastTx = 0;

void sendWeather() {
  if (!haveDirection) return;
  char packet[48];
  snprintf(packet, sizeof(packet), "<%d,0,%d,%d>\r\n",
           lastDirectionDeg, lastTempF, lastHumidity);
  Serial1.print(packet);
  Serial.print(packet);
}


constexpr uint8_t CAMERA_RX_PIN = 7;
constexpr uint8_t MOTOR_PINS[4] = {0, 1, 3, 4}; // AIN1, AIN2, BIN1, BIN2
constexpr uint32_t MOTOR_PWM_HZ = 31372;
constexpr int HEATER_PIN = 2; // Active-HIGH MOSFET gate/control input.
// GPIO2 is also a C3 strap pin (not the boot-mode selector). Use a gate-to-source
// pull-down so the heater is OFF while GPIO is high-impedance at power-up.
// Check cold startup after wiring. MOSFET must switch fully with a 3.3V gate.
bool pwmReady = false;
bool motorAttached[4] = {false, false, false, false};
bool heaterOn = false;
bool zoomReported = false;
uint8_t zoomCommand = 0; // 0 stop, 1 tele/in, 2 wide/out; observed serial commands only.
const char *motionName = "Stop";

void motorDuty(uint8_t index, uint8_t duty) {
  const uint8_t pin = MOTOR_PINS[index];
  if (!motorAttached[index]) { digitalWrite(pin, HIGH); return; }
#if ESP_ARDUINO_VERSION_MAJOR >= 4
  ledcWrite(pin, (duty == 255) ? 256 : duty);
#elif ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(index, duty);
#endif
}

void initMotors() {
  // Both inputs HIGH = brake. Start disarmed, and refuse motion if PWM fails.
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(MOTOR_PINS[i], HIGH);
    pinMode(MOTOR_PINS[i], OUTPUT);
  }
  bool ok = true;
  for (uint8_t i = 0; i < 4; ++i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    motorAttached[i] = ledcAttach(MOTOR_PINS[i], MOTOR_PWM_HZ, 8);
#else
    motorAttached[i] = ledcSetup(i, MOTOR_PWM_HZ, 8) > 0;
    if (motorAttached[i]) ledcAttachPin(MOTOR_PINS[i], i);
#endif
    if (!motorAttached[i]) ok = false;
    motorDuty(i, 255);
  }
  pwmReady = ok;
}

void setHeater(bool on) {
  heaterOn = on;
  if (HEATER_PIN >= 0) digitalWrite(HEATER_PIN, on ? HIGH : LOW);
}

const unsigned long SERIAL_FRAME_GAP_MS = 40;  // discard incomplete packet after a gap
// Set to 0 to disable until repeat-command behavior has been confirmed.
// For repeated commands, try 1500 ms; a valid STOP always acts immediately.
const unsigned long MOTION_WATCHDOG_MS = 0;

uint8_t packet[7];
uint8_t bytesUsed = 0;
unsigned long lastByteMs = 0;
unsigned long lastMotionCommandMs = 0;
uint8_t pwmSpeed = 255;
bool allowMotion = false;
bool sawPreset4 = false;
bool sawPreset14 = false;
bool motorsRunning = false;

void stopMotors() {
  for (uint8_t i = 0; i < 4; ++i) motorDuty(i, 255);
  motorsRunning = false;
  motionName = "Stop";
}

void disarm() {
  allowMotion = false;
  sawPreset4 = false;
  sawPreset14 = false;
  stopMotors();                    // unlike original, immediately stop
}

void moveMotors(uint8_t command2) {
  if (!allowMotion || !pwmReady) { stopMotors(); return; }
  // Brake both axes first. Preserve original direction and PWM behavior.
  stopMotors();
  switch (command2) {
    case 0x04: motorDuty(0, 0); motorDuty(1, pwmSpeed); motionName="Left"; break;
    case 0x02: motorDuty(1, 0); motorDuty(0, pwmSpeed); motionName="Right"; break;
    case 0x10: motorDuty(2, 0); motorDuty(3, pwmSpeed); motionName="Down"; break;
    case 0x08: motorDuty(3, 0); motorDuty(2, pwmSpeed); motionName="Up"; break;
    default: return;
  }
  motorsRunning = true;
  lastMotionCommandMs = millis();
}

void handlePacket(const uint8_t *p) {
  if (otaControlsLocked()) return; // No arming, movement or heater commands during OTA.
  // p[0] 0xFF, p[1] address, p[2] cmd1, p[3] cmd2,
  // p[4] data1, p[5] data2, p[6] checksum.
  // This unit previously accepted any address: preserve that behavior.

  // Observe standard Pelco-D zoom bits without controlling the camera lens.
  // Extended commands have cmd2 bit0=1; never decode their bits as zoom.
  if (p[2] == 0 && !(p[3] & 0x01)) {
    const uint8_t zoomBits = p[3] & 0x60;
    if (zoomBits == 0x20 || zoomBits == 0x40) {
      zoomReported = true;
      zoomCommand = (zoomBits == 0x20) ? 1 : 2;
    } else if (zoomBits == 0 && zoomReported) {
      zoomCommand = 0;
    }
  }

  // Standard Pelco-D STOP (both command bytes and data bytes zero).
  // Repeated STOP commands are harmless and always brake immediately.
  if (p[2] == 0 && p[3] == 0 && p[4] == 0 && p[5] == 0) {
    stopMotors();
    return;
  }

  // Pelco-D SET PRESET command only. Do not mistake other extended
  // commands with coincidentally matching data for an arming preset.
  if (p[2] == 0 && p[3] == 0x07 && p[4] == 0) {
    switch (p[5]) {
      case 1: pwmSpeed = 180; break;
      case 2: pwmSpeed = 220; break;
      case 3: pwmSpeed = 255; break;
      case 4: sawPreset4 = true; break;
      case 14: sawPreset14 = true; break;
      case 5:
      case 15:
        disarm();
        return;
      case 19: setHeater(true); break;
      case 20: setHeater(false); break;
    }
    if (sawPreset4 && sawPreset14) {
      allowMotion = true;
      sawPreset4 = false;
      sawPreset14 = false;
    }
    return;
  }

  // Standard single-axis movement. The original software ignored command1.
  // Reject commands with unsupported command1 flags for additional safety.
  if (p[2] == 0 && (p[3] == 0x02 || p[3] == 0x04 ||
                    p[3] == 0x08 || p[3] == 0x10)) {
    // Data fields (speed) may legitimately be 0 for some controllers.
    // Match original motor-speed-from-preset behavior.
    moveMotors(p[3]);
  }
}

bool checksumValid(const uint8_t *p) {
  // Pelco-D = low eight bits of sum(address, cmd1, cmd2, data1, data2).
  uint8_t sum = 0;
  for (uint8_t i = 1; i <= 5; ++i) sum += p[i];
  return sum == p[6];
}

void feedByte(uint8_t value) {
  unsigned long now = millis();
  if (bytesUsed && (unsigned long)(now - lastByteMs) > SERIAL_FRAME_GAP_MS)
    bytesUsed = 0;
  lastByteMs = now;

  if (!bytesUsed) {
    if (value != 0xFF) return;
    packet[bytesUsed++] = value;
    return;
  }
  packet[bytesUsed++] = value;
  if (bytesUsed < sizeof(packet)) return;

  if (checksumValid(packet)) {
    handlePacket(packet);
    bytesUsed = 0;
    return;
  }
  // Invalid packet: scan its remaining bytes for another possible 0xFF
  // start marker, retaining any trailing bytes rather than combining
  // arbitrarily with later packets.
  uint8_t start = 1;
  while (start < sizeof(packet) && packet[start] != 0xFF) ++start;
  if (start == sizeof(packet)) {
    bytesUsed = 0;
  } else {
    bytesUsed = sizeof(packet) - start;
    for (uint8_t i = 0; i < bytesUsed; ++i)
      packet[i] = packet[start + i];
  }
}


// HW-I2C argument order: rotation, reset, clock, data.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(
    U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);

bool readRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, count) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

void centered(const char *text, int baseline) {
  int x = SCREEN_LEFT + (SCREEN_WIDTH - oled.getStrWidth(text)) / 2;
  oled.drawStr(x, baseline, text);
}

void showError(const char *line1, const char *line2) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  centered("AS5600", 33);
  centered(line1, 47);
  centered(line2, 61);
  oled.sendBuffer();
}

// --- OLED wake gesture: unwrapped signed angle in a rolling 15s history ---
constexpr uint32_t ANGLE_INTERVAL_MS = 50;
constexpr uint32_t WAKE_WINDOW_MS = 15000;
constexpr uint32_t OLED_ON_MS = 120000;
constexpr int64_t WAKE_COUNTS = 2LL * 4096;
constexpr uint16_t HISTORY_SIZE = WAKE_WINDOW_MS / ANGLE_INTERVAL_MS + 2;
struct TurnSample { uint32_t time; int64_t position; };
TurnSample turnHistory[HISTORY_SIZE];
uint16_t turnStart = 0, turnCount = 0;
bool havePreviousAngle = false;
uint16_t previousAngle = 0;
uint32_t previousAngleMs = 0;
int64_t turnPosition = 0;
bool oledOn = false;
uint32_t oledStartedMs = 0;
uint32_t lastAnglePoll = 0;
uint8_t lastMagnetStatus = 0;
float lastDegrees = 0;
bool angleCurrentlyValid = false;
const char *angleError = "Starting";

void resetGesture() {
  turnStart = turnCount = 0;
  havePreviousAngle = false;
  turnPosition = 0;
}

void serviceOledTimeout(uint32_t now) {
  if (oledOn && uint32_t(now - oledStartedMs) >= OLED_ON_MS) {
    oled.setPowerSave(1);
    oledOn = false;
    resetGesture();
  }
}

void trackTurns(uint16_t raw, uint32_t now) {
  // Never reuse turns made while the screen was already awake.
  if (oledOn) { resetGesture(); return; }
  if (havePreviousAngle && uint32_t(now - previousAngleMs) > 250)
    resetGesture(); // discontinuity: cannot infer turns through a sampling gap
  if (havePreviousAngle) {
    int delta = int(raw) - int(previousAngle);
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    turnPosition += delta;
  }
  previousAngle = raw;
  previousAngleMs = now;
  havePreviousAngle = true;

  while (turnCount &&
         uint32_t(now - turnHistory[turnStart].time) > WAKE_WINDOW_MS) {
    turnStart = (turnStart + 1) % HISTORY_SIZE;
    --turnCount;
  }
  // Signed displacement prevents repeated small back-and-forth motions waking it.
  for (uint16_t i = 0; i < turnCount; ++i) {
    int64_t difference = turnPosition - turnHistory[(turnStart+i)%HISTORY_SIZE].position;
    if (difference >= WAKE_COUNTS || difference <= -WAKE_COUNTS) {
      oledOn = true;
      oledStartedMs = now;
      // The next render sends the current image before waking the panel.
      resetGesture();
      return;
    }
  }
  if (turnCount == HISTORY_SIZE) {
    turnStart = (turnStart + 1) % HISTORY_SIZE;
    --turnCount;
  }
  const uint16_t slot = (turnStart + turnCount) % HISTORY_SIZE;
  turnHistory[slot].time = now;
  turnHistory[slot].position = turnPosition;
  ++turnCount;
}

// --- SHT3x: single-shot, high repeatability, no clock stretching ---
uint8_t shtAddress = 0;
bool shtPending = false;
uint32_t shtStartedMs = 0;
uint32_t lastShtAttempt = 0;
bool shtTried = false;

uint8_t shtCrc(const uint8_t *data) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < 2; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x31) : uint8_t(crc << 1);
  }
  return crc;
}

bool startShtMeasurement(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x24);
  Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

bool acceptShtReading(const uint8_t *data) {
  if (shtCrc(data) != data[2] || shtCrc(data+3) != data[5]) return false;
  const uint16_t t = (uint16_t(data[0]) << 8) | data[1];
  const uint16_t h = (uint16_t(data[3]) << 8) | data[4];
  lastTempC = -45.0f + 175.0f * t / 65535.0f;
  lastHumidityPct = 100.0f * h / 65535.0f;
  lastShtValidMs = millis();
  lastTempF = int(lroundf(lastTempC * 1.8f + 32.0f));
  lastHumidity = int(lroundf(lastHumidityPct));
  haveTemperatureHumidity = true;
  return true;
}

void serviceSht(uint32_t now) {
  if (shtPending) {
    if (uint32_t(now - shtStartedMs) < 20) return;
    shtPending = false;
    uint8_t data[6];
    if (Wire.requestFrom(shtAddress, uint8_t(6)) != 6) {
      while (Wire.available()) Wire.read();
      shtAddress = 0;
      return;
    }
    for (uint8_t i = 0; i < 6; ++i) data[i] = Wire.read();
    if (!acceptShtReading(data)) shtAddress = 0;
    return;
  }
  if (shtTried && uint32_t(now - lastShtAttempt) < 1000) return;
  shtTried = true;
  lastShtAttempt = now;
  if (shtAddress) {
    if (!startShtMeasurement(shtAddress)) { shtAddress = 0; return; }
  } else if (startShtMeasurement(0x44)) shtAddress = 0x44;
  else if (startShtMeasurement(0x45)) shtAddress = 0x45;
  else return;
  shtStartedMs = millis();
  shtPending = true;
}

void sampleDirection(uint32_t now) {
  uint8_t status, angleBytes[2];
  if (!readRegisters(0x0B, &status, 1) ||
      !readRegisters(0x0C, angleBytes, 2)) {
    angleCurrentlyValid = false;
    angleError = "I2C error";
    resetGesture();
    return;
  }
  if (!(status & 0x20)) {
    angleCurrentlyValid = false;
    angleError = "No magnet";
    resetGesture();
    return;
  }
  uint16_t raw = ((uint16_t(angleBytes[0]) << 8) | angleBytes[1]) & 0x0FFF;
  lastDegrees = raw * (360.0f / 4096.0f);
  lastDirectionDeg = int(lastDegrees + 0.5f) % 360;
  haveDirection = angleCurrentlyValid = true;
  lastMagnetStatus = status;
  lastDirectionValidMs = now;
  if (status & 0x18) resetGesture(); // unreliable field: don't infer a wake gesture
  else trackTurns(raw, now);
}

void renderOled() {
  if (!oledOn) return; // No display traffic at all during normal dark operation.
  if (!angleCurrentlyValid) {
    showError(angleError, "Check sensor");
    oled.setPowerSave(0);
    return;
  }
  char text[16];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  if (haveTemperatureHumidity)
    snprintf(text, sizeof(text), "%dF %d%%", lastTempF, lastHumidity);
  else snprintf(text, sizeof(text), "SHT pending");
  centered(text, 33);
  oled.setFont(u8g2_font_logisoso16_tf);
  snprintf(text, sizeof(text), "%.1f", lastDegrees);
  int width = oled.getStrWidth(text);
  int x = SCREEN_LEFT + (SCREEN_WIDTH - (width+7))/2;
  oled.drawStr(x, 53, text);
  oled.drawCircle(x+width+4, 38, 2);
  oled.setFont(u8g2_font_6x10_tr);
  if (lastMagnetStatus & 0x08) snprintf(text, sizeof(text), "Too strong");
  else if (lastMagnetStatus & 0x10) snprintf(text, sizeof(text), "Too weak");
  else if (!pwmReady) snprintf(text, sizeof(text), "PWM ERROR");
  else if (!allowMotion) snprintf(text, sizeof(text), "Disarmed");
  else snprintf(text, sizeof(text), "ARM %s", motionName);
  centered(text, 63);
  oled.sendBuffer();
  oled.setPowerSave(0);
}

// --- Local WiFi weather dashboard ---
constexpr char WIFI_AP_SSID[] = "WindVane";
constexpr char WIFI_AP_PASSWORD[] = "vane2026test";
constexpr float WEB_DIRECTION_OFFSET_DEG = -125.0f;
constexpr bool WEB_DIAGNOSTICS = true; // USB only; GPIO10 weather stream stays clean.
volatile uint32_t webRequests = 0;
WebServer webServer(80);
portMUX_TYPE webMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t webTaskHandle = nullptr;
bool webReady = false;
struct WebSnapshot {
  bool haveAngle, angleValid, haveSht, armed;
  float direction, tempC, humidity;
  uint8_t magnet;
  uint32_t angleMs, shtMs;
  bool homeConnected;
  uint8_t pan, tilt, zoom;
  bool zoomSeen, heaterRequested, heaterAssigned;
  uint8_t homeIp[4];
};
WebSnapshot webSnapshot = {};
uint32_t lastWebSnapshot = 0;

const char WEATHER_PAGE[] PROGMEM = R"WINDPAGE(
<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#f2f5f4"><title>Wind vane</title>
<style>
:root{color-scheme:light;--ink:#18343b;--muted:#62777b;--line:#dce6e4;--teal:#087f79;--red:#dd6151}*{box-sizing:border-box}body{margin:0;background:#f2f5f4;color:var(--ink);font:16px system-ui,-apple-system,Segoe UI,sans-serif}main{max-width:760px;margin:auto;padding:28px 20px 36px}header{display:flex;justify-content:space-between;align-items:center;margin-bottom:22px}h1{font-size:24px;letter-spacing:-.8px;margin:2px 0}.eyebrow{text-transform:uppercase;font-size:11px;letter-spacing:2px;color:var(--muted);font-weight:650}.connection{font-size:12px;display:flex;gap:7px;align-items:center}.dot{height:7px;width:7px;border-radius:50%;background:#adbdba}.live .dot{background:var(--teal)}.card{background:white;border:1px solid var(--line);border-radius:22px;padding:24px;box-shadow:0 5px 25px #18343b05}.direction-card{text-align:center}.section-heading{text-align:left;display:flex;justify-content:space-between;align-items:center;font-size:13px;color:var(--muted)}.tag{background:#edf6f3;color:var(--teal);padding:5px 10px;border-radius:20px;font-size:11px}.dial{width:min(100%,340px);margin:8px auto 0;display:block;overflow:visible}.dial text{font-family:system-ui,sans-serif;fill:var(--muted)}.dial .cardinal{font-size:16px;font-weight:700;fill:var(--ink)}.dial .minor{font-size:11px}.dial .north{fill:var(--red)}#needle{opacity:.2}#needle.valid{opacity:1}.reading{font-size:46px;font-weight:650;letter-spacing:-2px;font-variant-numeric:tabular-nums;line-height:1.15;margin-top:2px}.bearing{color:var(--muted);font-size:14px;margin:7px 0 0}.sensor-note{color:var(--muted);font-size:12px;margin-top:17px;min-height:18px}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:14px}.metric .label{font-size:13px;color:var(--muted);display:flex;align-items:center;gap:8px}.icon{height:18px;width:18px;fill:none;stroke:var(--teal);stroke-width:1.7}.metric-value{font-size:34px;letter-spacing:-1px;font-weight:600;margin:12px 0 4px;font-variant-numeric:tabular-nums}.unit{font-size:18px;color:var(--muted);font-weight:400;letter-spacing:0}.caption{font-size:12px;color:var(--muted);min-height:18px}.switch{border:1px solid var(--line);background:white;color:var(--muted);border-radius:8px;padding:4px 7px;font:inherit;font-size:12px;cursor:pointer;float:right}.statusbar{display:flex;justify-content:space-between;flex-wrap:wrap;gap:8px;color:var(--muted);font-size:11px;margin:19px 4px 0}.error{color:#ad4c31}.calibration{font-size:11px;color:var(--muted);margin:14px 4px 0;line-height:1.6}.camera-state{margin-top:14px;padding:13px 16px;background:white;border:1px solid var(--line);border-radius:14px;font-size:12px;line-height:1.8;display:flex;flex-wrap:wrap;column-gap:18px;row-gap:3px}.camera-state .label{color:var(--muted)}.stale{opacity:.65}@media(max-width:420px){main{padding:20px 14px}.card{padding:19px}.reading{font-size:40px}.metric-value{font-size:30px}.metrics{gap:10px}.connection{font-size:11px}}
</style></head><body><main>
<header><div><div class="eyebrow">Rooftop weather</div><h1>Wind vane</h1></div><div class="connection" id="connection" role="status"><span class="dot"></span><span id="connectionText">Connecting</span></div></header>
<section class="card direction-card" aria-label="Wind direction"><div class="section-heading"><span>Wind direction</span><span class="tag">Wind coming from</span></div>
<svg class="dial" viewBox="0 0 320 320" role="img" aria-labelledby="compassTitle"><title id="compassTitle">Wind direction compass</title><circle cx="160" cy="160" r="122" fill="#f6f9f8"/><circle cx="160" cy="160" r="112" fill="white" stroke="#dce6e4"/><g id="ticks"></g><g fill="none" stroke="#edf2f0"><path d="M160 52V268M52 160H268"/><path d="M84 84L236 236M84 236L236 84"/></g>
<path d="M160 65L174 146L160 160L146 146Z" fill="#e8efed"/><path d="M255 160L174 174L160 160L174 146Z" fill="#eef3f1"/><path d="M160 255L146 174L160 160L174 174Z" fill="#e8efed"/><path d="M65 160L146 146L160 160L146 174Z" fill="#eef3f1"/>
<text class="cardinal north" x="160" y="21" text-anchor="middle">N</text><text class="cardinal" x="306" y="166" text-anchor="middle">E</text><text class="cardinal" x="160" y="312" text-anchor="middle">S</text><text class="cardinal" x="14" y="166" text-anchor="middle">W</text><text class="minor" x="261" y="62" text-anchor="middle">NE</text><text class="minor" x="261" y="265" text-anchor="middle">SE</text><text class="minor" x="59" y="265" text-anchor="middle">SW</text><text class="minor" x="59" y="62" text-anchor="middle">NW</text>
<g id="needle" transform="rotate(0 160 160)"><path d="M160 61L170 159L160 151L150 159Z" fill="#dd6151"/><path d="M160 237L150 159L160 167L170 159Z" fill="#517a80"/></g><circle cx="160" cy="160" r="9" fill="white" stroke="#18343b" stroke-width="2"/><circle cx="160" cy="160" r="3" fill="#18343b"/></svg>
<div class="reading" id="degrees">—</div><div class="bearing" id="bearing">Waiting for direction</div><div class="sensor-note" id="directionStatus" role="status">Waiting for the vane sensor</div></section>
<div class="metrics"><section class="card metric"><button class="switch" id="unitButton" aria-label="Switch temperature units">°C</button><div class="label"><svg class="icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M9 14V5a3 3 0 0 1 6 0v9a5 5 0 1 1-6 0Z"/><path d="M12 8v10"/></svg>Temperature</div><div class="metric-value"><span id="temperature">—</span><span class="unit" id="temperatureUnit"> °F</span></div><div class="caption" id="temperatureStatus">Waiting for SHT3x</div></section>
<section class="card metric"><div class="label"><svg class="icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M12 2C10 6 5 11 5 15a7 7 0 0 0 14 0c0-4-5-9-7-13Z"/><path d="M8 15a4 4 0 0 0 4 4"/></svg>Humidity</div><div class="metric-value"><span id="humidity">—</span><span class="unit"> %</span></div><div class="caption" id="humidityStatus">Relative humidity</div></section></div>
<div class="camera-state" id="cameraStatus" role="status"><span class="label">Camera</span><span id="panStatus">Pan: —</span><span id="tiltStatus">Tilt: —</span><span id="zoomStatus">Zoom cmd: —</span><span id="heaterStatus">Heater: —</span></div><div class="statusbar"><span id="updated">Waiting for live data</span><span id="controller">Pan/tilt: —</span></div><div class="calibration" id="homeNetwork" role="status">Home Wi-Fi: checking</div><div class="calibration" id="offset">North = 0° · East = 90° · South = 180° · West = 270°</div><div class="calibration"><a href="/update">Firmware update</a></div><noscript><p>Enable JavaScript to display live readings.</p></noscript>
</main><script>
const $=id=>document.getElementById(id), ns='http://www.w3.org/2000/svg';
for(let a=0;a<360;a+=5){let line=document.createElementNS(ns,'line');line.setAttribute('x1','160');line.setAttribute('x2','160');line.setAttribute('y1',a%30===0?'42':'45');line.setAttribute('y2',a%30===0?'52':'49');line.setAttribute('stroke',a%30===0?'#77938f':'#c1d0cc');line.setAttribute('stroke-width',a%30===0?'1.7':'1');line.setAttribute('transform',`rotate(${a} 160 160)`);$('ticks').append(line);}
const sectors=['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];
let lastData=null, useC=false, displayedAngle=null, targetAngle=null, lastGood=0;
const WEB_POLL_MS=100;
let lastFrameTime=null;
function pointNeedle(angle){
 if(targetAngle===null){targetAngle=displayedAngle=angle;$('needle').setAttribute('transform',`rotate(${angle} 160 160)`);}
 else targetAngle+=((angle-targetAngle)%360+540)%360-180;
}
function animateNeedle(time){
 const elapsed=lastFrameTime===null?16:Math.min(50,Math.max(0,time-lastFrameTime));
 lastFrameTime=time;
 if(displayedAngle!==null&&targetAngle!==null){
  // Smooth the visual pointer at the browser frame rate; do not filter sensor data.
  displayedAngle+=(targetAngle-displayedAngle)*(1-Math.exp(-elapsed/70));
  if(Math.abs(targetAngle-displayedAngle)<0.01)displayedAngle=targetAngle;
  $('needle').setAttribute('transform',`rotate(${displayedAngle} 160 160)`);
 }
 requestAnimationFrame(animateNeedle);
}
requestAnimationFrame(animateNeedle);
function showTemperature(){if(!lastData||lastData.temp_c===null){$('temperature').textContent='—';return;}$('temperature').textContent=(useC?lastData.temp_c:lastData.temp_c*1.8+32).toFixed(1);}
$('unitButton').addEventListener('click',()=>{useC=!useC;$('temperatureUnit').textContent=useC?' °C':' °F';$('unitButton').textContent=useC?'°F':'°C';showTemperature();});
function render(data){lastData=data;lastGood=Date.now();$('connection').className='connection live';$('connectionText').textContent='Live';
 const available=data.direction!==null, valid=available&&data.direction_valid&&data.direction_age_ms<1500;
 if(available){const angle=data.direction; pointNeedle(angle);
 $('degrees').textContent=`${angle.toFixed(1)}°`;$('bearing').textContent=sectors[Math.floor((angle+11.25)/22.5)%16];$('compassTitle').textContent=`Wind coming from ${angle.toFixed(1)} degrees`;
 }else{$('degrees').textContent='—';$('bearing').textContent='Waiting for direction';}
 $('needle').classList.toggle('valid',valid);$('degrees').classList.toggle('stale',!valid);
 const field=['Magnet OK','Magnet too strong — check spacing','Magnet too weak — check spacing','No magnet detected','Direction sensor unavailable'][data.magnet];
 $('directionStatus').textContent=valid?field:(available?'Last reading · '+field:field);$('directionStatus').classList.toggle('error',!valid||data.magnet!==0);
 showTemperature();$('humidity').textContent=data.humidity===null?'—':data.humidity.toFixed(1);
 const fresh=data.sht_valid&&data.sht_age_ms<5000;
 $('temperatureStatus').textContent=data.temp_c===null?'Waiting for SHT3x':fresh?'Air temperature':'Last good reading';
 $('humidityStatus').textContent=data.humidity===null?'Waiting for SHT3x':fresh?'Relative humidity':'Last good reading';
 $('temperature').classList.toggle('stale',!fresh);$('humidity').classList.toggle('stale',!fresh);
 const home=$('homeNetwork');
 home.replaceChildren();
 if(data.home_ip){const link=document.createElement('a');link.href='http://'+data.home_ip+'/';link.textContent=link.href;home.append('Home Wi-Fi address: ',link);}
 else home.textContent=data.home_configured?'Home Wi-Fi: connecting · WindVane hotspot is available':'Home Wi-Fi: not configured';
 $('panStatus').textContent='Pan: '+(['Stopped','Left','Right'][data.pan]||'Unknown');
 $('tiltStatus').textContent='Tilt: '+(['Stopped','Up','Down'][data.tilt]||'Unknown');
 $('zoomStatus').textContent='Zoom cmd: '+(data.zoom_reported?(['Stopped','In','Out'][data.zoom]||'Unknown'):'Not reported');
 $('heaterStatus').textContent='Heater: '+(data.heater_on?'ON':'OFF')+(data.heater_assigned?'':' (output unassigned)');
 $('cameraStatus').classList.remove('stale');
 $('updated').textContent='Updated '
+new Date().toLocaleTimeString();$('controller').textContent='Pan/tilt: '+(data.armed?'armed':'disarmed');
 $('offset').textContent=`Direction offset: ${data.offset}° · North 0° / East 90° / South 180° / West 270°`;
}
function offline(){ $('cameraStatus').classList.add('stale');$('connection').className='connection';$('connectionText').textContent='Disconnected';$('needle').classList.remove('valid');$('degrees').classList.add('stale');$('temperature').classList.add('stale');$('humidity').classList.add('stale');$('updated').textContent=lastGood?'Connection lost · readings held':'Unable to connect · retrying';}
async function poll(){const pollStarted=performance.now();try{const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2500);let response;try{response=await fetch('/api/weather',{cache:'no-store',signal:controller.signal});}finally{clearTimeout(timer);}if(!response.ok)throw Error('HTTP '+response.status);render(await response.json());}catch(e){offline();}finally{setTimeout(poll,Math.max(10,WEB_POLL_MS-(performance.now()-pollStarted)));}}
poll();
</script></body></html>
)WINDPAGE";

void publishWebSnapshot() {
  WebSnapshot next = {};
  next.haveAngle = haveDirection;
  next.angleValid = angleCurrentlyValid;
  next.haveSht = haveTemperatureHumidity;
  next.armed = allowMotion;
  next.pan = !motorsRunning ? 0 : strcmp(motionName,"Left")==0 ? 1 : strcmp(motionName,"Right")==0 ? 2 : 0;
  next.tilt = !motorsRunning ? 0 : strcmp(motionName,"Up")==0 ? 1 : strcmp(motionName,"Down")==0 ? 2 : 0;
  next.zoom = zoomCommand;
  next.zoomSeen = zoomReported;
  next.heaterRequested = heaterOn;
  next.heaterAssigned = HEATER_PIN >= 0;
  next.direction = fmodf(lastDegrees + WEB_DIRECTION_OFFSET_DEG + 360.0f, 360.0f);
  next.tempC = lastTempC;
  next.humidity = lastHumidityPct;
  next.angleMs = lastDirectionValidMs;
  next.shtMs = lastShtValidMs;
  next.homeConnected = HOME_WIFI_SSID[0] && WiFi.status() == WL_CONNECTED;
  if (next.homeConnected) {
    IPAddress ip = WiFi.localIP();
    for (uint8_t i = 0; i < 4; ++i) next.homeIp[i] = ip[i];
  }
  next.magnet = !angleCurrentlyValid ? ((strcmp(angleError,"No magnet")==0)?3:4) :
                (lastMagnetStatus & 0x08) ? 1 : (lastMagnetStatus & 0x10) ? 2 : 0;
  portENTER_CRITICAL(&webMux);
  webSnapshot = next;
  portEXIT_CRITICAL(&webMux);
}

void handleWebWeather() {
  ++webRequests;
  WebSnapshot value;
  portENTER_CRITICAL(&webMux);
  value = webSnapshot;
  portEXIT_CRITICAL(&webMux);
  char angle[20] = "null", temp[20] = "null", humidity[20] = "null";
  if (value.haveAngle) snprintf(angle, sizeof(angle), "%.2f", value.direction);
  if (value.haveSht) {
    snprintf(temp, sizeof(temp), "%.2f", value.tempC);
    snprintf(humidity, sizeof(humidity), "%.2f", value.humidity);
  }
  const uint32_t now = millis();
  char homeIp[32] = "null";
  if (value.homeConnected)
    snprintf(homeIp, sizeof(homeIp), "\"%u.%u.%u.%u\"",
        value.homeIp[0], value.homeIp[1], value.homeIp[2], value.homeIp[3]);
  char json[720];
  snprintf(json, sizeof(json),
      "{\"direction\":%s,\"direction_valid\":%s,\"direction_age_ms\":%lu,"
      "\"magnet\":%u,\"temp_c\":%s,\"humidity\":%s,\"sht_valid\":%s,"
      "\"sht_age_ms\":%lu,\"armed\":%s,\"offset\":%.1f,\"home_ip\":%s,\"home_configured\":%s,"
      "\"pan\":%u,\"tilt\":%u,\"zoom\":%u,\"zoom_reported\":%s,\"heater_on\":%s,\"heater_assigned\":%s}",
      angle, value.angleValid ? "true" : "false",
      (unsigned long)uint32_t(now-value.angleMs), value.magnet, temp, humidity,
      value.haveSht ? "true" : "false", (unsigned long)uint32_t(now-value.shtMs),
      value.armed ? "true" : "false", WEB_DIRECTION_OFFSET_DEG,
      homeIp, HOME_WIFI_SSID[0] ? "true" : "false",
      value.pan, value.tilt, value.zoom, value.zoomSeen ? "true" : "false",
      value.heaterRequested ? "true" : "false", value.heaterAssigned ? "true" : "false");
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", json);
}

// OTA state below is owned by the web task. Only the main task touches outputs.
bool otaReceiving = false, otaSucceeded = false;
size_t otaExpected = 0, otaWritten = 0;
uint32_t otaLastChunkMs = 0, otaRebootAt = 0;
String otaError;

void releaseOtaControls() {
  portENTER_CRITICAL(&otaMux);
  otaControlRequested = false;
  otaControlStopped = false;
  portEXIT_CRITICAL(&otaMux);
  // Main task has already disarmed and switched heater off. Never auto-rearm.
}

void failOta(const String &message) {
  if (otaReceiving) Update.abort();
  otaReceiving = false;
  otaSucceeded = false;
  otaError = message;
  releaseOtaControls();
}

bool stopControlsForOta() {
  portENTER_CRITICAL(&otaMux);
  otaControlStopped = false;
  otaControlRequested = true;
  portEXIT_CRITICAL(&otaMux);
  const uint32_t started = millis();
  while (uint32_t(millis()-started) < 2000) {
    portENTER_CRITICAL(&otaMux);
    const bool stopped = otaControlStopped;
    portEXIT_CRITICAL(&otaMux);
    if (stopped) return true;
    vTaskDelay(pdMS_TO_TICKS(5)); // Let main task stop outputs before flash writes.
  }
  return false;
}

bool otaRequestAuthorized() {
  // Authenticate the multipart callbacks too, not just the final response.
  return OTA_PASSWORD[0] && webServer.authenticate(OTA_USERNAME, OTA_PASSWORD);
}

const char OTA_PAGE[] PROGMEM = R"OTAPAGE(
<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wind vane firmware update</title><style>body{max-width:620px;margin:35px auto;padding:0 20px;font:16px system-ui;color:#18343b;background:#f2f5f4}main{background:white;border:1px solid #dce6e4;border-radius:18px;padding:24px}h1{font-size:24px}p{line-height:1.5}button,input{font:inherit;margin:10px 0;max-width:100%}button{padding:10px 18px;background:#087f79;color:white;border:0;border-radius:8px}button:disabled{opacity:.5}progress{width:100%}#status{white-space:pre-wrap}</style>
<main><a href="/">Back to weather</a><h1>Firmware update</h1>
<p>Select the application <b>.ino.bin</b> made by Arduino's <b>Sketch → Export Compiled Binary</b>. Use the same ESP32-C3 board and OTA partition settings. Do not select a merged, bootloader or partitions file.</p>
<p>Pan/tilt stops and the heater turns off when uploading starts. Keep power connected until the controller restarts. Include your Wi-Fi settings and this updater in the new sketch.</p>
<p id="slot">Checking update space…</p><form id="form"><input id="file" type="file" accept=".bin" required><br><button id="send" disabled>Upload firmware</button></form><progress id="progress" value="0" max="100"></progress><p id="status" role="status">Ready</p></main>
<script>
const $=id=>document.getElementById(id);let capacity=0;
fetch('/api/update',{cache:'no-store'}).then(r=>{if(!r.ok)throw Error('Unable to check firmware space');return r.json();}).then(d=>{capacity=d.bytes;$('slot').textContent=capacity?'Available application space: '+Math.floor(capacity/1024)+' KB':'No spare OTA partition. First upload by USB using a partition scheme with OTA.';$('send').disabled=!capacity;}).catch(e=>{$('status').textContent=e.message;});
$('form').addEventListener('submit',e=>{e.preventDefault();const file=$('file').files[0];if(!file)return;
 if(!file.name.toLowerCase().endsWith('.bin')||/merged|bootloader|partitions/i.test(file.name)){$('status').textContent='Select the application .ino.bin file.';return;}
 if(!file.size||file.size>capacity){$('status').textContent='This file exceeds the available application space or is empty.';return;}
 $('send').disabled=true;$('file').disabled=true;$('status').textContent='Uploading…';
 const request=new XMLHttpRequest();request.open('POST','/update');request.setRequestHeader('X-Firmware-Size',String(file.size));request.timeout=180000;
 request.upload.onprogress=e=>{if(e.lengthComputable){$('progress').value=e.loaded/e.total*100;$('status').textContent=e.loaded===e.total?'Checking firmware…':'Uploading '+Math.round(e.loaded/e.total*100)+'%';}};
 function unlock(){$('send').disabled=!capacity;$('file').disabled=false;}
 request.onload=()=>{if(request.status===200){$('progress').value=100;$('status').textContent=request.responseText+' Open the weather page again in about 10 seconds.';}else{$('status').textContent=request.responseText||'Upload failed';unlock();}};
 request.onerror=request.ontimeout=()=>{$('status').textContent='Connection lost. Wait, then reopen the weather page to check the controller before retrying.';unlock();};
 const data=new FormData();data.append('firmware',file);request.send(data);
});
</script></html>
)OTAPAGE";

void handleOtaUpload() {
  HTTPUpload &upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (!otaRequestAuthorized()) return;
    if (otaSucceeded) return; // Reboot already pending; never replace again.
    if (otaReceiving) failOta("Previous upload interrupted; retry.");
    otaError = "";
    otaWritten = 0;
    // Browser sets Origin; custom header also prevents cross-origin form posts.
    if (webServer.header("Origin") != String("http://") + webServer.header("Host")) {
      otaError = "Upload must come from this controller's update page.";
      return;
    }
    String filename = upload.filename;
    filename.toLowerCase();
    if (!filename.endsWith(".bin") || filename.indexOf("merged") >= 0 ||
        filename.indexOf("bootloader") >= 0 || filename.indexOf("partitions") >= 0) {
      otaError = "Select the application .ino.bin, not a merged/bootloader/partitions file.";
      return;
    }
    const String sizeHeader = webServer.header("X-Firmware-Size");
    if (!sizeHeader.length() || sizeHeader.length() > 8) {
      otaError = "Missing firmware size. Use this controller's update page.";
      return;
    }
    for (size_t i=0; i<sizeHeader.length(); ++i) {
      if (sizeHeader[i] < '0' || sizeHeader[i] > '9') {
        otaError = "Invalid firmware size.";
        return;
      }
    }
    otaExpected = sizeHeader.toInt();
    const esp_partition_t *slot = esp_ota_get_next_update_partition(nullptr);
    if (!slot || slot == esp_ota_get_running_partition()) {
      otaError = "No spare OTA partition. Upload by USB with an OTA partition scheme first.";
      return;
    }
    if (!otaExpected || otaExpected > slot->size) {
      otaError = "Firmware is empty or too large for the OTA partition.";
      return;
    }
    if (!stopControlsForOta()) {
      failOta("Could not confirm outputs stopped. Upload cancelled.");
      return;
    }
    if (!Update.begin(otaExpected, U_FLASH)) {
      failOta(String("Unable to start update: ") + Update.errorString());
      return;
    }
    otaReceiving = true;
    otaLastChunkMs = millis();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaRequestAuthorized() || !otaReceiving) return;
    otaLastChunkMs = millis();
    if (upload.currentSize > otaExpected-otaWritten) {
      failOta("Firmware is longer than expected.");
    } else if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      failOta(String("Flash write failed: ") + Update.errorString());
    } else {
      otaWritten += upload.currentSize;
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!otaRequestAuthorized() || !otaReceiving) return;
    if (otaWritten != otaExpected || upload.totalSize != otaExpected) {
      failOta("Upload incomplete; old firmware retained.");
    } else if (!Update.end()) { // Known exact length; never accept a short upload.
      failOta(String("Firmware validation failed: ") + Update.errorString());
    } else {
      otaReceiving = false;
      otaSucceeded = true;
      otaRebootAt = millis() + 1500; // Also reboot if client drops before response.
      // Outputs stay inhibited until restart, including while sending response.
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED && otaReceiving) {
    failOta("Upload interrupted; old firmware retained.");
  }
}

void handleOtaResult() {
  if (!otaRequestAuthorized()) { webServer.requestAuthentication(); return; }
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.sendHeader("Connection", "close");
  if (otaSucceeded) webServer.send(200, "text/plain", "Firmware installed. Restarting controller.");
  else webServer.send(400, "text/plain", otaError.length() ? otaError : String("No firmware received."));
}

void registerOtaRoutes() {
  const char *headers[] = {"Origin", "Host", "X-Firmware-Size"};
  webServer.collectHeaders(headers, 3);
  webServer.on("/update", HTTP_GET, []() {
    if (!otaRequestAuthorized()) { webServer.requestAuthentication(); return; }
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html; charset=utf-8", OTA_PAGE);
  });
  webServer.on("/api/update", HTTP_GET, []() {
    if (!otaRequestAuthorized()) { webServer.requestAuthentication(); return; }
    const esp_partition_t *slot = esp_ota_get_next_update_partition(nullptr);
    const size_t capacity = slot && slot != esp_ota_get_running_partition() ? slot->size : 0;
    char json[48];
    snprintf(json, sizeof(json), "{\"bytes\":%lu}", (unsigned long)capacity);
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", json);
  });
  webServer.on("/update", HTTP_POST, handleOtaResult, handleOtaUpload);
}

void weatherWebTask(void *) {
  for (;;) {
    webServer.handleClient();
    if (otaReceiving && uint32_t(millis()-otaLastChunkMs) > 30000)
      failOta("Upload timed out; old firmware retained.");
    if (otaSucceeded && int32_t(millis()-otaRebootAt) >= 0) ESP.restart();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void startWeatherWeb() {
  publishWebSnapshot();
  if (WEB_DIAGNOSTICS && Serial) Serial.println("[WiFi] Starting WindVane hotspot");
  WiFi.setHostname("windvane");
  WiFi.mode(HOME_WIFI_SSID[0] ? WIFI_AP_STA : WIFI_AP);
  if (!WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1),
                         IPAddress(255,255,255,0))) {
    if (WEB_DIAGNOSTICS && Serial) Serial.println("[WiFi] IP configuration FAILED");
    return;
  }
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, 1, 0, 2)) {
    if (WEB_DIAGNOSTICS && Serial) Serial.println("[WiFi] Hotspot start FAILED");
    return;
  }
  WiFi.setSleep(false);
  if (HOME_WIFI_SSID[0]) {
    WiFi.setAutoReconnect(true);
    WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);
    if (WEB_DIAGNOSTICS && Serial) Serial.println("[WiFi] Joining home router in background");
  }
  webServer.on("/", HTTP_GET, []() {
    ++webRequests;
    if (WEB_DIAGNOSTICS && Serial) Serial.println("[HTTP] Sending dashboard");
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html; charset=utf-8", WEATHER_PAGE);
  });
  webServer.on("/api/weather", HTTP_GET, handleWebWeather);
  webServer.on("/health", HTTP_GET, []() {
    ++webRequests;
    webServer.send(200, "text/plain", "WindVane web server OK");
  });
  webServer.on("/favicon.ico", HTTP_GET, []() { webServer.send(204); });
  webServer.onNotFound([]() { webServer.send(404, "text/plain", "Not found"); });
  registerOtaRoutes();
  webServer.begin();
  // Higher than the Arduino loop task; each request still yields to sensor/control work.
  webReady = xTaskCreate(weatherWebTask, "weather-web", 8192, nullptr, 2,
                         &webTaskHandle) == pdPASS;
  if (!webReady) webServer.stop();
  if (WEB_DIAGNOSTICS && Serial) {
    Serial.println(webReady ? "[WiFi] HTTP server ready: http://192.168.4.1/" :
                              "[WiFi] Web task creation FAILED");
  }
}

// Called from the main loop only. Never wait for a router connection here.
void serviceHomeWifi(uint32_t now) {
  if (!HOME_WIFI_SSID[0]) return;
  static bool wasConnected = false;
  static uint32_t lastRetryMs = 0;
  static uint32_t lastHomeReport = 0;
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected) {
    if (WEB_DIAGNOSTICS && Serial &&
        (!wasConnected || uint32_t(now-lastHomeReport) >= 10000)) {
      lastHomeReport = now;
      Serial.print("[WiFi] Home page: http://");
      Serial.println(WiFi.localIP());
    }
    lastRetryMs = now;
  } else {
    if (wasConnected && WEB_DIAGNOSTICS && Serial)
      Serial.println("[WiFi] Router disconnected; WindVane hotspot remains available");
    if (uint32_t(now-lastRetryMs) >= 30000) {
      lastRetryMs = now;
      WiFi.reconnect();
    }
  }
  wasConnected = connected;
}

void setup() {
  if (HEATER_PIN >= 0) {
    digitalWrite(HEATER_PIN, LOW);
    pinMode(HEATER_PIN, OUTPUT);
  }
  initMotors();
  Serial.begin(115200); // native USB only
  // UART1 weather TX=10; unused RX=9, leave unconnected.
  Serial1.begin(115200, SERIAL_8N1, 9, WEATHER_TX_PIN);
  // UART0 camera RX=7; unused TX=21 (not exposed), leave unconnected.
  Serial0.setRxBufferSize(512);
  Serial0.begin(9600, SERIAL_8N1, CAMERA_RX_PIN, 21);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(20);
  oled.setI2CAddress(0x3C << 1);
  oled.setBusClock(100000);
  oled.begin();
  Wire.setClock(100000);
  oled.setPowerSave(1); // Starts dark; no splash screen.
  resetGesture();
  while (Serial0.available()) Serial0.read();
  startWeatherWeb();
}

void serviceCamera() {
  if (otaControlsLocked()) {
    // Only this main-task code touches outputs. Acknowledge after stopping them.
    disarm();
    setHeater(false);
    zoomCommand = 0;
    bytesUsed = 0;
    while (Serial0.available() > 0) Serial0.read();
    portENTER_CRITICAL(&otaMux);
    otaControlStopped = otaControlRequested;
    portEXIT_CRITICAL(&otaMux);
    return;
  }
  while (Serial0.available() > 0) feedByte((uint8_t)Serial0.read());
  if (MOTION_WATCHDOG_MS > 0 && motorsRunning &&
      (unsigned long)(millis()-lastMotionCommandMs) > MOTION_WATCHDOG_MS)
    stopMotors();
    pinMode(8, OUTPUT); //led test
    digitalWrite(8, HIGH);
}

void loop() {
  serviceCamera();
  uint32_t now = millis();
  serviceHomeWifi(now);
  serviceOledTimeout(now);
  serviceSht(now);
  serviceCamera();
  now = millis();
  if (uint32_t(now - lastAnglePoll) >= ANGLE_INTERVAL_MS) {
    lastAnglePoll = now;
    const bool wasOn = oledOn;
    sampleDirection(now);
    // Render immediately on gesture completion; otherwise at weather cadence.
    if (!wasOn && oledOn) renderOled();
  }
  serviceCamera();
  now = millis();
  if (uint32_t(now - lastTx) >= TX_INTERVAL_MS) {
    lastTx = now;
    sendWeather();
    renderOled();
  }
  now = millis();
  if (uint32_t(now-lastWebSnapshot) >= 50) {
    lastWebSnapshot = now;
    publishWebSnapshot();
  }
  serviceCamera();
  // C3 has one CPU core. Do not leave the loop task continuously runnable.
  // Yield to the WiFi/IP stack, web task and idle task every pass.
  delay(2);
  static uint32_t lastWifiReport = 0;
  if (WEB_DIAGNOSTICS && uint32_t(millis()-lastWifiReport) >= 5000) {
    lastWifiReport = millis();
    if (Serial) Serial.printf("[WiFi] clients=%u server=%s requests=%lu heap=%lu\n",
        unsigned(WiFi.softAPgetStationNum()), webReady ? "ready" : "FAILED",
        (unsigned long)webRequests, (unsigned long)ESP.getFreeHeap());
  }
}
