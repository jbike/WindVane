#include <font_Arial.h>
#include <movingAvg.h>
#include "SPI.h"
#include "ILI9341_t3.h"

#define TFT_DC 9
#define TFT_CS 10
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

#define RE 4
#define DE 5
const byte anemometer[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B };
byte modbusResponse[9];
byte modbusCount = 0;
bool modbusWaiting = false;
unsigned long modbusSentAt = 0;
const unsigned long modbusTimeout = 400;  // ms; never block the display
unsigned long modbusGoodCount = 0;
unsigned long modbusTimeoutCount = 0;
unsigned long modbusCrcErrors = 0;
unsigned long lastModbusPrint = 0;

uint16_t modbusCRC(const byte *data, byte len) {
  uint16_t crc = 0xFFFF;
  for (byte i = 0; i < len; ++i) {
    crc ^= data[i];
    for (byte b = 0; b < 8; ++b)
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
  }
  return crc;
}


float angle = 0;
// Separate display angle from measured wind direction.
float smoothedAngle = 0;
bool angleInitialized = false;
unsigned long lastSmoothTime = 0;
const float DIRECTION_SMOOTH_TIME_MS = 140.0f; // Lower=quicker, higher=smoother

int centreX = 140;
int centreY = 120;
int diameter = 85;
int diameter1 = 91;
int last_dx, last_dy, dx, dy;
int last_md_dx = 0, last_md_dy = 0;
float wind_Spd = 0;
int temp = 0;
float humid = 0;
uint32_t updateTime = 0;
uint32_t updateTimeCard = 0;
uint32_t updateTimeDeg = 0;
int last = -1;
char buf[6], buf1[6], buf2[6], buf3[6];

const int NUMBER_OF_FIELDS = 4;
movingAvg avgTmp(50);
unsigned long lastModbusTime = 0;
const unsigned long modbusInterval = 1000;
// Bench use: false while sonic Modbus anemometer is disconnected.
// Set true when reconnected.
const bool sonicConnected = true;
float sonicSpeedMph = 0;
float modbusDirection = 0;
float lastGoodModbusDirection = 0;
bool sonicDirectionValid = false;  // true only after a real non-calm sonic direction
// Smooth the blue sonic pointer between the once-per-second Modbus readings.
float smoothedSonicAngle = 0;
bool sonicAngleInitialized = false;
unsigned long lastSonicSmoothTime = 0;
const float SONIC_SMOOTH_TIME_MS = 300.0f;
// Separate communication watchdogs. Only valid complete messages refresh them.
const unsigned long VANE_OFFLINE_MS = 2000;
const unsigned long SONIC_OFFLINE_MS = 3000;
unsigned long lastGoodVaneAt = 0;
unsigned long lastGoodSonicAt = 0;
bool vaneOnline = false;
bool sonicOnline = false;
bool redArrowDrawn = false;
bool blueArrowDrawn = false;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_WHITE);
  tft.drawCircle(centreX, centreY, diameter1, ILI9341_BLACK);
  Draw_Compass_Rose();
  avgTmp.begin();
  Serial.println("Sonic Modbus test: Serial3 4800 baud, slave 1, function 03, two registers");
  Serial.println("RS485: RE=4 DE=5; RS422 vane may remain disconnected");
}

void loop() {
  static char serial1Buffer[32];
  static byte bufPos = 0;
  static bool receivingPacket = false;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '<') {
      bufPos = 0;
      receivingPacket = true;
    } else if (c == '>' && receivingPacket) {
      serial1Buffer[bufPos] = '\0';
      int a, unusedSpeed, t, h;
      char extra;
      // Require exactly four values with no extra characters.
      if (sscanf(serial1Buffer, "%d,%d,%d,%d%c", &a, &unusedSpeed, &t, &h, &extra) == 4 &&
          a >= 0 && a < 360 && t >= -100 && t <= 200 && h >= 0 && h <= 100) {
        angle = a;
        wind_Spd = unusedSpeed;
        temp = t;
        humid = h;
        lastGoodVaneAt = millis();
        if (!vaneOnline) {
          vaneOnline = true;
          angleInitialized = false; // restart smoothly at actual position
          lastSmoothTime = 0;
          updateTimeDeg = 0;
          updateTimeCard = 0;
        }
        updateWeatherReadings();
      }
      receivingPacket = false;
      bufPos = 0;
    } else if (receivingPacket) {
      if (bufPos < sizeof(serial1Buffer) - 1) serial1Buffer[bufPos++] = c;
      else { receivingPacket = false; bufPos = 0; }
    }
  }

  unsigned long now = millis();
  if (vaneOnline && now - lastGoodVaneAt > VANE_OFFLINE_MS) {
    vaneOnline = false;
    angleInitialized = false;
    if (redArrowDrawn) {
      arrow(last_dx, last_dy, centreX, centreY, 8, 8, ILI9341_WHITE);
      redArrowDrawn = false;
    }
    tft.fillRect(240, 25, 80, 30, ILI9341_WHITE);
    tft.setFont(Arial_24);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(240, 25);
    tft.print("---");
    if (last >= 0 && last < 16) {
      const char* labels[] = {"NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW","N"};
      display_item2(labels[last]);
      last = -1;
    }
    tft.fillRect(0, 193, 80, 30, ILI9341_WHITE);
    tft.setFont(Arial_24);
    tft.setCursor(0, 193);
    tft.print("--");
    tft.fillRect(52, 210, 70, 25, ILI9341_WHITE);
    tft.setFont(Arial_12);
    tft.setCursor(52, 210);
    tft.print("--%");
    Serial.println("VANE OFFLINE");
  }
  if (sonicOnline && now - lastGoodSonicAt > SONIC_OFFLINE_MS) {
    sonicOnline = false;
    sonicDirectionValid = false;
    sonicAngleInitialized = false;
    lastSonicSmoothTime = 0;
    if (blueArrowDrawn) {
      arrow(last_md_dx, last_md_dy, centreX, centreY, 8, 8, ILI9341_WHITE);
      blueArrowDrawn = false;
    }
    tft.fillRect(220, 190, 90, 30, ILI9341_WHITE);
    tft.setFont(Arial_24);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(220, 190);
    tft.print("--");
    Serial.println("SONIC OFFLINE");
  }

  if (millis() >= updateTime) {
    updateTime = millis() + 45;

    if (vaneOnline) {
      unsigned long tick = millis();
      float elapsed = lastSmoothTime ? (float)(tick - lastSmoothTime) : 45.0f;
      lastSmoothTime = tick;
      if (!angleInitialized) {
        smoothedAngle = angle;
        angleInitialized = true;
      } else {
        float delta = fmodf(angle - smoothedAngle + 540.0f, 360.0f) - 180.0f;
        float fraction = elapsed / (DIRECTION_SMOOTH_TIME_MS + elapsed);
        smoothedAngle += fraction * delta;
        if (smoothedAngle >= 360.0f) smoothedAngle -= 360.0f;
        if (smoothedAngle < 0.0f) smoothedAngle += 360.0f;
      }
      float displayAngle = smoothedAngle - 125.0f;
      if (displayAngle < 0) displayAngle += 360.0f;

      dx = (diameter - 14) * cos((displayAngle - 90) * PI / 180) + centreX;
      dy = (diameter - 14) * sin((displayAngle - 90) * PI / 180) + centreY;

      // Erase only the arrow that was actually drawn.
      if (redArrowDrawn) arrow(last_dx, last_dy, centreX, centreY, 8, 8, ILI9341_WHITE);
      if (blueArrowDrawn) {
        arrow(last_md_dx, last_md_dy, centreX, centreY, 8, 8, ILI9341_WHITE);
        blueArrowDrawn = false;
      }
      arrow(dx, dy, centreX, centreY, 8, 8, ILI9341_RED);
      redArrowDrawn = true;
      last_dx = dx;
      last_dy = dy;

      if (millis() >= updateTimeDeg) {
        updateTimeDeg = millis() + 60;
        dtostrf(displayAngle, 5, 0, buf);
        tft.fillRect(240, 25, 80, 30, ILI9341_WHITE);
        tft.setFont(Arial_24);
        tft.setTextColor(ILI9341_RED);
        tft.setCursor(240, 25);
        tft.print(buf);
      }
      if (millis() >= updateTimeCard) {
        updateTimeCard = millis() + 100;
        getDir(displayAngle);
      }
    }

    // Draw the blue sonic pointer independently of red-vane availability.
    if (sonicOnline && sonicDirectionValid) {
      unsigned long sonicTick = millis();
      float sonicElapsed = lastSonicSmoothTime ? (float)(sonicTick - lastSonicSmoothTime) : 45.0f;
      lastSonicSmoothTime = sonicTick;
      if (!sonicAngleInitialized) {
        smoothedSonicAngle = lastGoodModbusDirection;
        sonicAngleInitialized = true;
      } else {
        // Take the shortest arc; e.g. 359 to 1 is +2, not -358 degrees.
        float delta = fmodf(lastGoodModbusDirection - smoothedSonicAngle + 540.0f, 360.0f) - 180.0f;
        float fraction = sonicElapsed / (SONIC_SMOOTH_TIME_MS + sonicElapsed);
        smoothedSonicAngle += fraction * delta;
        if (smoothedSonicAngle >= 360.0f) smoothedSonicAngle -= 360.0f;
        if (smoothedSonicAngle < 0.0f) smoothedSonicAngle += 360.0f;
      }
      float rad = (smoothedSonicAngle - 90) * PI / 180.0f;
      int md_dx = (diameter - 20) * cos(rad) + centreX;
      int md_dy = (diameter - 20) * sin(rad) + centreY;
      if (blueArrowDrawn) arrow(last_md_dx, last_md_dy, centreX, centreY, 8, 8, ILI9341_WHITE);
      arrow(md_dx, md_dy, centreX, centreY, 8, 8, ILI9341_BLUE);
      last_md_dx = md_dx;
      last_md_dy = md_dy;
      blueArrowDrawn = true;
    }
  }

  if (sonicConnected) serviceSonicModbus();
}

void Draw_Compass_Rose() {
  for (float i = 0; i < 360; i += 22.5) {
    float rad = (i - 90) * PI / 180.0;
    int tickLen = (fmod(i, 90.0) < 1) ? 18 : 10;
    int xOuter = round(centreX + (diameter + 4) * cos(rad));
    int yOuter = round(centreY + (diameter + 4) * sin(rad));
    int xInner = round(centreX + (diameter - tickLen) * cos(rad));
    int yInner = round(centreY + (diameter - tickLen) * sin(rad));
    tft.drawLine(xInner, yInner, xOuter, yOuter, ILI9341_BLACK);
  }

  display_item((centreX - 5), (centreY - (diameter + 24)), "N", ILI9341_RED, 2);
  display_item((centreX - 3), (centreY + (diameter + 12)), "S", ILI9341_RED, 2);
  display_item((centreX + (diameter + 10)), (centreY - 5), "E", ILI9341_RED, 2);
  display_item((centreX - (diameter + 20)), (centreY - 5), "W", ILI9341_RED, 2);

  tft.setFont(Arial_14);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(245, 55);
  tft.print("direction");
  tft.setCursor(215, 220);
  tft.print("sonic mph");
  tft.drawCircle(30, 225, 2, ILI9341_BLACK);
  tft.setCursor(35, 225);
  tft.print("F");
}

// AHT display does not depend on the separate sonic/Modbus sensor.
void updateWeatherReadings() {
    int avgTemp = avgTmp.reading(temp);
    dtostrf(avgTemp, 4, 0, buf2);
    tft.fillRect(0, 193, 80, 30, ILI9341_WHITE);
    tft.setCursor(0, 193);
    tft.setFont(Arial_24);
    tft.setTextColor(ILI9341_RED);
    tft.print(buf2);

    dtostrf(humid, 4, 0, buf3);
    tft.fillRect(52, 210, 70, 25, ILI9341_WHITE);
    tft.setCursor(52, 210);
    tft.setFont(Arial_12);
    tft.setTextColor(ILI9341_RED);
    tft.print(buf3);
    tft.print('%');

}

// Polling is asynchronous while awaiting the response: no 500-ms busy wait.
// At 4800 baud, Serial3.flush() spends about 17 ms finishing an 8-byte request.
void serviceSonicModbus() {
  unsigned long now = millis();
  if (!modbusWaiting && now - lastModbusTime >= modbusInterval) {
    lastModbusTime = now;
    while (Serial3.available()) Serial3.read(); // discard stale reply bytes
    modbusCount = 0;
    digitalWrite(RE, HIGH);   // disable receiver
    digitalWrite(DE, HIGH);   // enable transmitter
    delayMicroseconds(200);
    Serial3.write(anemometer, sizeof(anemometer));
    Serial3.flush();         // make sure last stop bit is sent
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);    // listen for response
    modbusSentAt = millis();
    modbusWaiting = true;
  }

  if (!modbusWaiting) return;
  while (Serial3.available() && modbusCount < sizeof(modbusResponse)) {
    modbusResponse[modbusCount++] = (byte)Serial3.read();
  }

  if (modbusCount == sizeof(modbusResponse)) {
    modbusWaiting = false;
    uint16_t crc = modbusCRC(modbusResponse, 7);
    uint16_t receivedCRC = (uint16_t)modbusResponse[7] |
                           ((uint16_t)modbusResponse[8] << 8);
    if (modbusResponse[0] != 1 || modbusResponse[1] != 3 ||
        modbusResponse[2] != 4 || crc != receivedCRC) {
      ++modbusCrcErrors;
      Serial.print("BAD MODBUS FRAME: ");
      for (byte i=0; i<9; ++i) {
        if (modbusResponse[i] < 16) Serial.print('0');
        Serial.print(modbusResponse[i], HEX);
        Serial.print(' ');
      }
      Serial.println();
      return;
    }

    ++modbusGoodCount;
    lastGoodSonicAt = millis();
    if (!sonicOnline) {
      sonicAngleInitialized = false; // fresh reading after a communications outage
      lastSonicSmoothTime = 0;
    }
    sonicOnline = true;
    int rawSpeed = ((int)modbusResponse[3] << 8) | modbusResponse[4];
    int rawDir = ((int)modbusResponse[5] << 8) | modbusResponse[6];
    float windMps = rawSpeed / 100.0f; // original sketch scaling
    sonicSpeedMph = windMps * 2.2369f;
    modbusDirection = rawDir;
    // At zero wind speed the sonic sensor reports direction 0, which is
    // NOT a valid north-wind indication. Keep the last measured direction.
    // A valid Modbus reply still refreshes the communication watchdog and
    // the displayed speed is updated (including 0.0 mph).
    if (rawSpeed > 0 && rawDir >= 0 && rawDir < 360) {
      lastGoodModbusDirection = modbusDirection;
      sonicDirectionValid = true;
    }
    // If calm before any valid direction, leave the blue arrow hidden.
    // If calm after valid readings, hold the previous blue arrow position.
    dtostrf(sonicSpeedMph, 4, 1, buf1);
    tft.fillRect(220, 190, 90, 30, ILI9341_WHITE);
    tft.setCursor(220, 190);
    tft.setFont(Arial_24);
    tft.setTextColor(ILI9341_RED);
    tft.print(buf1);

    // At most one diagnostic line per second.
    Serial.print("SONIC OK: ");
    Serial.print(sonicSpeedMph, 1);
    Serial.print(" mph, direction=");
    Serial.print(modbusDirection, 0);
    Serial.print(" deg, good=");
    Serial.println(modbusGoodCount);
  } else if (millis() - modbusSentAt > modbusTimeout) {
    modbusWaiting = false;
    ++modbusTimeoutCount;
    Serial.print("SONIC TIMEOUT: bytes=");
    Serial.print(modbusCount);
    Serial.print(" rx=");
    for (byte i=0; i<modbusCount; ++i) {
      if (modbusResponse[i] < 16) Serial.print('0');
      Serial.print(modbusResponse[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}

void getDir(float displayAngle) {
  const char* Cardinal_str[] = {
    "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S",
    "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW", "N"
  };

  int idx = -1;
  if (displayAngle >= 11.25 && displayAngle < 33.75) idx = 0;
  else if (displayAngle >= 33.75 && displayAngle < 56.25) idx = 1;
  else if (displayAngle >= 56.25 && displayAngle < 78.75) idx = 2;
  else if (displayAngle >= 78.75 && displayAngle < 101.25) idx = 3;
  else if (displayAngle >= 101.25 && displayAngle < 123.75) idx = 4;
  else if (displayAngle >= 123.75 && displayAngle < 146.25) idx = 5;
  else if (displayAngle >= 146.25 && displayAngle < 168.75) idx = 6;
  else if (displayAngle >= 168.75 && displayAngle < 191.25) idx = 7;
  else if (displayAngle >= 191.25 && displayAngle < 213.75) idx = 8;
  else if (displayAngle >= 213.75 && displayAngle < 236.25) idx = 9;
  else if (displayAngle >= 236.25 && displayAngle < 258.75) idx = 10;
  else if (displayAngle >= 258.75 && displayAngle < 281.25) idx = 11;
  else if (displayAngle >= 281.25 && displayAngle < 303.75) idx = 12;
  else if (displayAngle >= 303.75 && displayAngle < 326.25) idx = 13;
  else if (displayAngle >= 326.25 && displayAngle < 348.75) idx = 14;
  else if (displayAngle >= 348.75 || displayAngle < 11.25) idx = 15;

  if (idx >= 0 && idx != last) {
    if (last >= 0 && last < 16) display_item2(Cardinal_str[last]); // erase old
    display_item3(Cardinal_str[idx]);  // draw new
    last = idx;
  }
}

void display_item(int x, int y, String token, int txt_colour, int txt_size) {
  tft.setCursor(x, y);
  tft.setTextColor(txt_colour);
  tft.setTextSize(txt_size);
  tft.print(token);
  tft.setTextSize(2);
}

void display_item2(String token) {
  tft.setCursor(10, 20);
  tft.setTextColor(ILI9341_WHITE);
  tft.setFont(Arial_14);
  tft.print(token);
}

void display_item3(String token) {
  tft.setCursor(10, 20);
  tft.setTextColor(ILI9341_BLACK);
  tft.setFont(Arial_14);
  tft.print(token);
}

void arrow(int x2, int y2, int x1, int y1, int alength, int awidth, int colour) {
  float distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
  if (distance < 1.0f) return;
  int dx = x2 + (x1 - x2) * alength / distance;
  int dy = y2 + (y1 - y2) * alength / distance;
  int k = awidth / alength;
  int x2o = x2 - dx;
  int y2o = dy - y2;
  int x3 = y2o * k + dx;
  int y3 = x2o * k + dy;
  int x4 = dx - y2o * k;
  int y4 = dy - x2o * k;

  tft.drawLine(x1, y1, dx, dy, colour);
  tft.fillTriangle(x2, y2, x3, y3, x4, y4, colour);
}
