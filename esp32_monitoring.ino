/*
 * ESP32 DevKit V1 - Environmental Monitoring & Control System
 * ============================================================
 * Hardware:
 *   - 2x Relay Dual Channel
 *   - 2x DHT22 Sensor
 *   - 1x LCD 20x4 (I2C)
 *   - 1x MicroSD Card Module
 *
 * Relay 1 (CH1 & CH2) → Beban 1 & Beban 2 (kontrol kelembaban)
 * Relay 2 (CH3 & CH4) → Beban 3 & Beban 4 (kontrol suhu + timer)
 *
 * DHT22 #1 (bawah, dekat Beban 3) → kontrol suhu Beban 3
 * DHT22 #2 (atas, dekat Beban 4)  → kontrol kelembaban bersama DHT1
 */

#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

// ─── WiFi Credentials ──────────────────────────────────────────────────
const char* WIFI_SSID     = "rumah aku";
const char* WIFI_PASSWORD = "polban46";

// ─── Pin Definitions ────────────────────────────────────────────────────
// DHT22
#define DHT1_PIN  4   // DHT22 #1 (bawah / dekat Beban 3)
#define DHT2_PIN  2   // DHT22 #2 (atas  / dekat Beban 4)
#define DHT_TYPE  DHT22

// Relay (LOW = aktif untuk relay module umumnya)
// Relay 1 (dual channel)
#define RELAY1_CH1  25  // Beban 1
#define RELAY1_CH2  26  // Beban 2

// Relay 2 (dual channel)
#define RELAY2_CH3  27  // Beban 3 (suhu + timer)
#define RELAY2_CH4  14  // Beban 4 (timer saja)

// MicroSD (SPI)
#define SD_CS_PIN   5   // Chip Select SD Card
// SPI default: SCK=18, MISO=19, MOSI=23

// I2C LCD (default SDA=21, SCL=22)
#define LCD_ADDR    0x27  // Ubah ke 0x3F jika tidak terdeteksi
#define LCD_COLS    20
#define LCD_ROWS    4

// ─── Threshold / Setpoint ───────────────────────────────────────────────
float HUM_ON_THRESHOLD   = 80.0f;  // Relay1 ON  jika kelembaban < nilai ini
float HUM_OFF_THRESHOLD  = 90.0f;  // Relay1 OFF jika kelembaban >= nilai ini
float TEMP_ON_THRESHOLD  = 24.0f;  // Beban3 ON  jika suhu > nilai ini
float TEMP_OFF_THRESHOLD = 23.0f;  // Beban3 OFF jika suhu <= nilai ini

// ─── Timer Beban 3 & 4 ──────────────────────────────────────────────────
unsigned long TIMER_INTERVAL_MS = 3600000UL;  // 1 jam (ms)
unsigned long TIMER_ON_DURATION = 300000UL;   // 5 menit (ms)

// ─── Logging Interval ───────────────────────────────────────────────────
unsigned long LOG_INTERVAL_MS = 60000UL;  // 1 menit

// ─── Global State ───────────────────────────────────────────────────────
bool systemRunning  = true;   // Start/stop via web
bool beban1Active   = false;
bool beban2Active   = false;
bool beban3Active   = false;
bool beban4Active   = false;

float dht1Temp = 0, dht1Hum = 0;
float dht2Temp = 0, dht2Hum = 0;

unsigned long lastTimerStart = 0;
bool timerPhaseOn = false;

unsigned long lastLogTime   = 0;
unsigned long lastSensorRead = 0;
unsigned long lastLcdUpdate = 0;
int lcdPage = 0;

// ─── EEPROM Addresses ───────────────────────────────────────────────────
#define EEPROM_SIZE 64
#define ADDR_HUM_ON  0   // float 4 bytes
#define ADDR_HUM_OFF 4
#define ADDR_TEMP_ON 8
#define ADDR_TEMP_OFF 12
#define ADDR_LOG_INT 16  // unsigned long 4 bytes
#define ADDR_TMR_INT 20
#define ADDR_TMR_DUR 24

// ─── Objects ─────────────────────────────────────────────────────────────
DHT dht1(DHT1_PIN, DHT_TYPE);
DHT dht2(DHT2_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
WebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200); // UTC+7 WIB

// ─── Forward Declarations ───────────────────────────────────────────────
void setupWiFi();
void setupWebServer();
void readSensors();
void controlRelays();
void updateLCD();
void logToSD();
void saveSettings();
void loadSettings();
String getTimestamp();
String buildJsonStatus();
String buildJsonHistory();

// ─── SD History Buffer ───────────────────────────────────────────────────
struct SensorRecord {
  char timestamp[20];
  float t1, h1, t2, h2;
  bool b1, b2, b3, b4;
};
#define HISTORY_MAX 100
SensorRecord history[HISTORY_MAX];
int historyCount = 0;
int historyHead  = 0;

// ════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== ESP32 Monitoring System Boot ==="));

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();

  // GPIO Relay (HIGH = mati untuk active-LOW relay module)
  pinMode(RELAY1_CH1, OUTPUT); digitalWrite(RELAY1_CH1, HIGH);
  pinMode(RELAY1_CH2, OUTPUT); digitalWrite(RELAY1_CH2, HIGH);
  pinMode(RELAY2_CH3, OUTPUT); digitalWrite(RELAY2_CH3, HIGH);
  pinMode(RELAY2_CH4, OUTPUT); digitalWrite(RELAY2_CH4, HIGH);

  // DHT
  dht1.begin();
  dht2.begin();

  // LCD
  Wire.begin();
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print(F("Initializing...     "));
  lcd.setCursor(0,1); lcd.print(F("ESP32 Monitor v1.0  "));

  // SD Card
  SPI.begin();
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("[SD] GAGAL! Periksa modul dan kartu SD."));
    lcd.setCursor(0,2); lcd.print(F("SD: GAGAL!          "));
  } else {
    Serial.println(F("[SD] OK"));
    lcd.setCursor(0,2); lcd.print(F("SD: OK              "));
    // Buat header CSV jika file belum ada
    if (!SD.exists("/data.csv")) {
      File f = SD.open("/data.csv", FILE_WRITE);
      if (f) {
        f.println("Timestamp,Suhu1(C),Hum1(%),Suhu2(C),Hum2(%),Beban1,Beban2,Beban3,Beban4");
        f.close();
      }
    }
  }

  // WiFi & NTP
  setupWiFi();
  timeClient.begin();
  timeClient.update();

  // Web Server
  setupWebServer();

  // Timer beban 3 & 4: mulai dari sekarang
  lastTimerStart = millis();

  lcd.setCursor(0,3); lcd.print(F("Sistem Siap!        "));
  delay(1500);

  Serial.println(F("=== Setup Selesai ==="));
}

// ════════════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();
  timeClient.update();

  unsigned long now = millis();

  // Baca sensor setiap 2 detik
  if (now - lastSensorRead >= 2000) {
    lastSensorRead = now;
    readSensors();
    controlRelays();
  }

  // Update LCD setiap 3 detik, bergantian halaman
  if (now - lastLcdUpdate >= 3000) {
    lastLcdUpdate = now;
    updateLCD();
    lcdPage = (lcdPage + 1) % 3;
  }

  // Log ke SD & buffer setiap LOG_INTERVAL_MS
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;
    logToSD();
  }
}

// ════════════════════════════════════════════════════════════════════════
// WIFI SETUP
// ════════════════════════════════════════════════════════════════════════
void setupWiFi() {
  lcd.setCursor(0,3); lcd.print(F("WiFi Connecting...  "));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Terhubung: %s\n", WiFi.localIP().toString().c_str());
    lcd.setCursor(0,3);
    lcd.print(WiFi.localIP().toString() + "        ");
  } else {
    Serial.println(F("\n[WiFi] GAGAL - Mode Offline"));
    lcd.setCursor(0,3); lcd.print(F("WiFi: GAGAL (Offline)"));
  }
}

// ════════════════════════════════════════════════════════════════════════
// WEB SERVER
// ════════════════════════════════════════════════════════════════════════
void setupWebServer() {
  // CORS helper
  auto cors = [&]() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  };

  // GET /status → JSON real-time
  server.on("/status", HTTP_GET, [&, cors]() {
    cors();
    server.send(200, "application/json", buildJsonStatus());
  });

  // GET /history → JSON riwayat (max HISTORY_MAX record)
  server.on("/history", HTTP_GET, [&, cors]() {
    cors();
    server.send(200, "application/json", buildJsonHistory());
  });

  // POST /control → {"run": true/false}
  server.on("/control", HTTP_POST, [&, cors]() {
    cors();
    if (server.hasArg("plain")) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("run")) {
        systemRunning = doc["run"].as<bool>();
        if (!systemRunning) {
          // Matikan semua relay saat stop
          digitalWrite(RELAY1_CH1, HIGH);
          digitalWrite(RELAY1_CH2, HIGH);
          digitalWrite(RELAY2_CH3, HIGH);
          digitalWrite(RELAY2_CH4, HIGH);
          beban1Active = beban2Active = beban3Active = beban4Active = false;
        }
      }
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });

  // POST /settings → threshold & interval
  server.on("/settings", HTTP_POST, [&, cors]() {
    cors();
    if (server.hasArg("plain")) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("humOn"))      HUM_ON_THRESHOLD  = doc["humOn"].as<float>();
      if (doc.containsKey("humOff"))     HUM_OFF_THRESHOLD = doc["humOff"].as<float>();
      if (doc.containsKey("tempOn"))     TEMP_ON_THRESHOLD = doc["tempOn"].as<float>();
      if (doc.containsKey("tempOff"))    TEMP_OFF_THRESHOLD= doc["tempOff"].as<float>();
      if (doc.containsKey("logInt"))     LOG_INTERVAL_MS   = (unsigned long)(doc["logInt"].as<int>() * 1000);
      if (doc.containsKey("timerInt"))   TIMER_INTERVAL_MS = (unsigned long)(doc["timerInt"].as<int>() * 1000);
      if (doc.containsKey("timerDur"))   TIMER_ON_DURATION = (unsigned long)(doc["timerDur"].as<int>() * 1000);
      saveSettings();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });

  // GET /settings → baca setelan saat ini
  server.on("/settings", HTTP_GET, [&, cors]() {
    cors();
    StaticJsonDocument<256> doc;
    doc["humOn"]    = HUM_ON_THRESHOLD;
    doc["humOff"]   = HUM_OFF_THRESHOLD;
    doc["tempOn"]   = TEMP_ON_THRESHOLD;
    doc["tempOff"]  = TEMP_OFF_THRESHOLD;
    doc["logInt"]   = LOG_INTERVAL_MS / 1000;
    doc["timerInt"] = TIMER_INTERVAL_MS / 1000;
    doc["timerDur"] = TIMER_ON_DURATION / 1000;
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // GET /download → unduh CSV dari SD
  server.on("/download", HTTP_GET, [&, cors]() {
    cors();
    File f = SD.open("/data.csv", FILE_READ);
    if (!f) { server.send(404, "text/plain", "File tidak ditemukan"); return; }
    server.sendHeader("Content-Disposition", "attachment; filename=\"data.csv\"");
    server.streamFile(f, "text/csv");
    f.close();
  });

  // OPTIONS (pre-flight CORS)
  server.onNotFound([&, cors]() {
    if (server.method() == HTTP_OPTIONS) { cors(); server.send(204); }
    else server.send(404, "text/plain", "Not Found");
  });

  server.begin();
  Serial.println(F("[WebServer] Berjalan di port 80"));
}

// ════════════════════════════════════════════════════════════════════════
// SENSOR
// ════════════════════════════════════════════════════════════════════════
void readSensors() {
  float t1 = dht1.readTemperature();
  float h1 = dht1.readHumidity();
  float t2 = dht2.readTemperature();
  float h2 = dht2.readHumidity();

  if (!isnan(t1)) dht1Temp = t1;
  if (!isnan(h1)) dht1Hum  = h1;
  if (!isnan(t2)) dht2Temp = t2;
  if (!isnan(h2)) dht2Hum  = h2;
}

// ════════════════════════════════════════════════════════════════════════
// KONTROL RELAY
// ════════════════════════════════════════════════════════════════════════
void controlRelays() {
  if (!systemRunning) return;

  // ── RELAY 1 (Beban 1 & 2): kontrol kelembaban ──────────────────────
  // Nyala bersama jika KEDUA kelembaban (DHT1 & DHT2) < HUM_ON_THRESHOLD
  // Mati jika salah satu menyentuh HUM_OFF_THRESHOLD
  bool r1ShouldOn;
  if (!beban1Active) {
    r1ShouldOn = (dht1Hum < HUM_ON_THRESHOLD) && (dht2Hum < HUM_ON_THRESHOLD);
  } else {
    r1ShouldOn = (dht1Hum < HUM_OFF_THRESHOLD) && (dht2Hum < HUM_OFF_THRESHOLD);
  }
  beban1Active = beban2Active = r1ShouldOn;
  digitalWrite(RELAY1_CH1, r1ShouldOn ? LOW : HIGH);
  digitalWrite(RELAY1_CH2, r1ShouldOn ? LOW : HIGH);

  // ── RELAY 2 (Beban 3 & 4): timer + suhu ────────────────────────────
  unsigned long now = millis();
  unsigned long elapsed = now - lastTimerStart;

  // Fase timer
  if (!timerPhaseOn && elapsed >= TIMER_INTERVAL_MS) {
    timerPhaseOn   = true;
    lastTimerStart = now;
  } else if (timerPhaseOn && elapsed >= TIMER_ON_DURATION) {
    timerPhaseOn   = false;
    lastTimerStart = now;
  }

  // Beban 4: hanya timer
  beban4Active = timerPhaseOn;
  digitalWrite(RELAY2_CH4, beban4Active ? LOW : HIGH);

  // Beban 3: timer ATAU suhu DHT1
  bool tempCondition;
  if (!beban3Active) {
    tempCondition = (dht1Temp > TEMP_ON_THRESHOLD);
  } else {
    tempCondition = (dht1Temp > TEMP_OFF_THRESHOLD);
  }
  beban3Active = timerPhaseOn || tempCondition;
  digitalWrite(RELAY2_CH3, beban3Active ? LOW : HIGH);
}

// ════════════════════════════════════════════════════════════════════════
// LCD (bergantian 3 halaman)
// ════════════════════════════════════════════════════════════════════════
void updateLCD() {
  lcd.clear();
  switch(lcdPage) {
    case 0: // Suhu & Kelembaban DHT1 & DHT2
      lcd.setCursor(0,0); lcd.print(F("== SENSOR DATA ==   "));
      lcd.setCursor(0,1);
      lcd.printf("DHT1 T:%.1fC H:%.0f%%  ", dht1Temp, dht1Hum);
      lcd.setCursor(0,2);
      lcd.printf("DHT2 T:%.1fC H:%.0f%%  ", dht2Temp, dht2Hum);
      lcd.setCursor(0,3);
      lcd.print(systemRunning ? "Status: RUNNING     " : "Status: STOPPED     ");
      break;

    case 1: // Status Beban
      lcd.setCursor(0,0); lcd.print(F("== STATUS BEBAN ==  "));
      lcd.setCursor(0,1);
      lcd.printf("B1:%s B2:%s          ", beban1Active?"ON ":"OFF", beban2Active?"ON ":"OFF");
      lcd.setCursor(0,2);
      lcd.printf("B3:%s B4:%s          ", beban3Active?"ON ":"OFF", beban4Active?"ON ":"OFF");
      lcd.setCursor(0,3);
      lcd.print(getTimestamp().substring(11).c_str()); // Tampilkan jam
      break;

    case 2: // IP & Threshold
      lcd.setCursor(0,0); lcd.print(F("== NETWORK INFO ==  "));
      lcd.setCursor(0,1);
      lcd.print(WiFi.localIP().toString().c_str());
      lcd.setCursor(0,2);
      lcd.printf("HumON<%.0f OFF>=%.0f  ", HUM_ON_THRESHOLD, HUM_OFF_THRESHOLD);
      lcd.setCursor(0,3);
      lcd.printf("TmpON>%.0f OFF<=%.0f ", TEMP_ON_THRESHOLD, TEMP_OFF_THRESHOLD);
      break;
  }
}

// ════════════════════════════════════════════════════════════════════════
// LOG KE SD CARD
// ════════════════════════════════════════════════════════════════════════
void logToSD() {
  String ts = getTimestamp();

  // Simpan ke buffer memori
  SensorRecord& rec = history[historyHead];
  ts.toCharArray(rec.timestamp, 20);
  rec.t1 = dht1Temp; rec.h1 = dht1Hum;
  rec.t2 = dht2Temp; rec.h2 = dht2Hum;
  rec.b1 = beban1Active; rec.b2 = beban2Active;
  rec.b3 = beban3Active; rec.b4 = beban4Active;
  historyHead = (historyHead + 1) % HISTORY_MAX;
  if (historyCount < HISTORY_MAX) historyCount++;

  // Tulis ke SD
  File f = SD.open("/data.csv", FILE_APPEND);
  if (f) {
    f.printf("%s,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
      ts.c_str(),
      dht1Temp, dht1Hum,
      dht2Temp, dht2Hum,
      beban1Active, beban2Active,
      beban3Active, beban4Active);
    f.close();
    Serial.printf("[SD] Log: %s\n", ts.c_str());
  } else {
    Serial.println(F("[SD] Gagal menulis!"));
  }
}

// ════════════════════════════════════════════════════════════════════════
// UTILITIES
// ════════════════════════════════════════════════════════════════════════
String getTimestamp() {
  timeClient.update();
  time_t t = timeClient.getEpochTime();
  struct tm *tm_info = localtime(&t);
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
  return String(buf);
}

String buildJsonStatus() {
  StaticJsonDocument<512> doc;
  doc["ts"]         = getTimestamp();
  doc["running"]    = systemRunning;
  doc["dht1"]["t"]  = dht1Temp;
  doc["dht1"]["h"]  = dht1Hum;
  doc["dht2"]["t"]  = dht2Temp;
  doc["dht2"]["h"]  = dht2Hum;
  doc["beban1"]     = beban1Active;
  doc["beban2"]     = beban2Active;
  doc["beban3"]     = beban3Active;
  doc["beban4"]     = beban4Active;
  doc["timerOn"]    = timerPhaseOn;
  // Sisa waktu timer (detik)
  unsigned long elapsed = millis() - lastTimerStart;
  if (timerPhaseOn) {
    doc["timerRemaining"] = (TIMER_ON_DURATION > elapsed) ? (TIMER_ON_DURATION - elapsed) / 1000 : 0;
  } else {
    doc["timerRemaining"] = (TIMER_INTERVAL_MS > elapsed) ? (TIMER_INTERVAL_MS - elapsed) / 1000 : 0;
  }
  String out; serializeJson(doc, out);
  return out;
}

String buildJsonHistory() {
  // Kirim sebagai array JSON
  String out = "[";
  int start = (historyCount < HISTORY_MAX) ? 0 : historyHead;
  for (int i = 0; i < historyCount; i++) {
    int idx = (start + i) % HISTORY_MAX;
    SensorRecord& r = history[idx];
    if (i > 0) out += ",";
    out += "{\"ts\":\"";  out += r.timestamp;
    out += "\",\"t1\":";  out += r.t1;
    out += ",\"h1\":";    out += r.h1;
    out += ",\"t2\":";    out += r.t2;
    out += ",\"h2\":";    out += r.h2;
    out += ",\"b1\":";    out += r.b1 ? "true" : "false";
    out += ",\"b2\":";    out += r.b2 ? "true" : "false";
    out += ",\"b3\":";    out += r.b3 ? "true" : "false";
    out += ",\"b4\":";    out += r.b4 ? "true" : "false";
    out += "}";
  }
  out += "]";
  return out;
}

void saveSettings() {
  EEPROM.put(ADDR_HUM_ON,  HUM_ON_THRESHOLD);
  EEPROM.put(ADDR_HUM_OFF, HUM_OFF_THRESHOLD);
  EEPROM.put(ADDR_TEMP_ON, TEMP_ON_THRESHOLD);
  EEPROM.put(ADDR_TEMP_OFF,TEMP_OFF_THRESHOLD);
  EEPROM.put(ADDR_LOG_INT, LOG_INTERVAL_MS);
  EEPROM.put(ADDR_TMR_INT, TIMER_INTERVAL_MS);
  EEPROM.put(ADDR_TMR_DUR, TIMER_ON_DURATION);
  EEPROM.commit();
  Serial.println(F("[EEPROM] Pengaturan tersimpan"));
}

void loadSettings() {
  float tmpF; unsigned long tmpL;
  EEPROM.get(ADDR_HUM_ON,  tmpF); if (!isnan(tmpF) && tmpF > 0) HUM_ON_THRESHOLD  = tmpF;
  EEPROM.get(ADDR_HUM_OFF, tmpF); if (!isnan(tmpF) && tmpF > 0) HUM_OFF_THRESHOLD = tmpF;
  EEPROM.get(ADDR_TEMP_ON, tmpF); if (!isnan(tmpF) && tmpF > 0) TEMP_ON_THRESHOLD = tmpF;
  EEPROM.get(ADDR_TEMP_OFF,tmpF); if (!isnan(tmpF) && tmpF > 0) TEMP_OFF_THRESHOLD= tmpF;
  EEPROM.get(ADDR_LOG_INT, tmpL); if (tmpL > 0 && tmpL < 86400000UL) LOG_INTERVAL_MS   = tmpL;
  EEPROM.get(ADDR_TMR_INT, tmpL); if (tmpL > 0 && tmpL < 86400000UL) TIMER_INTERVAL_MS = tmpL;
  EEPROM.get(ADDR_TMR_DUR, tmpL); if (tmpL > 0 && tmpL < 3600000UL)  TIMER_ON_DURATION = tmpL;
  Serial.println(F("[EEPROM] Pengaturan dimuat"));
}
