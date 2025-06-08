#include <Wire.h>
#include <U8g2lib.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define SCROLL_BUTTON 32
#define SELECT_BUTTON 25

#define MAX_KICKS 5
#define EEPROM_SIZE 128
#define EEPROM_ID_ADDR 0
#define EEPROM_WRITE_INDEX_ADDR 4
#define EEPROM_KICK_COUNT_ADDR 8
#define EEPROM_DATA_START_ADDR 12

const char* ssid = "Sunil_4g";
const char* password = "12345678";
const char* mqtt_server = "51.68.237.246";
const char* mqtt_topic = "ball/ao";

WiFiClient espClient;
PubSubClient client(espClient);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MPU6050 mpu;

struct KickData {
  int id;
  float speed;
  float strength;
};

enum AppState { HOME, KICK_COUNTDOWN, KICK_RECORDING, SHOW_RESULT, UPLOADING };
AppState state = HOME;

int menuIndex = 0;
int currentKickId = 0;
bool readyToSend = false;

// === FUNCTION DECLARATIONS ===
void showHomeScreen();
void showCountdown(int sec = 3);
void recordKick();
void showResult(float speed, float strength);
void showUploadingScreen();
void uploadStoredKicks();

void connectToWiFi() {
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    attempts++;
  }
}

void publishKick(const KickData& k) {
  if (!isfinite(k.speed) || !isfinite(k.strength)) return;
  if (WiFi.status() != WL_CONNECTED) return;

  client.setServer(mqtt_server, 1883);
  if (!client.connect("ESP32Client")) return;

  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"id\":\"%04d\",\"height\":0,\"speed\":%.2f,\"strength\":%.2f}",
           k.id, k.speed, k.strength);
  client.publish(mqtt_topic, payload);
}

int getNextKickId() {
  int id;
  EEPROM.get(EEPROM_ID_ADDR, id);
  id = (id + 1) % 10000;
  EEPROM.put(EEPROM_ID_ADDR, id);
  EEPROM.commit();
  return id;
}

void saveKickToEEPROM(KickData k) {
  int index, count;
  EEPROM.get(EEPROM_WRITE_INDEX_ADDR, index);
  EEPROM.get(EEPROM_KICK_COUNT_ADDR, count);

  index = constrain(index, 0, MAX_KICKS - 1);
  EEPROM.put(EEPROM_DATA_START_ADDR + index * sizeof(KickData), k);

  index = (index + 1) % MAX_KICKS;
  if (count < MAX_KICKS) count++;

  EEPROM.put(EEPROM_WRITE_INDEX_ADDR, index);
  EEPROM.put(EEPROM_KICK_COUNT_ADDR, count);
  EEPROM.commit();
}

void uploadStoredKicks() {
  showUploadingScreen();
  connectToWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  int index, count;
  EEPROM.get(EEPROM_WRITE_INDEX_ADDR, index);
  EEPROM.get(EEPROM_KICK_COUNT_ADDR, count);

  for (int i = 0; i < count; i++) {
    int readIndex = (index - count + i + MAX_KICKS) % MAX_KICKS;
    KickData k;
    EEPROM.get(EEPROM_DATA_START_ADDR + readIndex * sizeof(KickData), k);
    publishKick(k);
    delay(100);
  }

  EEPROM.put(EEPROM_KICK_COUNT_ADDR, 0);
  EEPROM.commit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
  EEPROM.begin(EEPROM_SIZE);
  mpu.initialize();

  pinMode(SCROLL_BUTTON, INPUT_PULLUP);
  pinMode(SELECT_BUTTON, INPUT_PULLUP);

  showHomeScreen();
}

void loop() {
  static bool lastScroll = HIGH;
  static bool lastSelect = HIGH;

  bool currentScroll = digitalRead(SCROLL_BUTTON);
  bool currentSelect = digitalRead(SELECT_BUTTON);

  if (lastScroll == HIGH && currentScroll == LOW) {
    menuIndex = (menuIndex + 1) % 2;
    if (state == HOME) showHomeScreen();
    delay(200);
  }

  if (lastSelect == HIGH && currentSelect == LOW) {
    delay(50);
    if (state == HOME) {
      if (menuIndex == 0) {
        state = KICK_COUNTDOWN;
        currentKickId = getNextKickId();
        
        // Show Kick ID before countdown
        char idBuffer[24];
        sprintf(idBuffer, "Kick ID: %04d", currentKickId);
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(10, 20, "Starting Kick...");
        u8g2.drawStr(10, 40, idBuffer);
        u8g2.sendBuffer();
        delay(1500);

        showCountdown();
      } else if (menuIndex == 1) {
        state = UPLOADING;
        uploadStoredKicks();
        state = HOME;
        showHomeScreen();
      }
    } else if (state == SHOW_RESULT && readyToSend) {
      readyToSend = false;
      state = HOME;
      showHomeScreen();
    }
  }

  lastScroll = currentScroll;
  lastSelect = currentSelect;

  if (state == KICK_COUNTDOWN) {
    for (int i = 3; i >= 1; i--) {
      showCountdown(i);
      delay(1000);
    }
    state = KICK_RECORDING;
    recordKick();
  }
}

void recordKick() {
  unsigned long startTime = millis();
  float maxG = 0;
  float accSum = 0;
  int count = 0;

  while (millis() - startTime < 1500) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gForce = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (gForce > maxG) maxG = gForce;

    float horizontalAcc = sqrt(ax_g * ax_g + ay_g * ay_g);
    accSum += horizontalAcc;
    count++;
    delay(10);
  }

  float avgAcc = accSum / count;
  float speed = avgAcc * 1.5;

  KickData k = { currentKickId, speed, maxG };
  saveKickToEEPROM(k);
  showResult(k.speed, k.strength);
  readyToSend = true;
  state = SHOW_RESULT;
}

// === DISPLAY ===

void showHomeScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  const char* options[2] = {"Start Throw", "Upload to AO"};
  for (int i = 0; i < 2; i++) {
    char line[24];
    sprintf(line, "%s%s", menuIndex == i ? "> " : "  ", options[i]);
    u8g2.drawStr(0, 20 + i * 20, line);
  }
  u8g2.sendBuffer();
}

void showCountdown(int sec) {
  char buffer[16];
  sprintf(buffer, "Throw in %d...", sec);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(10, 30, buffer);
  u8g2.sendBuffer();
}

void showResult(float speed, float strength) {
  char buffer1[24], buffer2[24];
  sprintf(buffer1, "Speed:    %.2f m/s", speed);
  sprintf(buffer2, "Strength: %.2f g", strength);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 24, "Throw Result:");
  u8g2.drawStr(0, 40, buffer1);
  u8g2.drawStr(0, 56, buffer2);
  u8g2.sendBuffer();
}

void showUploadingScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(10, 30, "Uploading throws...");
  u8g2.sendBuffer();
}
