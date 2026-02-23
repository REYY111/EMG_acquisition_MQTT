/*************************************************
 * FINAL EMG LOGGER v3 (MQTT + CHART.JS READY)
 * ESP32 - Stable 1kHz Sampling
 *************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================= WIFI =================
const char* ssid     = "reyhannn";
const char* password = "12345678";

// ================= MQTT =================
const char* mqtt_server = "10.170.222.19";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "sensor/emg";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ================= EMG PIN =================
#define EMG_PIN   34
#define LO_PLUS   25
#define LO_MINUS  26

// ================= FILTER =================

// ---- DC Highpass (~1.6Hz) ----
const float ALPHA_DC = 0.99;
float last_raw = 0;
float last_dc  = 0;

// ---- NOTCH 50Hz ----
const float b2 = -1.9021;
const float a2 = -1.8831;
const float a3 = 0.9801;

// ⚠️ renamed (avoid math.h conflict)
float x1_n = 0, x2_n = 0;
float y1_n = 0, y2_n = 0;

// ---- LOWPASS ----
const float ALPHA_LP = 0.30;
float lp_emg = 0;

// ---- ENVELOPE ----
const float ALPHA_ENV = 0.05;
float envelope = 0;

// ================= TIMING =================
unsigned long lastMicros = 0;
const int SAMPLE_PERIOD_US = 1000; // 1kHz
unsigned long t0;

// =================================================
// WIFI CONNECT
// =================================================
void setupWiFi() {

  Serial.print("Connecting WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// =================================================
// MQTT RECONNECT
// =================================================
void reconnectMQTT() {

  while (!mqtt.connected()) {

    Serial.print("Connecting MQTT...");

    if (mqtt.connect("ESP32_EMG")) {
      Serial.println("connected");
    } else {
      Serial.print("failed rc=");
      Serial.println(mqtt.state());
      delay(2000);
    }
  }
}

// =================================================
void setup() {

  Serial.begin(115200);

  analogReadResolution(12);

  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);

  delay(2000);

  setupWiFi();

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setKeepAlive(60);

  // IMPORTANT for fast publish
  mqtt.setBufferSize(512);

  t0 = millis();

  Serial.println("✅ EMG MQTT READY");
}

// =================================================
void loop() {

  if (!mqtt.connected())
    reconnectMQTT();

  mqtt.loop();

  // ===== STABLE 1kHz SAMPLING =====
  if (micros() - lastMicros < SAMPLE_PERIOD_US)
    return;

  lastMicros += SAMPLE_PERIOD_US;

  // ===== LEAD OFF DETECTION =====
  if (digitalRead(LO_PLUS) || digitalRead(LO_MINUS)) {
    return;
  }

  // ===== ADC AVERAGING =====
  int rawADC = 0;
  for(int i=0;i<4;i++)
    rawADC += analogRead(EMG_PIN);

  rawADC /= 4;

  float x0 = (float)rawADC;

  // ================= DC REMOVAL =================
  float dc_filtered = x0 - last_raw + (ALPHA_DC * last_dc);

  last_raw = x0;
  last_dc  = dc_filtered;

  // ================= NOTCH 50Hz =================
  float notch_out =
        dc_filtered
      + (b2 * x1_n)
      + x2_n
      - (a2 * y1_n)
      - (a3 * y2_n);

  x2_n = x1_n;
  x1_n = dc_filtered;
  y2_n = y1_n;
  y1_n = notch_out;

  // ================= LOWPASS =================
  lp_emg = (ALPHA_LP * notch_out)
         + ((1.0 - ALPHA_LP) * lp_emg);

  float clean_emg = lp_emg;

  // ================= ENVELOPE =================
  float rectified = abs(clean_emg);

  envelope = (ALPHA_ENV * rectified)
           + ((1.0 - ALPHA_ENV) * envelope);

  // ================= MQTT JSON =================
  char payload[200];

  snprintf(payload, sizeof(payload),
    "{\"raw\":%d,"
    "\"clean\":%.2f,"
    "\"envelope\":%.2f,"
    "\"ts\":%lu}",
    rawADC,
    clean_emg,
    envelope,
    millis() - t0
  );

  mqtt.publish(mqtt_topic, payload);
}