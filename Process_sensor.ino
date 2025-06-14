#define BLYNK_TEMPLATE_NAME "Fire Detection and Suppression System Using IoT an"
#define BLYNK_AUTH_TOKEN "7TFV59n-ZIyzilkWjf5_E7FgWhMgPU5V"
#define BLYNK_TEMPLATE_ID "TMPL6H8P_STa0"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <math.h>

// WiFi & MQTT
const char* ssid = "Redmi 13";
const char* pass = "99999999";
const char* mqtt_server = "192.168.117.72";
const int mqtt_port = 1883;
const char* mqtt_topic_fire = "esp32/fire_detection";

// Sensor & Pin
#define DHTPIN 27
#define DHTTYPE DHT11
#define MQ2_PIN 33
#define FLAME_PIN 26
#define BUZZER_PIN 32 
#define ALARM_LED_PIN 14

// Ngưỡng
const float TEMP_THRESHOLD = 45;
const float CO_PPM_THRESHOLD = 250; // ngưỡng ppm 

// MQ2 Constants
const float RL = 10;  // kOhm
float Ro = 50; // Will be calibrated

// Biến trạng thái
bool aiFireDetected = false;
unsigned long fireDetectedAt = 0;
const unsigned long ALERT_DURATION = 10000;  // 10 giây

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

float getSensorResistance(int raw_adc) {
  float voltage = raw_adc * 3.3 / 4095.0;
  float rs = ((5.0 - voltage) / voltage) * RL;
  return rs;
}

float getPPM_CO(float rs_ro_ratio) {
  float log_ppm = (-0.43) * log10(rs_ro_ratio) + 2.595;
  return pow(10, log_ppm);
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == mqtt_topic_fire) {
    aiFireDetected = (message == "1");
  }
}

bool reconnectMQTT() {
  if (!mqttClient.connected()) {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      mqttClient.subscribe(mqtt_topic_fire);
    }
  }
  return mqttClient.connected();
}

void sendSensorData() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int mq2Value = analogRead(MQ2_PIN);
  int flameValue = digitalRead(FLAME_PIN); // digital
  bool flameDetected = (flameValue == LOW); // LOW = phát hiện lửa

  float rs = getSensorResistance(mq2Value);
  float ratio = rs / Ro;
  float co_ppm = getPPM_CO(ratio);

  bool tempHigh = temperature > TEMP_THRESHOLD;
  bool gasHigh = co_ppm > CO_PPM_THRESHOLD;

  Serial.println("---- SENSOR DATA ----");
  Serial.print("Temp: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
  Serial.print("MQ2: "); Serial.println(mq2Value);
  Serial.print("CO PPM: "); Serial.println(co_ppm);
  Serial.print("Flame (digital): "); Serial.println(flameDetected ? "YES" : "NO");
  Serial.print("AI fire: "); Serial.println(aiFireDetected ? "YES" : "NO");
  Serial.println("----------------------");

  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V3, !flameValue); // FLAME_PIN
  Blynk.virtualWrite(V5, co_ppm);

  bool otherDanger = gasHigh || flameDetected || aiFireDetected;
  Serial.println(otherDanger);

  if (tempHigh || otherDanger) {
    fireDetectedAt = millis();
    Blynk.virtualWrite(V4, 1);  // Gửi giá trị 1 khi có cảnh báo

  } else {
    Blynk.virtualWrite(V4, 0);  // Không có cảnh báo
  }

  // Hiển thị cảnh báo theo đúng điều kiện
  bool inAlert = (millis() - fireDetectedAt <= ALERT_DURATION);

  if (inAlert) {
    // Nếu có cháy bởi gas/flame/AI → bật còi
    digitalWrite(BUZZER_PIN, otherDanger ? HIGH : LOW);

    // LED nhấp nháy trong cảnh báo
    for (int i = 0; i < 20; i++) {
      digitalWrite(ALARM_LED_PIN, HIGH);
      delay(250);
      digitalWrite(ALARM_LED_PIN, LOW);
      delay(250);
    }
  } else {
    digitalWrite(ALARM_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}


void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
    }
  }
}

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(MQ2_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(ALARM_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  WiFi.begin(ssid, pass);
  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 10000; 

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Không thể kết nối WiFi sau 10 giây. Tiếp tục chạy chế độ offline.");
  } else {
    Serial.println("Đã kết nối WiFi thành công.");
  }

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();

  timer.setInterval(2000L, sendSensorData);
  timer.setInterval(30000L, checkWiFiConnection);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
  }
  timer.run();
}
