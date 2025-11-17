# AIOT-enabled-effective-administration-of-facilities
# A project to design a system that will integrate AI and IOT to administer facilities that will conserve and efficiently utilize energy to administer facilities
/*******************************************************
 AIoT Enabled Classroom/Office Automation System
 ESP32 + PIR + LDR + DHT11 + Relay Module
 MQTT-based monitoring and control
 *******************************************************/

// ------------------- Libraries -------------------------
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>

// ------------------- WiFi Credentials ------------------
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ------------------- MQTT Broker ------------------------
const char* mqtt_server = "YOUR_MQTT_BROKER_IP";   // e.g. 192.168.1.10
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ------------------- Sensors & Pins ---------------------
#define PIR_PIN 14
#define LDR_PIN 34
#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// ------------------- Relay Pins -------------------------
#define RELAY_LIGHT 25
#define RELAY_FAN   26
#define RELAY_SPARE1 27
#define RELAY_SPARE2 33

// ------------------- Control Parameters -------------------
int lightThreshold = 200;    // LDR threshold (0–4095 ADC scale)
float tempThreshold = 28.0;  // Temperature threshold (°C)
unsigned long noMotionTimeout = 300000; // 5 min (300000 ms)

unsigned long lastMotionTime = 0;
bool isOccupied = false;

// ------------------- MQTT Topics -------------------------
const char* topic_pub = "classroom1/sensordata";
const char* topic_sub_light = "classroom1/light/set";
const char* topic_sub_fan   = "classroom1/fan/set";


// -----------------------------------------------------------
//                 Wi-Fi Connection Function
// -----------------------------------------------------------
void setup_wifi() {
  delay(10);
  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected.");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
}


// -----------------------------------------------------------
//                 MQTT Callback (Subscriber)
// -----------------------------------------------------------
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Received message on topic: ");
  Serial.println(topic);

  String msg;
  for (int i = 0; i < length; i++) msg += (char)message[i];

  Serial.println("Message: " + msg);

  if (String(topic) == topic_sub_light) {
    if (msg == "ON") digitalWrite(RELAY_LIGHT, LOW);
    else digitalWrite(RELAY_LIGHT, HIGH);
  }

  if (String(topic) == topic_sub_fan) {
    if (msg == "ON") digitalWrite(RELAY_FAN, LOW);
    else digitalWrite(RELAY_FAN, HIGH);
  }
}


// -----------------------------------------------------------
//                 MQTT Reconnect Routine
// -----------------------------------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection... ");

    if (client.connect("ESP32_AIOT_CLASSROOM")) {
      Serial.println("Connected.");
      client.subscribe(topic_sub_light);
      client.subscribe(topic_sub_fan);
    }
    else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      Serial.println("Retrying in 3 seconds...");
      delay(3000);
    }
  }
}


// -----------------------------------------------------------
//                 Setup Function
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  pinMode(RELAY_LIGHT, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_SPARE1, OUTPUT);
  pinMode(RELAY_SPARE2, OUTPUT);

  // Fail-safe default OFF (relay HIGH for inactive if low-level trigger)
  digitalWrite(RELAY_LIGHT, HIGH);
  digitalWrite(RELAY_FAN, HIGH);

  dht.begin();
  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}


// -----------------------------------------------------------
//                 Main Control Logic
// -----------------------------------------------------------
void loop() {

  if (!client.connected()) reconnect();
  client.loop();

  int pirState = digitalRead(PIR_PIN);
  int ldrValue = analogRead(LDR_PIN);
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  // -------------------- Occupancy Logic --------------------
  if (pirState == HIGH) {
    isOccupied = true;
    lastMotionTime = millis();
  } 
  else {
    if (millis() - lastMotionTime > noMotionTimeout) {
      isOccupied = false;
    }
  }

  // -------------------- Lighting Control --------------------
  if (isOccupied) {
    if (ldrValue < lightThreshold) {
      digitalWrite(RELAY_LIGHT, LOW); // turn light ON
    } else {
      digitalWrite(RELAY_LIGHT, HIGH); // enough daylight
    }
  } else {
    digitalWrite(RELAY_LIGHT, HIGH); // no occupancy → OFF
  }

  // -------------------- Fan/Cooling Control --------------------
  if (isOccupied && temp >= tempThreshold) {
    digitalWrite(RELAY_FAN, LOW); // ON
  } else {
    digitalWrite(RELAY_FAN, HIGH); // OFF
  }

  // -------------------- Publish JSON Sensor Data --------------------
  StaticJsonDocument<256> doc;
  doc["motion"] = isOccupied ? 1 : 0;
  doc["ldr"] = ldrValue;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["light_state"] = digitalRead(RELAY_LIGHT) == LOW ? "ON" : "OFF";
  doc["fan_state"]   = digitalWrite(RELAY_FAN) == LOW ? "ON" : "OFF";

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

  client.publish(topic_pub, jsonBuffer);

  // Debug output
  Serial.print("PIR: "); Serial.print(isOccupied);
  Serial.print(" | LDR: "); Serial.print(ldrValue);
  Serial.print(" | Temp: "); Serial.print(temp);
  Serial.print(" | Hum: "); Serial.println(hum);

  delay(1500);
}
