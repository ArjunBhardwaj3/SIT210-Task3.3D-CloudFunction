// SIT210: Embedded Systems Development 
// Task3.3D Modified code for pat 

#include <ArduinoMqttClient.h>
#include <WiFi.h>

// Your Wi-Fi credentials
const char ssid[] = "Arjun's galaxyA31";     // network SSID (name)
const char pass[] = "mziboo299"; //network password

// Define ultrasonic sensor pins
const int trigPin = 2;
const int echoPin = 3;

// Create a WiFi client and an MQTT client
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// MQTT broker and topic
const char broker[] = "broker.mqttdashboard.com";
const int port = 1883;
const char topic[] = "SIT210/waves";

// Interval for distance measurements
const long interval = 1000;
unsigned long previousMillis = 0;

// LED pins
const int waveLedPin = 4; // LED for "wave" event
const int patLedPin = 5;  // LED for "pat" event

void setupWiFi() {
  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();
}

void setupMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();

  Serial.print("Subscribing to topic: ");
  Serial.println(topic);
  Serial.println();

  mqttClient.subscribe(topic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(topic);
  Serial.println();
}

void sendMQTTMessage(float distance) {
  mqttClient.beginMessage(topic);
  mqttClient.print("Wave is detected: ");
  mqttClient.print("Distance: ");
  mqttClient.print(distance);
  mqttClient.endMessage();
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  setupWiFi();
  setupMQTT();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(waveLedPin, OUTPUT);
  pinMode(patLedPin, OUTPUT);
}

void loop() {
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance < 12) {
      sendMQTTMessage(distance);
    }

    Serial.println();
  }
}

void waveLED() {
  // LED behavior for "wave" event
  digitalWrite(waveLedPin, HIGH);
  delay(100);
  digitalWrite(waveLedPin, LOW);
  delay(100);
}

void patLED() {
  // LED behavior for "pat" event
  digitalWrite(patLedPin, HIGH);
  delay(500);
  digitalWrite(patLedPin, LOW);
  delay(500);
}
