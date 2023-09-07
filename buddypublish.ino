// SIT210: Embedded System Development
// Task3.3D by Arjun
//buddy publish 

#include <ArduinoMqttClient.h>
#include <WiFi.h>

// Define your Wi-Fi credentials
const char wifiSSID[] = "Arjun's galaxyA31";     // Your network SSID (name)
const char wifiPassword[] = "mziboo299";         // Your network password

// Define ultrasonic sensor pins
const int triggerPin = 2;  // Trigger pin for the ultrasonic sensor
const int echoPin = 3;     // Echo pin for the ultrasonic sensor

// Create a WiFi client and an MQTT client
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Define MQTT broker and topic
const char mqttBroker[] = "broker.mqttdashboard.com";  // MQTT broker address
const int mqttPort = 1883;                             // MQTT broker port
const char mqttTopic[] = "SIT210/waves";               // MQTT topic

// Define the interval for distance measurements
const long measurementInterval = 1000;  // Interval for taking distance measurements
unsigned long previousMillis = 0;       // Stores the last time a measurement was taken

// Function to set up the Wi-Fi connection
void setupWiFi() {
  // Attempt to connect to WiFi network:
  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(wifiSSID);
  WiFi.begin(wifiSSID, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    // Failed to connect, retrying
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();
}

// Function to set up the MQTT connection
void setupMQTT() {
  // Attempt to connect to the MQTT broker:
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(mqttBroker);

  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();
}

// Function to send an MQTT message with distance information
void sendMQTTMessage(float distance) {
  // Send a message to the MQTT broker
  mqttClient.beginMessage(mqttTopic);
  mqttClient.print("Wave is detected: ");
  mqttClient.print("Distance: ");
  mqttClient.print(distance);
  mqttClient.endMessage();
  delay(1000);
}

void setup() {
  // Initialize serial and wait for the port to open
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect (needed for native USB port only)
  }

  // Initialize Wi-Fi and MQTT
  setupWiFi();
  setupMQTT();
}

void loop() {
  // Call poll() regularly to allow the library to send MQTT keep-alives
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= measurementInterval) {
    previousMillis = currentMillis;

    // Ultrasonic sensor measurements
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    float duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.println(distance);

    // Check if distance is less than 12 units, and if so, send an MQTT message
    if (distance < 12) {
      sendMQTTMessage(distance);
    }

    Serial.println();
  }
}
