//SIT210 : Embedded Systems Development
//Task3.3D by Arjun

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

// Wi-Fi and MQTT connection details
char wifiSSID[] = "Arjun's galaxyA31";    // Wi-Fi network SSID
char wifiPassword[] = "mziboo299";        // Wi-Fi password
int ledPin = 2;                           // Pin number for the LED

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "mqtt-dashboard.com";  // MQTT broker address
int mqttPort = 1883;                             // MQTT broker port
const char mqttTopic[] = "arjun-wave";            // MQTT topic to subscribe to

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  // Attempt to connect to the Wi-Fi network
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(wifiSSID);
  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    // Failed to connect, retry every 5 seconds
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  // Attempt to connect to the MQTT broker
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(mqttBroker);

  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);  // Infinite loop if MQTT connection fails
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // Subscribe to the specified MQTT topic
  Serial.print("Subscribing to topic: ");
  Serial.println(mqttTopic);
  Serial.println();
  mqttClient.subscribe(mqttTopic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(mqttTopic);
  Serial.println();
}

void loop() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    // Received an MQTT message
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    // Print the message content
    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();

    // Blink the LED in response to the message
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);

    Serial.println();
  }
}
