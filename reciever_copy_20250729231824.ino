#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Servo.h>

#define ENA D1
#define IN1 D2
#define IN2 D3
#define ENB D6
#define IN3 D4
#define IN4 D5
#define LED_PIN LED_BUILTIN  // Onboard LED (usually GPIO2 on ESP8266)

const char* ssid = "MPU_AP";
const char* password = "12345678";

WiFiClient client;
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED OFF

  WiFi.begin(ssid, password);
  Serial.println("Connecting to transmitter...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, LOW); // Keep LED OFF while not connected
  }

  Serial.println("\nConnected!");
  digitalWrite(LED_PIN, HIGH); // Turn LED ON after connection

  server.begin();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW); // Turn LED OFF if disconnected
    return;
  } else {
    digitalWrite(LED_PIN, HIGH); // Ensure LED stays ON if connected
  }

  WiFiClient client = server.available();
  if (client) {
    String data = client.readStringUntil('\n');
    Serial.println("Received: " + data);

    float yaw = data.substring(0, data.indexOf(',')).toFloat();
    float pitch = data.substring(data.indexOf(',') + 1, data.lastIndexOf(',')).toFloat();
    float roll = data.substring(data.lastIndexOf(',') + 1).toFloat();

    // Control logic based on pitch/roll (example)
    if (pitch > 20) {
      // Move forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 700);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 700);
    } else if (pitch < -20) {
      // Move backward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 700);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 700);
    } else {
      // Stop
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

    delay(50); // Small delay for stability
  }
}
