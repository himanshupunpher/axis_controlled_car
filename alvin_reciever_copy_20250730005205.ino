/*
 * Car Receiver for Gesture Controller
 * ESP8266 + L298N Motor Driver
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid = "MPU_AP";
const char* password = "12345678";

// UDP setup
WiFiUDP udp;
const int localPort = 8888;
char incomingPacket[128];

// ✅ Final L298N Motor Pin Mapping
const int IN1 = D2;  // Left motor direction 1
const int IN2 = D3;  // Left motor direction 2
const int IN3 = D4;  // Right motor direction 1
const int IN4 = D5;  // Right motor direction 2
const int ENB = D6;  // Right motor PWM
const int ENA = D1;  // Left motor PWM

// Motor config
const int DEAD_ZONE = 10;
bool debugEnabled = true;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  connectToWiFi();
  startUDP();
  startMDNS();

  Serial.println("Car Receiver Ready");
}

void loop() {
  int packetSize = udp.parsePacket();

  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
      handlePacket(String(incomingPacket));
    }
  }
}

// ----------------------

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void startUDP() {
  udp.begin(localPort);
  Serial.printf("UDP listening on port %d\n", localPort);
}

void startMDNS() {
  if (MDNS.begin("car-esp")) {
    Serial.println("mDNS responder started as 'car-esp.local'");
    MDNS.addService("car-control", "udp", localPort);
  }
}

// ----------------------

void handlePacket(String packet) {
  if (debugEnabled) {
    Serial.print("Received packet: ");
    Serial.println(packet);
  }

  int lIndex = packet.indexOf('L');
  int rIndex = packet.indexOf('R');
  int eIndex = packet.indexOf('E');

  if (lIndex == -1 || rIndex == -1 || eIndex == -1) return;

  int left = packet.substring(lIndex + 1, rIndex - 1).toInt();
  int right = packet.substring(rIndex + 1, eIndex - 1).toInt();
  int enable = packet.substring(eIndex + 1).toInt();

  if (!enable) {
    stopMotors();
    return;
  }

  controlL298Motor(ENA, IN1, IN2, left);    // Left motor
  controlL298Motor(ENB, IN3, IN4, right);   // Right motor
}

void controlL298Motor(int pwmPin, int dirPin1, int dirPin2, int value) {
  Serial.print(value);
  if (abs(value) < DEAD_ZONE) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  if (value >= 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  }

  analogWrite(pwmPin, abs(value));
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}