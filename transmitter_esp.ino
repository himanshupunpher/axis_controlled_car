#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "MPU_AP";
const char* password = "12345678";
WiFiUDP Udp;
const int udpPort = 4210;

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];
uint8_t fifoBuffer[64];
bool dmpReady = false;
uint16_t packetSize;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");

  Udp.begin(udpPort);

  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP Ready");
  } else {
    Serial.println("DMP Init Failed");
    while (1);
  }
}

void loop() {
  if (!dmpReady) return;

  if (mpu.getFIFOCount() >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw = ypr[0] * 180 / PI;
    float pitch = ypr[1] * 180 / PI;
    float roll = ypr[2] * 180 / PI;

    String message = String(yaw, 1) + "," + String(pitch, 1) + "," + String(roll, 1);
    Serial.println("Sending: " + message);

    // Use WiFi.subnetMask() instead of softAPSubnetMask
    IPAddress localIP = WiFi.softAPIP();
    IPAddress subnet = WiFi.subnetMask();

    IPAddress broadcastIP(
      (localIP[0] & subnet[0]) | (~subnet[0] & 0xFF),
      (localIP[1] & subnet[1]) | (~subnet[1] & 0xFF),
      (localIP[2] & subnet[2]) | (~subnet[2] & 0xFF),
      (localIP[3] & subnet[3]) | (~subnet[3] & 0xFF)
    );

    Udp.beginPacket(broadcastIP, udpPort);
    Udp.print(message);
    Udp.endPacket();

    delay(50); // adjust update rate if needed
  }
}
