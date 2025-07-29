#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

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

    // Send yaw, pitch, roll over Serial
    Serial.print(ypr[0] * 180 / PI); Serial.print(",");
    Serial.print(ypr[1] * 180 / PI); Serial.print(",");
    Serial.println(ypr[2] * 180 / PI);
  }
}
