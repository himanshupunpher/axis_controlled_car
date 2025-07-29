import processing.serial.*;

Serial myPort;
float yaw, pitch, roll;

void setup() {
  size(600, 600, P3D);
  println(Serial.list());  // Show available ports
  myPort = new Serial(this, Serial.list()[2], 115200); // Change index if needed
  myPort.bufferUntil('\n');  // Read until newline
}

void draw() {
  background(20);
  lights();
  smooth();

  // Read serial data if available
  while (myPort.available() > 0) {
    String data = myPort.readStringUntil('\n');
    if (data != null) {
      data = trim(data);
      String[] values = split(data, ',');
      if (values.length == 3) {
        roll = float(values[0]);
        pitch = float(values[1]);
        yaw = float(values[2]);
      }
    }
  }

  translate(width/2, height/2, 0);

  // Apply rotations
  rotateZ(radians(yaw));   // Yaw (Z-axis)
  rotateX(radians(pitch)); // Pitch (X-axis)
  rotateY(radians(roll));  // Roll (Y-axis)

  // Optional: Draw axis lines
  drawAxes();

  // Draw cube
  fill(100, 150, 255);
  stroke(255);
  box(200);
}

// Draw XYZ axes
void drawAxes() {
  strokeWeight(3);
  
  stroke(255, 0, 0); // X - Red
  line(0, 0, 0, 100, 0, 0);

  stroke(0, 255, 0); // Y - Green
  line(0, 0, 0, 0, 100, 0);

  stroke(0, 0, 255); // Z - Blue
  line(0, 0, 0, 0, 0, 100);
}
