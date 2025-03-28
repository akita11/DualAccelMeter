import processing.serial.*;
Serial myPort;

float[] yaw = new float[2];
float[] pitch = new float[2];
float[] roll = new float[2];

void setup() {
  size(800, 600, P3D);
  //myPort = new Serial(this, Serial.list()[0], 115200);
  myPort = new Serial(this, "/dev/cu.usbmodem101", 115200);  // Mac
  //myPort = new Serial(this, "COM5", 115200);                    // Windows
  textSize(24);
  textAlign(CENTER, CENTER);
}

void draw() {
  serialEvent();  // シリアルからデータ取得
  background(255);
  lights();

  translate(width / 2, height / 2); // 画面中心へ移動

  // i=0（左）
  pushMatrix();
  translate(-150, 0, 0);
  applyRotationMatrix(0);
  drawObject(0);
  drawLabel("0", 0, -120, 0);
  popMatrix();

  // i=1（右）
  pushMatrix();
  translate(150, 0, 0);
  applyRotationMatrix(1);
  drawObject(1);
  drawLabel("1", 0, -120, 0);
  popMatrix();
}

void serialEvent() {
  String message;
  while ((message = myPort.readStringUntil('\n')) != null) {
    message = trim(message);
    String[] list = split(message, ",");
    if (list.length >= 8 && list[0].equals("Dir:")) {
      roll[0] = float(list[2]);
      pitch[0] = float(list[3]);
      yaw[0] = float(list[4]);
      roll[1] = float(list[5]);
      pitch[1] = float(list[6]);
      yaw[1] = float(list[7]);
    }
  }
}

void applyRotationMatrix(int i) {
  float c1 = cos(radians(roll[i]));
  float s1 = sin(radians(roll[i]));
  float c2 = cos(radians(pitch[i]));
  float s2 = sin(radians(pitch[i]));
  float c3 = cos(radians(yaw[i]));
  float s3 = sin(radians(yaw[i]));

  applyMatrix(
    c2 * c3,                  s1 * s3 + c1 * c3 * s2,   c3 * s1 * s2 - c1 * s3, 0,
    -s2,                      c1 * c2,                  c2 * s1,               0,
    c2 * s3,                  c1 * s2 * s3 - c3 * s1,   c1 * c3 + s1 * s2 * s3, 0,
    0,                        0,                        0,                     1
  );
}

void drawObject(int i) {
  float arrowLength = 100;
  float tipSize = (i == 0) ? 10 : 15;
  float strokeWeightValue = (i == 0) ? 2 : 4;

  strokeWeight(strokeWeightValue);

  color[] colors = {
    (i == 0) ? color(255, 0, 0) : color(255, 100, 100), // X=red
    (i == 0) ? color(0, 255, 0) : color(100, 255, 100), // Y=green
    (i == 0) ? color(0, 0, 255) : color(100, 100, 255)  // Z=blue
  };

  // X軸
  stroke(colors[0]);
  fill(colors[0]);
  line(0, 0, 0, arrowLength, 0, 0);
  pushMatrix();
  translate(arrowLength, 0, 0);
  rotateZ(HALF_PI);
  drawConeShape(tipSize);
  popMatrix();

  // Y軸
  stroke(colors[1]);
  fill(colors[1]);
  line(0, 0, 0, 0, arrowLength, 0);
  pushMatrix();
  translate(0, arrowLength, 0);
  rotateX(-HALF_PI);
  drawConeShape(tipSize);
  popMatrix();

  // Z軸
  stroke(colors[2]);
  fill(colors[2]);
  line(0, 0, 0, 0, 0, arrowLength);
  pushMatrix();
  translate(0, 0, arrowLength);
  drawConeShape(tipSize);
  popMatrix();

  strokeWeight(1); // リセット
}

void drawConeShape(float size) {
  beginShape(TRIANGLES);
  float r = size * 0.5;
  for (int i = 0; i < 3; i++) {
    float angle1 = TWO_PI * i / 3;
    float angle2 = TWO_PI * (i + 1) / 3;
    float x1 = r * cos(angle1);
    float y1 = r * sin(angle1);
    float x2 = r * cos(angle2);
    float y2 = r * sin(angle2);
    vertex(0, 0, -size);
    vertex(x1, y1, 0);
    vertex(x2, y2, 0);
  }
  endShape();
}

void drawLabel(String label, float x, float y, float z) {
  pushMatrix();
  translate(x, y, z);
  fill(0);
  text(label, 0, 0);
  popMatrix();
}
