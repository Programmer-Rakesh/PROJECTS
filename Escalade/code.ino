#include <BluetoothSerial.h>
BluetoothSerial BT;

int IN1 = 26;
int IN2 = 25;
int IN3 = 33;
int IN4 = 32;

void setup() {
  Serial.begin(115200);
  BT.begin("ESP32-CAR");  // Bluetooth name

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  if (BT.available()) {
    char cmd = BT.read();
    Serial.println(cmd);

    if (cmd == 'F') forward();
    else if (cmd == 'B') backward();
    else if (cmd == 'L') left();
    else if (cmd == 'R') right();
    else if (cmd == 'S') stopCar();
  }
}
