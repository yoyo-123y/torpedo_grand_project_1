#include <WiFi.h>
WiFiServer server(3333);  // TCP server on port 3333
WiFiClient client;

const char* ssid = "myLaptop";
const char* password = "12345678";

// Motor pins
const int LEFT_PWM = 5;
const int RIGHT_PWM = 6;

// IR sensor pins
const int IR_LEFT = A0;
const int IR_RIGHT = A1;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  server.begin();

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    return;
  }

  if (client.available()) {
    String cmd = client.readStringUntil('\n');
    if (cmd.startsWith("VEL:")) {
      int sep = cmd.indexOf(',');
      float lin = cmd.substring(4, sep).toFloat();
      float ang = cmd.substring(sep + 1).toFloat();

      float left = lin - ang;
      float right = lin + ang;

      int l_pwm = constrain(int(left * 255), 0, 255);
      int r_pwm = constrain(int(right * 255), 0, 255);

      analogWrite(LEFT_PWM, l_pwm);
      analogWrite(RIGHT_PWM, r_pwm);
    }
  }

  // Send IR sensor data
  int ir_left = analogRead(IR_LEFT);
  int ir_right = analogRead(IR_RIGHT);
  client.printf("IR:%d,%d\n", ir_left, ir_right);

  delay(50);
}
