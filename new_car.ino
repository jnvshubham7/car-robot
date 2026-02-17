#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

char ssid[] = "varun";
char pass[] = "12345678002";

// ================= MOTOR PINS =================
#define L_IN1 13
#define L_IN2 12
#define L_ENA 14
#define L_IN3 27
#define L_IN4 33
#define L_ENB 15

#define R_IN1 16
#define R_IN2 17
#define R_ENA 18
#define R_IN3 19
#define R_IN4 21
#define R_ENB 22

// ================= ULTRASONIC =================
#define TRIG_FRONT 23
#define ECHO_FRONT 34
#define TRIG_LEFT  25
#define ECHO_LEFT  26
#define TRIG_RIGHT 4
#define ECHO_RIGHT 2

// ================= ENCODERS =================
#define ENC_LEFT  35
#define ENC_RIGHT 32

#define SERVO_PIN 5

// ================= VARIABLES =================
volatile long left_pulses = 0;
volatile long right_pulses = 0;

unsigned long last_control_time = 0;

Servo scanServo;

// ================= SPEED =================
int target_pps = 28;
float Kp_speed = 8.0;

int base_pwm = 95;
int left_pwm = base_pwm;
int right_pwm = base_pwm + 18;

const int PWM_MIN = 85;
const int PWM_MAX = 210;

// ================= DISTANCE =================
const int SAFE_FRONT = 45;
const int DANGER_FRONT = 20;
const int PANIC_FRONT = 15;
const int SIDE_SAFE = 25;
const int CORNER_ALERT = 30;

// ================= WALL FOLLOW =================
const int WALL_TARGET = 25;
const int WALL_RANGE  = 4;
const int WALL_DETECT = 40;
float Kp_wall = 3.0;

// ================= ISR =================
void IRAM_ATTR leftEncoderISR() { left_pulses++; }
void IRAM_ATTR rightEncoderISR() { right_pulses++; }

// ================= DISTANCE =================
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 999;
  return duration * 0.0343 / 2.0;
}

// ================= MOTOR =================
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
    digitalWrite(L_IN3, HIGH); digitalWrite(L_IN4, LOW);
    analogWrite(L_ENA, speed); analogWrite(L_ENB, speed);
  } else if (speed < 0) {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, HIGH);
    analogWrite(L_ENA, -speed); analogWrite(L_ENB, -speed);
  } else {
    analogWrite(L_ENA, 0); analogWrite(L_ENB, 0);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH);
    digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
    analogWrite(R_ENA, speed); analogWrite(R_ENB, speed);
  } else if (speed < 0) {
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
    digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, HIGH);
    analogWrite(R_ENA, -speed); analogWrite(R_ENB, -speed);
  } else {
    analogWrite(R_ENA, 0); analogWrite(R_ENB, 0);
  }
}

// ================= PANIC ESCAPE =================
void panicEscape() {

  Serial.println("!!! PANIC !!!");

  setLeftMotor(0);
  setRightMotor(0);
  delay(80);

  setLeftMotor(-150);
  setRightMotor(-150);
  delay(350);

  setLeftMotor(0);
  setRightMotor(0);
  delay(120);

  // scan
  scanServo.write(180);
  delay(300);
  float left_scan = getDistance(TRIG_FRONT, ECHO_FRONT);

  scanServo.write(0);
  delay(300);
  float right_scan = getDistance(TRIG_FRONT, ECHO_FRONT);

  scanServo.write(90);
  delay(120);

  if (left_scan > right_scan) {
    setLeftMotor(-170);
    setRightMotor(170);
  } else {
    setLeftMotor(170);
    setRightMotor(-170);
  }

  delay(450);

  setLeftMotor(0);
  setRightMotor(0);
  delay(80);
}

// ================= WALL FOLLOW =================
void wallFollowRight(float dist_right) {
  int error = WALL_TARGET - dist_right;

  if (abs(error) <= WALL_RANGE) {
    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);
    return;
  }

  int correction = (int)(Kp_wall * error);

  int l = constrain(left_pwm + correction, PWM_MIN, PWM_MAX);
  int r = constrain(right_pwm - correction, PWM_MIN, PWM_MAX);

  setLeftMotor(l);
  setRightMotor(r);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_ENA, OUTPUT);
  pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT); pinMode(L_ENB, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_ENA, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT); pinMode(R_ENB, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), rightEncoderISR, RISING);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(400);
  ArduinoOTA.begin();

  scanServo.attach(SERVO_PIN);
  scanServo.write(90);
}

// ================= LOOP =================
void loop() {
  ArduinoOTA.handle();

  float f = getDistance(TRIG_FRONT, ECHO_FRONT);
  float l = getDistance(TRIG_LEFT, ECHO_LEFT);
  float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  // 🚨 PANIC
  if (f < PANIC_FRONT) {
    panicEscape();
    return;
  }

  // ================= SPEED PID =================
  if (millis() - last_control_time >= 150) {
    long lp = left_pulses;
    long rp = right_pulses;
    left_pulses = right_pulses = 0;

    float left_pps  = lp * (1000.0 / 150.0);
    float right_pps = rp * (1000.0 / 150.0);

    left_pwm  += (int)(Kp_speed * (target_pps - left_pps));
    right_pwm += (int)(Kp_speed * (target_pps - right_pps));
    right_pwm += 18;

    left_pwm  = constrain(left_pwm, PWM_MIN, PWM_MAX);
    right_pwm = constrain(right_pwm, PWM_MIN, PWM_MAX);

    last_control_time = millis();
  }

  // 🧱 CORNER PROTECTION
  if (l < CORNER_ALERT) {
    setLeftMotor(left_pwm + 25);
    setRightMotor(right_pwm - 60);
    return;
  }

  if (r < CORNER_ALERT) {
    setLeftMotor(left_pwm - 60);
    setRightMotor(right_pwm + 25);
    return;
  }

  // 🧭 WALL FOLLOW
  if (r < WALL_DETECT && f > SAFE_FRONT) {
    wallFollowRight(r);
    return;
  }

  // 🙂 NORMAL SMOOTH AVOID
  if (f < SAFE_FRONT) {
    if (l > r) {
      setLeftMotor(-120);
      setRightMotor(140);
    } else {
      setLeftMotor(140);
      setRightMotor(-120);
    }
  } else {
    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);
  }

  delay(20);
}
