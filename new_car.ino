#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

// ================= SETTINGS =================
char ssid[] = "varun";
char pass[] = "12345678002";

// MOTOR PINS (L298N or similar)
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

// SENSORS
#define TRIG_FRONT 23
#define ECHO_FRONT 34
#define TRIG_LEFT  25
#define ECHO_LEFT  26
#define TRIG_RIGHT 4
#define ECHO_RIGHT 2
#define SERVO_PIN 5

// ENCODERS
#define ENC_LEFT  35
#define ENC_RIGHT 32

// ================= GLOBALS =================
volatile long left_pulses = 0;
volatile long right_pulses = 0;
unsigned long last_control_time = 0;

Servo scanServo;

// PID / SPEED CONSTANTS
int target_pps = 30; // Pulses Per Second target
float Kp_speed = 5.0;
int left_pwm = 110;
int right_pwm = 110;

const int PWM_MIN = 80;
const int PWM_MAX = 220;

// THRESHOLDS (Adjusted to prevent logic overlap)
const int PANIC_FRONT = 15;
const int SAFE_FRONT = 40;
const int CORNER_ALERT = 20; 
const int WALL_TARGET = 22;
const int WALL_DETECT = 35;
float Kp_wall = 4.0;

// ================= ISR =================
void IRAM_ATTR leftEncoderISR() { left_pulses++; }
void IRAM_ATTR rightEncoderISR() { right_pulses++; }

// ================= SENSOR READ =================
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 25000); // 25ms timeout
  if (duration == 0) return 999; 
  return duration * 0.0343 / 2.0;
}

// ================= MOTOR CONTROL =================
void drive(int lSpeed, int rSpeed) {
  // Left Motors
  if (lSpeed > 0) {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
    digitalWrite(L_IN3, HIGH); digitalWrite(L_IN4, LOW);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, HIGH);
  }
  analogWrite(L_ENA, abs(lSpeed)); analogWrite(L_ENB, abs(lSpeed));

  // Right Motors
  if (rSpeed > 0) {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH);
    digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
  } else {
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
    digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, HIGH);
  }
  analogWrite(R_ENA, abs(rSpeed)); analogWrite(R_ENB, abs(rSpeed));
}

// ================= BEHAVIORS =================
void panicEscape() {
  drive(0, 0); delay(200);
  drive(-150, -150); delay(400);
  drive(0, 0);
  
  scanServo.write(160); delay(400);
  float dL = getDistance(TRIG_FRONT, ECHO_FRONT);
  scanServo.write(20); delay(400);
  float dR = getDistance(TRIG_FRONT, ECHO_FRONT);
  scanServo.write(90);
  
  if (dL > dR) drive(-180, 180);
  else drive(180, -180);
  delay(500);
}

void wallFollow(float dist) {
  float error = WALL_TARGET - dist;
  int adjust = (int)(error * Kp_wall);
  drive(constrain(left_pwm + adjust, PWM_MIN, PWM_MAX), 
        constrain(right_pwm - adjust, PWM_MIN, PWM_MAX));
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  
  int pins[] = {L_IN1, L_IN2, L_ENA, L_IN3, L_IN4, L_ENB, R_IN1, R_IN2, R_ENA, R_IN3, R_IN4, R_ENB, TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT};
  for(int p : pins) pinMode(p, OUTPUT);
  pinMode(ECHO_FRONT, INPUT); pinMode(ECHO_LEFT, INPUT); pinMode(ECHO_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), rightEncoderISR, RISING);

  WiFi.begin(ssid, pass);
  ArduinoOTA.begin();
  
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);
}

// ================= MAIN LOOP =================
void loop() {
  ArduinoOTA.handle();

  float f = getDistance(TRIG_FRONT, ECHO_FRONT);
  float l = getDistance(TRIG_LEFT, ECHO_LEFT);
  float r = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  // 1. CRITICAL: PANIC MODE
  if (f < PANIC_FRONT) {
    panicEscape();
    return;
  }

  // 2. SPEED PID (Calculated every 100ms)
  if (millis() - last_control_time >= 100) {
    float lp = left_pulses * 10.0; // Convert to pulses per second
    float rp = right_pulses * 10.0;
    left_pulses = 0; right_pulses = 0;

    left_pwm  += (target_pps - lp) * Kp_speed;
    right_pwm += (target_pps - rp) * Kp_speed;
    
    left_pwm = constrain(left_pwm, PWM_MIN, PWM_MAX);
    right_pwm = constrain(right_pwm, PWM_MIN, PWM_MAX);
    last_control_time = millis();
  }

  // 3. NAVIGATION HIERARCHY
  if (l < CORNER_ALERT) {
    drive(left_pwm + 30, right_pwm - 50); // Sharp turn right
  } 
  else if (r < CORNER_ALERT) {
    drive(left_pwm - 50, right_pwm + 30); // Sharp turn left
  }
  else if (r < WALL_DETECT && f > SAFE_FRONT) {
    wallFollow(r); // Smooth wall following
  }
  else if (f < SAFE_FRONT) {
    if (l > r) drive(-130, 150); // Turn toward open space
    else drive(150, -130);
  } 
  else {
    drive(left_pwm, right_pwm); // Straight ahead
  }

  delay(30);
}