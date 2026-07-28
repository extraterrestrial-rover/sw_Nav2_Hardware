/**
 * rover_firmware.ino
 *
 * ESP32 firmware for a 6-wheel skid-steer rover.
 * Receives velocity commands from a laptop (ROS2 nav2_hardware package)
 * over USB Serial at 115200 baud.
 *
 * Serial Protocol:
 *   PC  → ESP32  :  "CMD:<left_pwm>,<right_pwm>\n"
 *                   left_pwm / right_pwm : -255 to +255
 *                   positive = forward, negative = backward
 *
 *   ESP32 → PC   :  "OK\n"   (ACK after each command)
 *
 * Motor Layout (6 motors, 3 per side):
 *   Motor Driver 1 → Left Front  + Left Back
 *   Motor Driver 2 → Left Middle + Right Middle
 *   Motor Driver 3 → Right Front + Right Back
 *h
 * Each motor channel: one DIR pin (digital) + one PWM pin (analogWrite)
 *   DIR = LOW  → forward
 *   DIR = HIGH → backward
 *   PWM = 0-255 → speed
 *
 * IMPORTANT: Adjust the pin definitions below to matc your PCB!
 * ──────────────────────────────────────────────────────────────
 * The pin assignments below are suggestions — verify against your schematic.
 */

#include <Arduino.h>

// ── Watchdog: stop motors if no command received for this long (ms) ──────────
#define WATCHDOG_MS  500

// ── Middle wheel reduction during mixed motion (0.0 - 1.0) ──────────────────
// The middle wheel scrubs more on a 6-wheel rover. Reduce its power to ease
// turning. Set to 1.0 to disable.
#define MIDDLE_RATIO  0.857f  // 6/7 ratio, matches original tested firmware

// ════════════════════════════════════════════════════════════════════════════
//  GPIO PIN ASSIGNMENTS  —  ADJUST TO MATCH YOUR PCB
// ════════════════════════════════════════════════════════════════════════════
// Each entry: {DIR_pin, PWM_pin}

// Motor Driver 1  (verified hardware pins)
const uint8_t LEFT_FRONT[]  = {19, 21};   // DIR=GPIO19, PWM=GPIO21
const uint8_t LEFT_BACK[]   = {22, 23};   // DIR=GPIO22, PWM=GPIO23

// Motor Driver 2  (verified hardware pins)
const uint8_t LEFT_MID[]    = {33, 32};   // DIR=GPIO33, PWM=GPIO32
const uint8_t RIGHT_MID[]   = {26, 25};   // DIR=GPIO26, PWM=GPIO25

// Motor Driver 3  (verified hardware pins)
const uint8_t RIGHT_FRONT[] = {13, 12};   // DIR=GPIO13, PWM=GPIO12
const uint8_t RIGHT_BACK[]  = {14, 27};   // DIR=GPIO14, PWM=GPIO27
// ════════════════════════════════════════════════════════════════════════════

// ── ESP32 LEDC PWM channels (0-15, each channel used by one PWM pin) ────────
#define CH_LF   0
#define CH_LB   1
#define CH_LM   2
#define CH_RM   3
#define CH_RF   4
#define CH_RB   5

#define PWM_FREQ  1000   // Hz
#define PWM_RES   8      // bits (0-255)

// ── Globals ───────────────────────────────────────────────────────────────────
unsigned long last_cmd_ms = 0;
String serial_buf = "";

// ── Setup PWM channel and DIR pin for one motor ───────────────────────────────
void setupMotor(const uint8_t motor[], uint8_t channel) {
  pinMode(motor[0], OUTPUT);                          // DIR
  ledcSetup(channel, PWM_FREQ, PWM_RES);             // configure LEDC
  ledcAttachPin(motor[1], channel);                  // attach PWM pin
  ledcWrite(channel, 0);                             // start at 0
  digitalWrite(motor[0], LOW);
}

// ── Drive one motor ───────────────────────────────────────────────────────────
//   speed: -255 to +255
//   positive → forward (DIR=LOW), negative → backward (DIR=HIGH)
void driveMotor(const uint8_t motor[], uint8_t channel, int speed, float ratio = 1.0f) {
  int s = constrain(speed, -255, 255);
  int scaled = (int)(abs(s) * ratio);
  scaled = constrain(scaled, 0, 255);

  if (s >= 0) {
    digitalWrite(motor[0], LOW);   // forward
  } else {
    digitalWrite(motor[0], HIGH);  // backward
  }
  ledcWrite(channel, scaled);
}

// ── Apply left/right commands to all 6 motors ────────────────────────────────
void applyWheels(int left_pwm, int right_pwm) {
  // Determine if this is a pure turn or mixed (for middle ratio)
  // Simple approach: always apply MIDDLE_RATIO to middle wheels
  driveMotor(LEFT_FRONT,  CH_LF, left_pwm,  1.0f);
  driveMotor(LEFT_BACK,   CH_LB, left_pwm,  1.0f);
  driveMotor(LEFT_MID,    CH_LM, left_pwm,  MIDDLE_RATIO);

  driveMotor(RIGHT_FRONT, CH_RF, right_pwm, 1.0f);
  driveMotor(RIGHT_BACK,  CH_RB, right_pwm, 1.0f);
  driveMotor(RIGHT_MID,   CH_RM, right_pwm, MIDDLE_RATIO);
}

// ── Stop all motors ───────────────────────────────────────────────────────────
void stopAll() {
  applyWheels(0, 0);
}

// ── Parse and execute a command line ─────────────────────────────────────────
//   Expected format: "CMD:<left>,<right>"
void processLine(const String & line) {
  if (!line.startsWith("CMD:")) {
    return;  // ignore unknown lines
  }

  // Parse "CMD:<left>,<right>"
  int comma = line.indexOf(',', 4);
  if (comma < 0) {
    Serial.println("ERR:bad_format");
    return;
  }

  int left_pwm  = line.substring(4, comma).toInt();
  int right_pwm = line.substring(comma + 1).toInt();

  applyWheels(left_pwm, right_pwm);
  last_cmd_ms = millis();

  Serial.println("OK");
}

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // wait for USB serial

  // Setup all motor pins
  setupMotor(LEFT_FRONT,  CH_LF);
  setupMotor(LEFT_BACK,   CH_LB);
  setupMotor(LEFT_MID,    CH_LM);
  setupMotor(RIGHT_MID,   CH_RM);
  setupMotor(RIGHT_FRONT, CH_RF);
  setupMotor(RIGHT_BACK,  CH_RB);

  stopAll();
  last_cmd_ms = millis();

  Serial.println("ROVER_READY");
}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  // ── Read serial line ───────────────────────────────────────────────────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serial_buf.trim();
      if (serial_buf.length() > 0) {
        processLine(serial_buf);
      }
      serial_buf = "";
    } else {
      serial_buf += c;
    }
  }

  // ── Watchdog: stop if no command for WATCHDOG_MS ──────────────────────────
  if (millis() - last_cmd_ms > WATCHDOG_MS) {
    stopAll();
    last_cmd_ms = millis();  // reset so we don't spam stop
  }
}
