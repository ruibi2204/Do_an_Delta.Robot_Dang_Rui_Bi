#include <AccelStepper.h>
#define STEP1 PA0
#define DIR1 PA1
#define STEP2 PA2
#define DIR2 PA3
#define STEP3 PA4
#define DIR3 PA5
#define SW1 PA6
#define SW2 PB0
#define SW3 PA7
#define HOME_BTN PB1
#define HOMING_SPEED_FAST 500
#define HOMING_SPEED_SLOW 200
#define HOMING_ACCEL 50
#define RUN_SPEED 10000
#define RUN_ACCEL 5000
#define BACKOFF_STEPS 800

const float GOC_TO_STEP = 6400.0 / 360.0;
float target_theta1 = 0.0;
float target_theta2 = 0.0;
float target_theta3 = 0.0;
bool homeDone = false;

#define DEBOUNCE_BTN_MS 50
#define DEBOUNCE_SW_MS 10

static uint32_t sw1_last = 0, sw2_last = 0, sw3_last = 0;
static bool sw1_state = HIGH, sw2_state = HIGH, sw3_state = HIGH;
static uint32_t btn_last = 0;
static bool btn_state = HIGH;

AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper motor3(AccelStepper::DRIVER, STEP3, DIR3);

void setMaxSpeed3(float s);
void setAccel3(float a);
void setRunConfig();
void doHoming();
void homingPhase(float speed, uint32_t drainMs);
void moveAllTo(long pos);
bool readSwitch(uint8_t pin, bool &state, uint32_t &lastTime, uint32_t dMs);
bool readButton();
void xuly_Uart();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("=== DELTA ROBOT - 2-PHASE HOMING ===");
  Serial1.begin(115200);
  pinMode(HOME_BTN, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  setMaxSpeed3(HOMING_SPEED_FAST);
  setAccel3(HOMING_ACCEL);
  Serial.println("Nhấn PB1 để bắt đầu homing...");
  while (digitalRead(HOME_BTN) == HIGH) {}
  while (digitalRead(HOME_BTN) == LOW) {}
  doHoming();
}
void loop() {
  if (readButton()) {
    Serial.println("[RE-HOME] PB1 → Re-homing...");
    homeDone = false;
    doHoming();
    return;
  }
  if (homeDone) {
    xuly_Uart();
  }
  motor1.run();
  motor2.run();
  motor3.run();
}
void doHoming() {
  Serial.println("[PHA 1] Tìm home nhanh...");
  setMaxSpeed3(HOMING_SPEED_FAST);
  homingPhase(-HOMING_SPEED_FAST, 50);
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  motor3.setCurrentPosition(0);
  Serial.println("[PHA 1] Lùi backoff cố định 500 bước...");
  moveAllTo(BACKOFF_STEPS);
  Serial.println("[PHA 2] Tìm home chậm (chính xác)...");
  setMaxSpeed3(HOMING_SPEED_SLOW);
  homingPhase(-HOMING_SPEED_SLOW, 50);
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  motor3.setCurrentPosition(0);
  Serial.println("[HOMING] Đã xác định HOME = 0 chuẩn xác tuyệt đối.");
  // ==== Hạ robot xuống 100 xung sau khi home xong ====
  // =====================================================
  setRunConfig();
  Serial1.println("READY");
  homeDone = true;
}
void homingPhase(float speed, uint32_t drainMs) {
  bool homed1 = false, homed2 = false, homed3 = false;
  uint32_t drain1 = 0, drain2 = 0, drain3 = 0;
  if (readSwitch(SW1, sw1_state, sw1_last, DEBOUNCE_SW_MS)) {
    motor1.move(BACKOFF_STEPS / 2);
    while (motor1.distanceToGo() != 0) motor1.run();
  }
  if (readSwitch(SW2, sw2_state, sw2_last, DEBOUNCE_SW_MS)) {
    motor2.move(BACKOFF_STEPS / 2);
    while (motor2.distanceToGo() != 0) motor2.run();
  }
  if (readSwitch(SW3, sw3_state, sw3_last, DEBOUNCE_SW_MS)) {
    motor3.move(BACKOFF_STEPS / 2);
    while (motor3.distanceToGo() != 0) motor3.run();
  }
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  while (!homed1 || !homed2 || !homed3) {
    uint32_t now = millis();
    if (!homed1) {
      if (drain1 > 0) {
        if (now - drain1 >= drainMs) {
          motor1.setSpeed(0);
          homed1 = true;
          Serial.println("  Motor 1 OK (PA6)");
        } else {
          motor1.runSpeed();
        }
      } else if (readSwitch(SW1, sw1_state, sw1_last, DEBOUNCE_SW_MS)) {
        drain1 = millis();
      } else {
        motor1.runSpeed();
      }
    }
    if (!homed2) {
      if (drain2 > 0) {
        if (now - drain2 >= drainMs) {
          motor2.setSpeed(0);
          homed2 = true;
          Serial.println("  Motor 2 OK (PB0)");
        } else {
          motor2.runSpeed();
        }
      } else if (readSwitch(SW2, sw2_state, sw2_last, DEBOUNCE_SW_MS)) {
        drain2 = millis();
      } else {
        motor2.runSpeed();
      }
    }
    if (!homed3) {
      if (drain3 > 0) {
        if (now - drain3 >= drainMs) {
          motor3.setSpeed(0);
          homed3 = true;
          Serial.println("  Motor 3 OK (PA7)");
        } else {
          motor3.runSpeed();
        }
      } else if (readSwitch(SW3, sw3_state, sw3_last, DEBOUNCE_SW_MS)) {
        drain3 = millis();
      } else {
        motor3.runSpeed();
      }
    }
  }
}
bool readSwitch(uint8_t pin, bool &state, uint32_t &lastTime, uint32_t dMs) {
  bool raw = (digitalRead(pin) == LOW);
  if (raw != state) {
    state = raw;
    lastTime = millis();
  }
  if ((millis() - lastTime) >= dMs) {
    return state;
  }
  return false;
}
bool readButton() {
  static bool last_stable_state = HIGH;
  bool raw = digitalRead(HOME_BTN);

  if (raw != btn_state) {
    btn_state = raw;
    btn_last = millis();
  }

  if ((millis() - btn_last) >= DEBOUNCE_BTN_MS) {
    if (last_stable_state == HIGH && btn_state == LOW) {
      last_stable_state = btn_state;
      return true;
    }
    last_stable_state = btn_state;
  }
  return false;
}
void xuly_Uart() {
  const byte numChars = 64;
  static char receivedChars[numChars];
  static byte ndx = 0;
  while (Serial1.available() > 0) {
    char rc = Serial1.read();
    if (rc != '\n') {
      if (ndx < numChars - 1) {
        receivedChars[ndx++] = rc;
      }
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      if (strcmp(receivedChars, "HOME") == 0) {
        Serial.println("[UART] Nhận lệnh HOME -> Tiến hành Re-homing...");
        homeDone = false;
        doHoming();
        return;
      }
      char *p = strstr(receivedChars, "T1:");
      if (p) target_theta1 = atof(p + 3);
      p = strstr(receivedChars, "T2:");
      if (p) target_theta2 = atof(p + 3);
      p = strstr(receivedChars, "T3:");
      if (p) target_theta3 = atof(p + 3);
      motor1.moveTo((long)(target_theta2 * GOC_TO_STEP));
      motor2.moveTo((long)(target_theta1 * GOC_TO_STEP));
      motor3.moveTo((long)(target_theta3 * GOC_TO_STEP));
      Serial.print("[UART-RX] M1:");
      Serial.print(target_theta1);
      Serial.print(" | M2:");
      Serial.print(target_theta2);
      Serial.print(" | M3:");
      Serial.println(target_theta3);
    }
  }
}
void moveAllTo(long pos) {
  motor1.moveTo(pos);
  motor2.moveTo(pos);
  motor3.moveTo(pos);
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
  }
}
void setMaxSpeed3(float s) {
  motor1.setMaxSpeed(s);
  motor2.setMaxSpeed(s);
  motor3.setMaxSpeed(s);
}
void setAccel3(float a) {
  motor1.setAcceleration(a);
  motor2.setAcceleration(a);
  motor3.setAcceleration(a);
}
void setRunConfig() {
  setMaxSpeed3(RUN_SPEED);
  setAccel3(RUN_ACCEL);
}