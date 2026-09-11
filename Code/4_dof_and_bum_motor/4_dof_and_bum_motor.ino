#include <AccelStepper.h>

// ===================== CẤU HÌNH CHÂN =====================
// Máy bơm
#define PUMP_IN1 PA1

// Bàn xoay - Step 17, driver kiểu PUL/DIR
#define TURN_PUL_PIN PA2   // PUL+
#define TURN_DIR_PIN PA3   // DIR+

// Step motor xoay theo độ (điều khiển vị trí tương đối)
#define STEP_PIN PA4
#define DIR_PIN  PA5

// ===================== CẤU HÌNH STEP: TRỤC "GÓC" =====================
#define STEPS_PER_REV 6400     // 1/32 vi bước: 200*32
#define STEP_MAX_SPEED 10000.0f
#define STEP_ACCEL     5000.0f

// ===================== CẤU HÌNH STEP: BÀN XOAY =====================
#define TURN_STEPS_PER_REV -800.0f   // 1/4 vi bước: 200*4
#define TURN_MAX_RPM 60.0f          // giới hạn an toàn, chỉnh theo motor/tải thực tế
// steps/giây tối đa tương ứng RPM tối đa
#define TURN_MAX_SPEED (TURN_MAX_RPM * TURN_STEPS_PER_REV / 60.0f)

AccelStepper stepMotor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper turnMotor(AccelStepper::DRIVER, TURN_PUL_PIN, TURN_DIR_PIN);

// ===================== BIẾN TRẠNG THÁI =====================
bool pumpOn = false;
float turnRpm = 0.0f;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PUMP_IN1, OUTPUT);
  digitalWrite(PUMP_IN1, LOW);

  stepMotor.setMaxSpeed(STEP_MAX_SPEED);
  stepMotor.setAcceleration(STEP_ACCEL);

  // Bàn xoay: chạy tốc độ không đổi -> không dùng move()/acceleration,
  // chỉ cần setMaxSpeed đủ lớn để runSpeed() cho phép setSpeed() tới mức đó.
  turnMotor.setMaxSpeed(TURN_MAX_SPEED);
  turnMotor.setSpeed(0);

  Serial.println("=== READY - Nhan lenh UART tu PyCharm ===");
  Serial1.println("READY");
}

// ===================== LOOP =====================
void loop() {
  xuly_Uart();
  stepMotor.run();       // trục góc: có gia tốc, chạy tới vị trí đích
  turnMotor.runSpeed();  // bàn xoay: chạy tốc độ không đổi liên tục
}

// ===================== XỬ LÝ LỆNH UART =====================
// Định dạng lệnh (kết thúc bằng '\n'):
//   PUMP:1        -> bật bơm
//   PUMP:0        -> tắt bơm
//   TURN:12.5     -> đặt tốc độ bàn xoay = 12.5 vòng/phút (âm = quay ngược, 0 = dừng)
//   STEP:90       -> quay trục góc 90 độ (tương đối, có thể âm)
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

      char *p;

      // ---- Lệnh bơm ----
      p = strstr(receivedChars, "PUMP:");
      if (p) {
        int val = atoi(p + 5);
        pumpOn = (val != 0);
        digitalWrite(PUMP_IN1, pumpOn ? HIGH : LOW);
        Serial.print("[PUMP] ");
        Serial.println(pumpOn ? "ON" : "OFF");
      }

      // ---- Lệnh bàn xoay (RPM) ----
      p = strstr(receivedChars, "TURN:");
      if (p) {
        float rpm = atof(p + 5);
        rpm = constrain(rpm, -TURN_MAX_RPM, TURN_MAX_RPM);
        turnRpm = rpm;

        float stepsPerSec = rpm * TURN_STEPS_PER_REV / 60.0f;
        turnMotor.setSpeed(stepsPerSec);

        Serial.print("[TURN] rpm=");
        Serial.print(turnRpm);
        Serial.print(" -> steps/s=");
        Serial.println(stepsPerSec);
      }

      // ---- Lệnh step (quay theo độ) ----
      p = strstr(receivedChars, "STEP:");
      if (p) {
        float deg = atof(p + 5);
        long steps = (long)(deg * STEPS_PER_REV / 360.0f);
        stepMotor.move(steps);
        Serial.print("[STEP] degree=");
        Serial.print(deg);
        Serial.print(" -> steps=");
        Serial.println(steps);
      }
    }
  }
}