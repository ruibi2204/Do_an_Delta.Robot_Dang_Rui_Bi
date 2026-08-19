#include <AccelStepper.h>

// ===================== CẤU HÌNH CHÂN =====================
// Máy bơm (chạy full tốc, chỉ bật/tắt qua IN1, ENA giả định đã bypass = luôn 5V)
#define PUMP_IN1 PA1

// Bàn xoay (L298N kênh B) - IN4 giả định nối GND cố định (chỉ chạy 1 chiều)
#define TURN_IN3 PA2
#define TURN_ENB PA3   // PWM tốc độ bàn xoay

// Step motor (bàn xoay theo độ)
#define STEP_PIN PA4
#define DIR_PIN  PA5

// ===================== CẤU HÌNH STEP =====================
// Vi bước 1/32 -> 200 * 32 = 6400 xung/vòng
#define STEPS_PER_REV 6400
#define STEP_MAX_SPEED 3000.0f   // xung/giây
#define STEP_ACCEL     2000.0f   // xung/giây^2

AccelStepper stepMotor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ===================== BIẾN TRẠNG THÁI =====================
bool pumpOn = false;
int turnSpeed = 0; // 0-255

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);           // Debug qua USB
  Serial1.begin(115200);          // UART giao tiếp với PyCharm (USART1: PA9=TX, PA10=RX)

  pinMode(PUMP_IN1, OUTPUT);
  digitalWrite(PUMP_IN1, LOW);

  pinMode(TURN_IN3, OUTPUT);
  pinMode(TURN_ENB, OUTPUT);
  digitalWrite(TURN_IN3, HIGH);   // cho phép chạy (IN4 nối GND cố định)
  analogWrite(TURN_ENB, 0);       // mặc định dừng

  stepMotor.setMaxSpeed(STEP_MAX_SPEED);
  stepMotor.setAcceleration(STEP_ACCEL);

  Serial.println("=== READY - Nhan lenh UART tu PyCharm ===");
  Serial1.println("READY");
}

// ===================== LOOP =====================
void loop() {
  xuly_Uart();
  stepMotor.run(); // luôn phải gọi để step motor di chuyển
}

// ===================== XỬ LÝ LỆNH UART =====================
// Định dạng lệnh (kết thúc bằng '\n'):
//   PUMP:1        -> bật bơm
//   PUMP:0        -> tắt bơm
//   TURN:200      -> đặt tốc độ bàn xoay (L298N) = 200 (0-255, 0 = dừng)
//   STEP:90       -> quay step 90 độ (có thể âm để quay ngược chiều), tương đối
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

      // ---- Lệnh bàn xoay (L298N) ----
      p = strstr(receivedChars, "TURN:");
      if (p) {
        int val = atoi(p + 5);
        val = constrain(val, 0, 255);
        turnSpeed = val;
        analogWrite(TURN_ENB, turnSpeed);
        Serial.print("[TURN] speed=");
        Serial.println(turnSpeed);
      }

      // ---- Lệnh step (quay theo độ) ----
      p = strstr(receivedChars, "STEP:");
      if (p) {
        float deg = atof(p + 5);
        long steps = (long)(deg * STEPS_PER_REV / 360.0f);
        stepMotor.move(steps); // di chuyển tương đối
        Serial.print("[STEP] degree=");
        Serial.print(deg);
        Serial.print(" -> steps=");
        Serial.println(steps);
      }
    }
  }
}
