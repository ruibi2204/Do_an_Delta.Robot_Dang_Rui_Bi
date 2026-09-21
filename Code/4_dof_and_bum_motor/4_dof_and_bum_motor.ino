#include <AccelStepper.h>

// ===================== CẤU HÌNH CHÂN =====================
// Máy bơm
#define PUMP_IN1 PA1

// Bàn xoay - Step 17, driver kiểu PUL/DIR
// GIỮ NGUYÊN PB0/PB1 - KHÔNG dùng lại PA2/PA3 vì trùng chức năng
// USART2 (TX/RX), có thể bị cấu hình lại bất cứ lúc nào -> bàn xoay
// chạy sai/giật. PB0/PB1 là 2 chân GPIO thường, không thuộc ngoại vi
// UART/I2C/SPI mặc định nào trên STM32F103.
#define TURN_PUL_PIN PA6  // PUL+
#define TURN_DIR_PIN PA7   // DIR+

// Step motor xoay theo độ (điều khiển vị trí tương đối)
#define STEP_PIN PA4
#define DIR_PIN  PA5

// ===================== CẤU HÌNH STEP: TRỤC "GÓC" =====================
#define STEPS_PER_REV 6400     // 1/32 vi bước: 200*32
#define STEP_MAX_SPEED 5000.0f
#define STEP_ACCEL     2000.0f

// ===================== CẤU HÌNH STEP: BÀN XOAY =====================
#define TURN_STEPS_PER_REV -800.0f   // 1/4 vi bước: 200*4
#define TURN_MAX_RPM 50.0f
#define TURN_MAX_SPEED (TURN_MAX_RPM * TURN_STEPS_PER_REV / 60.0f)

// Chu kỳ ngắt sinh xung cho BÀN XOAY: 200us (5kHz) - dư biên nhiều so
// với tốc độ bước tối đa cần (50rpm*800/60 ~ 667 bước/s).
#define TURN_ISR_PERIOD_US 200

// Chu kỳ ngắt sinh xung cho TRỤC GÓC: 50us (20kHz) - đủ nhanh để
// AccelStepper.run() theo kịp STEP_MAX_SPEED=10000 bước/s có gia tốc,
// đồng thời tách hẳn khỏi loop() để không bị trễ khi Serial1 nhận dồn
// dập nhiều lệnh PUMP/TURN/STEP liên tiếp trong lúc gắp-thả.
#define STEP_ISR_PERIOD_US 50

AccelStepper stepMotor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper turnMotor(AccelStepper::DRIVER, TURN_PUL_PIN, TURN_DIR_PIN);

// 2 timer phần cứng riêng biệt (API core "maple": HardwareTimer(<số thứ
// tự timer>)). Timer 2 cho bàn xoay, Timer 3 cho trục góc - không dùng
// chung 1 timer để việc tính toán gia tốc của trục góc (nặng hơn) không
// làm trễ nhịp xung của bàn xoay.
HardwareTimer turnTimer(2);
HardwareTimer stepTimer(3);

// ===================== BIẾN TRẠNG THÁI =====================
bool pumpOn = false;
volatile float turnRpm = 0.0f;

// ===================== NGẮT: SINH XUNG CHO BÀN XOAY =====================
void onTurnTimer() {
  turnMotor.runSpeed();
}

// ===================== NGẮT: SINH XUNG CHO TRỤC GÓC =====================
void onStepTimer() {
  stepMotor.run();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(PUMP_IN1, OUTPUT);
  digitalWrite(PUMP_IN1, LOW);

  stepMotor.setMaxSpeed(STEP_MAX_SPEED);
  stepMotor.setAcceleration(STEP_ACCEL);

  turnMotor.setMaxSpeed(TURN_MAX_SPEED);
  turnMotor.setSpeed(0);

  // ---- Timer bàn xoay ----
  turnTimer.pause();
  turnTimer.setPeriod(TURN_ISR_PERIOD_US);
  turnTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  turnTimer.setCompare(TIMER_CH1, 1);
  turnTimer.attachInterrupt(TIMER_CH1, onTurnTimer);
  turnTimer.refresh();

  // ---- Timer trục góc ----
  stepTimer.pause();
  stepTimer.setPeriod(STEP_ISR_PERIOD_US);
  stepTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  stepTimer.setCompare(TIMER_CH1, 1);
  stepTimer.attachInterrupt(TIMER_CH1, onStepTimer);
  stepTimer.refresh();

  // ---- Nâng độ ưu tiên ngắt (NVIC) cho TIMER2 (bàn xoay) LÊN CAO NHẤT,
  // để nó luôn thắng ngắt UART (USART1 RX của Serial1) hay bất kỳ ngắt
  // nào khác nếu chúng vô tình trùng thời điểm. TIMER3 (trục góc) đặt
  // ưu tiên thấp hơn 1 bậc vì mượt tuyệt đối không quan trọng bằng
  // bàn xoay (trục góc có gia tốc, trễ vài chục us không đáng kể).
  // 0 = ưu tiên cao nhất trên STM32F103 (4 bit priority: 0-15).
  nvic_irq_set_priority(NVIC_TIMER2, 0);
  nvic_irq_set_priority(NVIC_TIMER3, 1);

  turnTimer.resume();
  stepTimer.resume();

  Serial.println("=== READY - Nhan lenh UART tu PyCharm ===");
  Serial1.println("READY");
}

// ===================== LOOP =====================
// Giờ loop() CHỈ còn việc đọc/parse UART - cả 2 động cơ bước đều chạy
// hoàn toàn độc lập trong ngắt phần cứng, không còn bị ảnh hưởng bởi
// tốc độ xử lý của loop() nữa.
void loop() {
  xuly_Uart();
}

// ===================== XỬ LÝ LỆNH UART =====================
// Định dạng lệnh (kết thúc bằng '\n'):
//   PUMP:1        -> bật bơm
//   PUMP:0        -> tắt bơm
//   TURN:12.5     -> đặt tốc độ bàn xoay = 12.5 vòng/phút
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
      }

      // ---- Lệnh bàn xoay (RPM) ----
      p = strstr(receivedChars, "TURN:");
      if (p) {
        float rpm = atof(p + 5);
        rpm = constrain(rpm, -TURN_MAX_RPM, TURN_MAX_RPM);
        turnRpm = rpm;

        float stepsPerSec = rpm * TURN_STEPS_PER_REV / 60.0f;

        noInterrupts();
        turnMotor.setSpeed(stepsPerSec);
        interrupts();
      }

      // ---- Lệnh step (quay theo độ) ----
      p = strstr(receivedChars, "STEP:");
      if (p) {
        float deg = atof(p + 5);
        long steps = (long)(deg * STEPS_PER_REV / 360.0f);

        noInterrupts();
        stepMotor.move(steps);
        interrupts();
      }
    }
  }
}