#include <AccelStepper.h>
#include <Wire.h>

// ================== STEPPER ==================
AccelStepper stepA(AccelStepper::DRIVER, PA0, PA1);
AccelStepper stepB(AccelStepper::DRIVER, PA2, PA3);
AccelStepper stepC(AccelStepper::DRIVER, PA4, PA5);

// ================== LIMIT SWITCH (NC) ==================
#define LIMIT_A PA6
#define LIMIT_B PB0
#define LIMIT_C PA7

bool homeDone = false;

// Hệ số chuyển đổi từ ĐỘ sang XUNG (Steps)
// Ví dụ: Động cơ 1.8 độ/bước, driver chỉnh 1/16 vi bước -> 3200 xung = 360 độ.

const float GOC_TO_STEP = (3200.0) / 360.0; // ~33.7778 xung/độ

// Biến lưu góc đích nhận từ Laptop
float target_theta1 = 0.00;
float target_theta2 = 0.00;
float target_theta3 = 0.00;

// ================== HOMING (Code của bạn) ==================
void doHoming()
{
    const float HOMING_SPEED = -2000;   // CHẬM – CHẮC

    while ( digitalRead(LIMIT_A) == HIGH ||
            digitalRead(LIMIT_B) == HIGH ||
            digitalRead(LIMIT_C) == HIGH )
    {
        if (digitalRead(LIMIT_A) == HIGH) stepA.setSpeed(HOMING_SPEED);
        else stepA.setSpeed(0);

        if (digitalRead(LIMIT_B) == HIGH) stepB.setSpeed(HOMING_SPEED);
        else stepB.setSpeed(0);

        if (digitalRead(LIMIT_C) == HIGH) stepC.setSpeed(HOMING_SPEED);
        else stepC.setSpeed(0);

        stepA.runSpeed();
        stepB.runSpeed();
        stepC.runSpeed();
    }

    stepA.setCurrentPosition(0);
    stepB.setCurrentPosition(0);
    stepC.setCurrentPosition(0);

    // Giải phóng lực căng nếu có (Khoảng cách toGo hiện tại bằng 0 nên vòng lặp này sẽ qua nhanh)
    while (stepA.distanceToGo() || stepB.distanceToGo() || stepC.distanceToGo())
    {
        stepA.run();
        stepB.run();
        stepC.run();
    }

    stepA.setCurrentPosition(0);
    stepB.setCurrentPosition(0);
    stepC.setCurrentPosition(0);

    homeDone = true;
    
    // GỬI TÍN HIỆU CHO LAPTOP BÁO ĐÃ HOME XONG
    Serial1.println("READY");
}

// ================== HÀM NHẬN VÀ GIẢI MÃ UART ==================
void xuly_Uart() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    const byte numChars = 32;
    static char receivedChars[numChars];

    while (Serial1.available() > 0) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                if (ndx < numChars - 1) {
                    receivedChars[ndx] = rc;
                    ndx++;
                }
            }
            else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                
                // Tách chuỗi dữ liệu góc
                char* strtokIndx;
                strtokIndx = strtok(receivedChars, ",");
                if(strtokIndx != NULL) target_theta1 = atof(strtokIndx);

                strtokIndx = strtok(NULL, ",");
                if(strtokIndx != NULL) target_theta2 = atof(strtokIndx);

                strtokIndx = strtok(NULL, ",");
                if(strtokIndx != NULL) target_theta3 = atof(strtokIndx);

                // Chuyển đổi từ Góc (độ) sang số Bước (Steps) tương ứng và nạp mục tiêu mới
                stepA.moveTo((long)(target_theta1 * GOC_TO_STEP));
                stepB.moveTo((long)(target_theta2 * GOC_TO_STEP));
                stepC.moveTo((long)(target_theta3 * GOC_TO_STEP));
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// ================== SETUP ==================
void setup()
{
    // Sử dụng Serial1 để truyền nhận dữ liệu với Laptop (Chân TX1/RX1 của STM32)
    Serial1.begin(115200);

    pinMode(LIMIT_A, INPUT_PULLUP);
    pinMode(LIMIT_B, INPUT_PULLUP);
    pinMode(LIMIT_C, INPUT_PULLUP);

    stepA.setMaxSpeed(5000);
    stepA.setAcceleration(2000); 

    stepB.setMaxSpeed(5000);
    stepB.setAcceleration(2000);

    stepC.setMaxSpeed(5000);
    stepC.setAcceleration(2000);

    // Chạy chu trình Set Home trước
    doHoming();
}

// ================== LOOP ==================
void loop()
{
    if (homeDone) {
        // Kiểm tra lệnh mới từ Laptop liên tục
        xuly_Uart();
        
        // Cập nhật trạng thái phát xung cho Motor chạy mượt theo gia tốc
        stepA.run();
        stepB.run();
        stepC.run();
    }
}