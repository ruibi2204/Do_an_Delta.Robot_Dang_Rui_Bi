// Định nghĩa chân
const int limitSwitch = PB0; 

void setup() {
  // Khởi tạo Serial1 ở baudrate 9600
  // Kết nối: TX của PL2303 -> PA10 (RX1), RX của PL2303 -> PA9 (TX1)
  Serial1.begin(9600);

  // Cấu hình chân PA6 có trở kéo lên nội bộ
  pinMode(limitSwitch, INPUT_PULLUP);

  Serial1.println("--- STM32 F103 & PL2303 TEST ---");
  Serial1.println("Doc tin hieu tu chan PA6...");
}

void loop() {
  int state = digitalRead(limitSwitch);

  if (state == LOW) {
    Serial1.println("Trang thai: DA CHAM (LOW)");
  } else {
    Serial1.println("Trang thai: CHUA CHAM (HIGH)");
  }

  delay(200); // Đọc mỗi 0.2 giây
}