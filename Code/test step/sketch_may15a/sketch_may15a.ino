const int pul = PA2; 
const int dir = PA3;

void setup() {
  pinMode(pul, OUTPUT);
  pinMode(dir, OUTPUT);
}

void loop() {
  // QUAY THUẬN
  digitalWrite(dir, HIGH);
  for(int i = 0; i < 200; i++) {
    digitalWrite(pul, HIGH);
    delayMicroseconds(300); // Tốc độ (giảm số này để quay nhanh hơn)
    digitalWrite(pul, LOW);
    delayMicroseconds(300);
  }
  
  delay(1000); // Nghỉ 1s

  // ĐẢO CHIỀU
  digitalWrite(dir, LOW);
  for(int i = 0; i < 200; i++) {
    digitalWrite(pul, HIGH);
    delayMicroseconds(300);
    digitalWrite(pul, LOW);
    delayMicroseconds(300);
  }
  
  delay(1000);
}