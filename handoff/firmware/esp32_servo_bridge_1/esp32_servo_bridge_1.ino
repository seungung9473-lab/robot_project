const int PUMP = 12, VALVE = 9, PRESS = 10;

bool streaming = true;      // 켜자마자 실시간 출력
int  baseline  = 0;         // 대기압 기준값 (z로 설정)
unsigned long lastPrint = 0;

int readPressure() {        // 16번 읽어서 평균 (잡음 줄이기)
  long sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PRESS);
  return sum / 16;
}

void setup() {
  pinMode(PUMP, OUTPUT);  pinMode(VALVE, OUTPUT);
  digitalWrite(PUMP, LOW); digitalWrite(VALVE, LOW);
  Serial.begin(115200);
  delay(500);
  baseline = readPressure();          // 시작할 때 대기압을 기준으로 잡음
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') digitalWrite(PUMP, HIGH);   // 펌프 켜기
    if (c == 'o') digitalWrite(PUMP, LOW);    // 펌프 끄기
    if (c == 'v') digitalWrite(VALVE, HIGH);  // 밸브 켜기
    if (c == 'c') digitalWrite(VALVE, LOW);   // 밸브 끄기
    if (c == 's') streaming = !streaming;     // 실시간 출력 켜기/끄기
    if (c == 'z') baseline = readPressure();  // 지금 값을 기준(0)으로
  }

  if (streaming && millis() - lastPrint >= 200) {   // 0.2초마다 출력
    lastPrint = millis();
    int raw = readPressure();
    Serial.print("raw:");    Serial.print(raw);
    Serial.print(" diff:");  Serial.print(raw - baseline);
    Serial.print(" pump:");  Serial.print(digitalRead(PUMP) ? 1000 : 0);
    Serial.print(" valve:"); Serial.println(digitalRead(VALVE) ? 500 : 0);
  }
}
