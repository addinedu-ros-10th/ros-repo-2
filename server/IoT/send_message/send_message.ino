#define LED_PIN 13

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  static bool ledState = false;

  // 테스트용: 1초마다 LED 상태 토글
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);

  int id = 1;
  String sensor = "OB";
  float x = 1.23;
  float y = 4.56;
  float z = 7.89;
  int led = ledState; // LED 상태 (1=ON, 0=OFF)

  // 시리얼로 데이터 전송
  Serial.print(id);
  Serial.print(",");
  Serial.print(sensor);
  Serial.print(",");
  Serial.print(x, 2);
  Serial.print(",");
  Serial.print(y, 2);
  Serial.print(",");
  Serial.print(z, 2);
  Serial.print(",");
  Serial.println(led); // 마지막에 LED 상태 추가

  delay(1000);
}
