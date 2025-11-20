#include <SPI.h>
#include <MFRC522.h>

// === RFID 핀 설정 ===
#define RST_PIN_1 9
#define SS_PIN_1  53  // RFID 1

#define RST_PIN_2 8
#define SS_PIN_2  49  // RFID 2

#define RST_PIN_3 7
#define SS_PIN_3  47  // RFID 3 (추가)

// === RFID 객체 3개 생성 ===
MFRC522 mfrc522_1(SS_PIN_1, RST_PIN_1);
MFRC522 mfrc522_2(SS_PIN_2, RST_PIN_2);
MFRC522 mfrc522_3(SS_PIN_3, RST_PIN_3);

// === 가스 센서 핀 ===
#define SENSOR1 54
#define SENSOR2 55
#define SENSOR3 56
#define SENSOR4 57

int val1, val2, val3, val4;

bool rfid1_present = false;
bool rfid2_present = false;
bool rfid3_present = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();

  // RFID 초기화
  mfrc522_1.PCD_Init();
  mfrc522_2.PCD_Init();
  mfrc522_3.PCD_Init();

  delay(4);
  Serial.println(F("✅ 시스템 시작됨. 3개의 RFID 및 가스 센서 대기 중..."));
}

void loop() {
  // === RFID 1 ===
  if (mfrc522_1.PICC_IsNewCardPresent() && mfrc522_1.PICC_ReadCardSerial()) {
    if (!rfid1_present) {
      rfid1_present = true;
      Serial.write(0xAA);
      Serial.write(0x11); // 태그 인식됨
      for (byte i = 0; i < mfrc522_1.uid.size; i++)
        Serial.write(mfrc522_1.uid.uidByte[i]);
      Serial.write(0x55);
    }
    mfrc522_1.PICC_HaltA();
    mfrc522_1.PCD_StopCrypto1();
  } else {
    if (rfid1_present) {
      rfid1_present = false;
      // Serial.write(0xAA);
      // Serial.write(0x21); // 태그 제거됨
      // Serial.write(0x55);
    }
  }

  // === RFID 2 ===
  if (mfrc522_2.PICC_IsNewCardPresent() && mfrc522_2.PICC_ReadCardSerial()) {
    if (!rfid2_present) {
      rfid2_present = true;
      Serial.write(0xAA);
      Serial.write(0x12);
      for (byte i = 0; i < mfrc522_2.uid.size; i++)
        Serial.write(mfrc522_2.uid.uidByte[i]);
      Serial.write(0x55);
    }
    mfrc522_2.PICC_HaltA();
    mfrc522_2.PCD_StopCrypto1();
  } else {
    if (rfid2_present) {
      rfid2_present = false;
      // Serial.write(0xAA);
      // Serial.write(0x22); // 태그 제거됨
      // Serial.write(0x55);
    }
  }

  // === RFID 3 ===
  if (mfrc522_3.PICC_IsNewCardPresent() && mfrc522_3.PICC_ReadCardSerial()) {
    if (!rfid3_present) {
      rfid3_present = true;
      Serial.write(0xAA);
      Serial.write(0x13);
      for (byte i = 0; i < mfrc522_3.uid.size; i++)
        Serial.write(mfrc522_3.uid.uidByte[i]);
      Serial.write(0x55);
    }
    mfrc522_3.PICC_HaltA();
    mfrc522_3.PCD_StopCrypto1();
  } else {
    if (rfid3_present) {
      rfid3_present = false;
      // Serial.write(0xAA);
      // Serial.write(0x23); // 태그 제거됨
      // Serial.write(0x55);
    }
  }

  // === 가스 센서 ===
  val1 = analogRead(SENSOR1);
  val2 = analogRead(SENSOR2);
  val3 = analogRead(SENSOR3);
  val4 = analogRead(SENSOR4);

  bool gas1 = (val1 > 400);
  bool gas2 = (val2 > 400);
  bool gas3 = (val3 > 400);
  bool gas4 = (val4 > 400);

  byte packet = 0;
  packet |= (gas1 << 0);
  packet |= (gas2 << 1);
  packet |= (gas3 << 2);
  packet |= (gas4 << 3);

  Serial.write(0xAA);
  Serial.write(0x20);
  Serial.write(packet);
  Serial.write(0x55);

  delay(100);
}