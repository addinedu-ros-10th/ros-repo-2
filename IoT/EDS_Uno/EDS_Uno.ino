
 /* -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 * More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
 */

#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN         9          // Configurable, see typical pin layout above
#define SS_PIN          10         // Configurable, see typical pin layout above
#define Btn1_PIN        A0
#define Btn2_PIN        A1
#define LED1_PIN        5
#define LED2_PIN        4

MFRC522 mfrc522_1(SS_PIN, RST_PIN);  // Create MFRC522 instance

bool rfid1_present = false;
bool LED1_status =false , LED2_status = false;

unsigned long prevMillis = 0;   // 마지막 실행 시각 저장
const unsigned long interval = 200; // 0.2초 (200ms)

void setup() {
	Serial.begin(9600);		// Initialize serial communications with the PC
	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	mfrc522_1.PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
	mfrc522_1.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis(); // 현재 시간 읽기

// RFID 카드 태그 및 제거 감지
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

// 버튼 1 -> LED1 ON/OFF => 가스 감지 3번 예시
  if (digitalRead(Btn1_PIN)){
    LED1_status = !LED1_status;
    while(digitalRead(Btn1_PIN));
    digitalWrite(LED1_PIN, LED1_status);
  }
  
// 버튼 2 -> LED2 ON/OFF => 가스 감지 3번 예시
  if (digitalRead(Btn2_PIN)){
    LED2_status = !LED2_status;
    while(digitalRead(Btn2_PIN));
    digitalWrite(LED2_PIN, LED2_status);
  }
  
  if(LED1_status || LED2_status){
    if (currentMillis - prevMillis >= interval) {
      prevMillis = currentMillis; // 기준 시각 갱신

      byte packet = 0;
      packet |= (false << 0);
      packet |= (LED1_status << 1);
      packet |= (LED2_status << 2);
      packet |= (false << 3);
      
      Serial.write(0xAA);
      Serial.write(0x20);
      Serial.write(packet);
      Serial.write(0x55);
    }
  } // 두개 LED중 하나만 켜졌을때
  
}
