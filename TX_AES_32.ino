/*
 * ESP32 + SX1278 LoRa Telemetry System
 * 
 * Required Libraries:
 * - LoRa by Sandeep Mistry (install via Library Manager)
 * - TinyGPSPlus by Mikal Hart (for GPS parsing)
 * 
 * SX1278 Wiring to ESP32:
 * SX1278 NSS   -> GPIO 5
 * SX1278 RESET -> GPIO 14
 * SX1278 DIO0  -> GPIO 2
 * SX1278 MOSI  -> GPIO 23
 * SX1278 MISO  -> GPIO 19
 * SX1278 SCK   -> GPIO 18
 * 
 * GPS Module (Neo-6M/7M/8M) Wiring:
 * GPS TX -> ESP32 RX2 (GPIO 33)
 * GPS RX -> ESP32 TX2 (GPIO 32)
 *
 * Debugging LED -> GPIO 13
 *   blinking -> Lora not detected
 *   turn on for 2sec and then off for 1s then turn on -> system is recieving the data
 *
 */
 
// ============================================================================
// TRANSMITTER CODE
// ============================================================================

#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <AESLib.h>
#include <esp_timer.h>

#define LORA_NSS    5
#define LORA_RST    14
#define LORA_DIO0   2

#define LORA_FREQ   433E6  // 433 MHz 

HardwareSerial gpsSerial(2); 
TinyGPSPlus gps;

uint32_t packetCounter = 0;

enum AES_MODE {
  MODE_CBC = 0,
  MODE_CTR = 1,
  MODE_GCM = 2
};
enum AES_KEY_SIZE {
  KEY_128 = 128,
  KEY_192 = 192,
  KEY_256 = 256
};

AESLib aesLib;

byte aes_key[32] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};
byte iv_vector[16] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};
String toHex(byte* data, int len) {
  String result = "";
  for(int i = 0; i < len; i++) {
    if(data[i] < 16) result += "0";
    result += String(data[i], HEX);
  }
  return result;
}
void fromHex(String hex, byte* output, int* len) {
  *len = hex.length() / 2;
  for(int i = 0; i < *len; i++) {
    String byteStr = hex.substring(i * 2, i * 2 + 2);
    output[i] = (byte)strtol(byteStr.c_str(), NULL, 16);
  }
}

String AES_Encryption(AES_MODE mode, AES_KEY_SIZE keySize, String plaintext) {
  uint16_t inputLen = plaintext.length();
  uint16_t cipherLen = aesLib.get_cipher_length(inputLen);
  byte input[inputLen + 1];
  byte output[cipherLen];
  byte key[32];
  byte iv[16];
  plaintext.getBytes(input, inputLen + 1);
  memcpy(key, aes_key, 32);
  memcpy(iv, iv_vector, 16);
  int bits = keySize;
  uint16_t encryptedLen = aesLib.encrypt(input, inputLen, output, key, bits, iv);
  return toHex(output, encryptedLen);
}

String data_to_send;
#define LED_PIN 13

volatile int64_t tx_start_us = 0;
volatile int64_t tx_end_us   = 0;
volatile int64_t tx_airtime_us = 0;
volatile bool tx_done_flag = false;

int64_t toa_prev_us = 0;

int64_t t_end, t_start, encrypt_time_us;

/* void IRAM_ATTR onTxDone() {
  tx_end_us = esp_timer_get_time();
  tx_airtime_us = tx_end_us - tx_start_us;
  tx_done_flag = true;
} */

void onTxDone() {
  tx_end_us = esp_timer_get_time();
  tx_airtime_us = tx_end_us - tx_start_us;
  tx_done_flag = true;
}

void setup() {
Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 33, 32);  // GPS at 9600 baud
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  Serial.println("LoRa Transmitter - GPS Telemetry");
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  
while (!LoRa.begin(LORA_FREQ)) {
  Serial.println("LoRa initialization failed! Retrying...");
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(1000); 
}
LoRa.onTxDone(onTxDone);
/* pinMode(LORA_DIO0, INPUT);
attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onTxDone, RISING);
 */

Serial.println("LoRa Initialized successfully!");

  LoRa.setSpreadingFactor(7);      // SF7 to SF12 (higher = longer range, slower)
  LoRa.setSignalBandwidth(125E3);   // 125 kHz bandwidth
  LoRa.setCodingRate4(5);           // 4/5 coding rate
  LoRa.setTxPower(20);              // 20 dBm transmit power
  
  Serial.println("Waiting for GPS fix...");

}

void loop() {
while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    digitalWrite(LED_PIN, HIGH);
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    uint8_t satellites = gps.satellites.value();

    uint32_t heapTX = ESP.getFreeHeap();
    packetCounter++;
    String telemetryData = String(latitude, 6) + ", " + 
                          String(longitude, 6) + ", " +
                          String(satellites) + ", " +
                          String(heapTX) + ", " +
                          String(packetCounter) + ", " + 
                          String(encrypt_time_us) + ", " +
                          String(toa_prev_us);
   
   t_start = esp_timer_get_time();

   data_to_send = AES_Encryption(MODE_GCM, KEY_256, telemetryData);

   t_end = esp_timer_get_time(); 
    encrypt_time_us = t_end - t_start;
      
tx_done_flag = false;
tx_start_us = esp_timer_get_time();

LoRa.beginPacket();
LoRa.print(data_to_send);
LoRa.endPacket(true);   // ASYNC TX

while (!tx_done_flag) {
  yield();   // important on ESP32
}

toa_prev_us = tx_airtime_us;

    Serial.print("telemetryData: ");
    Serial.println(telemetryData);
    Serial.print("data_to_send: ");
    Serial.print(data_to_send);
    Serial.print("Experimental ToA (us): ");
    Serial.println(tx_airtime_us);

    delay(500); 
  } else {
    Serial.println("Waiting for GPS fix...");
    delay(1000);
  }

}
