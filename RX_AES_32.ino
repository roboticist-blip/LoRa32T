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
 * Debugging LED -> GPIO 13
 *   blinking -> Lora not detected
 *   turn on for 2sec and then off for 1s then turn on -> system is recieving the data
 */

// ============================================================================
// RECEIVER CODE
// ============================================================================

 
#include <LoRa.h>
#include <AESLib.h>

#define LORA_NSS    5
#define LORA_RST    14
#define LORA_DIO0   2
#define LORA_FREQ   433E6  // 433 MHz -> must match transmitter

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

String AES_Decryption(AES_MODE mode, AES_KEY_SIZE keySize, String cipherHex) {
  byte cipherInput[512];
  int cipherLen = 0;
  fromHex(cipherHex, cipherInput, &cipherLen);
  byte output[cipherLen];
  byte key[32];
  byte iv[16];
  memcpy(key, aes_key, 32);
  memcpy(iv, iv_vector, 16);
  int bits = keySize;
  uint16_t decryptedLen = aesLib.decrypt(cipherInput, cipherLen, output, key, bits, iv);
  String result = "";
  for(int i = 0; i < decryptedLen; i++) {
    if(output[i] == 0 || output[i] > 127) {
      break;
    }
    result += (char)output[i];
  }
  return result;
}

String decrypted;
int64_t t_end, t_start, decrypt_time_us;

#define LED_PIN 13
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("LoRa Receiver - Telemetry Monitor");
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  
while (!LoRa.begin(LORA_FREQ)) {
  Serial.println("LoRa initialization failed! Retrying...");
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(1000); 
}

Serial.println("LoRa Initialized successfully!");
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  Serial.println("Waiting for packets...\n");
  Serial.println(" The data is in format of LAT, LON, SATELLITES, HEAP_TX, MESSAGE_COUNT,en_time,EXP_Tao, HEAP_RX, RSSI, SNR, dec_time");
}

void loop() {
int packetSize = LoRa.parsePacket();
  
  if (packetSize) { String receivedData = "";
  digitalWrite(LED_PIN, HIGH);
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    uint32_t heapRX = ESP.getFreeHeap();
    t_start = esp_timer_get_time();  
    decrypted = AES_Decryption(MODE_GCM, KEY_256, receivedData);
    t_end = esp_timer_get_time(); 
    decrypt_time_us = t_end - t_start;

     String outputData = decrypted + ", " + 
                       String(heapRX) + ", " + 
                       String(rssi) + ", " + 
                       String(snr, 2)+ ", " + 
                       String(decrypt_time_us);

    Serial.print("data: ");
    Serial.println(outputData);
}
}