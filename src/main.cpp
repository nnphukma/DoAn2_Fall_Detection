#include <Wire.h>
#include <Arduino.h>
#include <stdio.h>
// Định nghĩa chân I2C mặc định cho ESP32-C3 Super Mini
#define I2C_SDA 8
#define I2C_SCL 9

void setup() {
  Serial.begin(115200);
  while (!Serial); // Chờ mở Serial Monitor trên kết nối USB CDC Native

  // Khởi tạo bus I2C với SDA = GPIO 8, SCL = GPIO 9
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("\n==================================");
  Serial.println("  ESP32-C3 Super Mini I2C Scanner ");
  Serial.println("==================================");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Đang quét các địa chỉ I2C...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("-> Đã tìm thấy thiết bị tại địa chỉ: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("-> Lỗi truyền thông tại địa chỉ: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("KHÔNG tìm thấy thiết bị I2C nào! Hãy kiểm tra dây nối SDA/SCL và nguồn 3.3V/GND.\n");
  } else {
    Serial.println("Hoàn thành quét I2C.\n");
  }

  delay(3000); // Quét lại sau mỗi 3 giây
}