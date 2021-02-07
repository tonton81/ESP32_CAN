#include <ESP32_CAN.h>
ESP32_CAN<RX_SIZE_256, TX_SIZE_16> Can0;


void onReceive(const CAN_message_t &msg) {
  Serial.print("LEN: "); Serial.print(msg.len);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}


void setup() {
  Serial.begin(115200);
  Can0.onReceive(onReceive) ;
  Can0.setRX(26);
  Can0.setTX(25);
  Can0.begin();
  Can0.setBaudRate(95000);
}


void loop() {
  static uint32_t t = millis();
  if ( millis() - t > 1000 ) {
    CAN_message_t msg;
    msg.id = 0x111;
    msg.len = 8;
    msg.buf[0] = 1;
    msg.buf[1] = 2;
    msg.buf[2] = 3;
    msg.buf[3] = 4;
    msg.buf[4] = 5;
    msg.buf[5] = 6;
    msg.buf[6] = 7;
    msg.buf[7] = 8;
    Can0.write(msg);
    t = millis();
  }
  vTaskDelay(1);
}
