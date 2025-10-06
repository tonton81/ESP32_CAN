#include <ESP32_CAN.h> // OONO F-1055 CAN DPDT LATCH
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
  Can0.setBaudRate(125000);

  /* Turn off onboard RGB light */
  const uint8_t led_pins[3] = { 2, 15, 4 }; /* ONBOARD RGB LIGHT */
  for ( int i = 0; i < 3; i++ ) {
    ledcAttachPin(led_pins[i], i + 1); // Bind led to 1-16 PWM channels
    ledcSetup(i + 1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  }
  ledcWrite(1, 256);
  ledcWrite(2, 256);
  ledcWrite(3, 256);


  pinMode(5, INPUT | OUTPUT_OPEN_DRAIN); // SCK
  pinMode(18, INPUT | OUTPUT_OPEN_DRAIN); // MOSI
  pinMode(19, INPUT | OUTPUT_OPEN_DRAIN); // MISO
  pinMode(21, INPUT | OUTPUT_OPEN_DRAIN); // IO21
  digitalWrite(5, HIGH);
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);
  digitalWrite(21, HIGH);

  digitalWrite(18, LOW);
  delay(500);
  digitalWrite(18, HIGH);
  delay(1000);
  digitalWrite(19, LOW);
  delay(500);
  digitalWrite(19, HIGH);
  delay(1000);
  digitalWrite(18, LOW);
  delay(500);
  digitalWrite(18, HIGH);
  delay(1000);
  digitalWrite(19, LOW);
  delay(500);
  digitalWrite(19, HIGH);
}


void loop() {
  static uint32_t t = millis();
  if ( millis() - t > 1000 ) {
    Serial.println(millis());
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
