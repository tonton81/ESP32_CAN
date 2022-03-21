/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Designed and tested for ESP32.

  Thanks goes to skpang, mjs513, and collin for tech/testing support

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#include <ESP32_CAN.h>

#include "esp_system.h"
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR > 3
#include "esp_intr_alloc.h"
#else
#include "esp_intr.h"
#endif

#include "soc/dport_reg.h"
#include "driver/gpio.h"

TaskHandle_t CANBUS_TASK = NULL;

static void _CAN_TASK(void * parameter) {
  while(1) {
    static volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
    while( !(addr[REG_IER] & 0x1) ) vTaskDelay(1000);
/*
    If bus-off is triggered, the controller disables itself with no
    self-recovery. The CPU must manually re-enable the controller.
*/
    if ( (addr[REG_MOD] & 0x1) ) addr[REG_MOD] &= ~0x1;

/*
    If the controller is active:
      1) if the error counters exceed the limit, reset the counters
      2) transmit only when bus-on and counter limit is not flagged.
*/
    if ( !(addr[REG_MOD] & 0x1) ) { /* not in reset mode */
      if ( (addr[REG_SR] & 0xC0) ) {
        addr[REG_MOD] |= 0x1;
        addr[REG_RXERR] = addr[REG_TXERR] = 0x00;
        addr[REG_MOD] &= ~0x1;
      }
      if ( !_CAN->isEventsUsed && _CAN->messages_available() ) _CAN->_CAN_EVENTS_COMMON();
      if ( !((addr[REG_SR] & 0xC0) == 0xC0) ) { /* transmit only when bus off and error states are good */
        _CAN->tx_task();
      }
    }
    vTaskDelay(1);
  }
  vTaskDelete( NULL );
}


ESP32_CAN_FUNC ESP32_CAN_OPT::ESP32_CAN() {
  _CAN = this;
  xTaskCreatePinnedToCore(
    _CAN_TASK,     /* Task function. */
    "_CAN_TASK",   /* name of task. */
    2048 * 2,      /* Stack size of task */
    NULL,          /* parameter of the task */
    4,             /* priority of the task */
    &CANBUS_TASK,
    1);            /* CORE 1 */         
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::setTX(uint8_t pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
  gpio_matrix_out((gpio_num_t)pin, CAN_TX_IDX, 0, 0);
  gpio_pad_select_gpio((gpio_num_t)pin);
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::setRX(uint8_t pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  gpio_matrix_in((gpio_num_t)pin, CAN_RX_IDX, 0);
  gpio_pad_select_gpio((gpio_num_t)pin);
}


ESP32_CAN_FUNC uint32_t ESP32_CAN_OPT::setBaudRate(uint32_t baud, ESP32_CAN_LISTEN_ONLY listen_only) {
  ESP32_CAN_TASK_SUSPEND;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  currentBitrate = baud;
  bool wasResetMode = !(addr[REG_MOD] & 0x1);
  addr[REG_MOD] |= 0x1;
  int error, brp, tseg, tseg1 = 0, tseg2 = 0, MAX_TSEG1 = 15,
                        MAX_TSEG2 = 7, best_error = 1000000000,
                        best_tseg = 0, best_brp = 0, best_baud = 0,
                        JUMPWIDTH = 0x40, SAM = (baud > 100000 ? 0 : 1),
                        SAMPLE_POINT = 75, clock = 80000000 >> 1;
  for (tseg = (0 + 0 + 2) * 2; tseg <= (MAX_TSEG2 + MAX_TSEG1 + 2) * 2 + 1; tseg++) {
    brp = clock / ((1 + tseg / 2) * baud) + tseg % 2;
    if ((brp > 0) && (brp <= 64)) {
      error = baud - clock / (brp * (1 + tseg / 2));
      if (error < 0) error = -error;
      if (error <= best_error) {
        best_error = error;
        best_tseg = tseg / 2;
        best_brp = brp - 1;
        best_baud = clock / (brp * (1 + tseg / 2));
      }
    }
  }
  tseg2 = best_tseg - (SAMPLE_POINT * (best_tseg + 1)) / 100;
  if (tseg2 < 0) tseg2 = 0;
  else if (tseg2 > MAX_TSEG2) tseg2 = MAX_TSEG2;
  tseg1 = best_tseg - tseg2 - 2;
  if (tseg1 > MAX_TSEG1) {
    tseg1 = MAX_TSEG1;
    tseg2 = best_tseg - tseg1 - 2;
  }
  int priv_btr = ((best_brp | JUMPWIDTH)<<8) + ((SAM << 7) | (tseg2 << 4) | tseg1);
  addr[REG_CDR] = 0xCF; /* pelican mode, clockout disabled, comparator disabled */
  addr[REG_BTR0] = priv_btr >> 8;
  addr[REG_BTR1] = priv_btr;
  addr[REG_RXERR] = addr[REG_TXERR] = 0x00; /* reset error counters */
  ( listen_only == LISTEN_ONLY ) ? addr[REG_MOD] |= 0x2 : addr[REG_MOD] &= ~0x2;
  if ( wasResetMode ) addr[REG_MOD] &= ~0x1;
  ESP32_CAN_TASK_RESTORE;
  return best_baud;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::begin() {
  vTaskSuspend(CANBUS_TASK); /* suspend task until configured */
  for (uint8_t i = 0; i < SIZE_LISTENERS; i++) listener[i] = nullptr;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
  setBaudRate(currentBitrate, LISTEN_ONLY);
  addr[REG_MOD] |= 0x1;
  for ( uint8_t i = 0; i < 4; i++ ) { /* set filters to allow everything */
    addr[REG_ACRn(i)] = 0x0;
    addr[REG_AMRn(i)] = 0xFF;
  }
  addr[REG_OCR] = 0x02; /* normal output mode */
  (void)addr[REG_ECC]; /* clear errors */ 
  (void)addr[REG_IR]; /* clear interrupts */
  addr[REG_MOD] = 0xA; /* single filter mode, listen only mode */
  addr[REG_IER] = 0x1; /* receive interrupt */
  vTaskResume(CANBUS_TASK); /* resume task after configuration */
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::onReceive(_ESP32_CAN_ptr handler) {
  _mainHandler = handler;
  if (_intrHandle) {
    esp_intr_free(_intrHandle);
    _intrHandle = nullptr;
  }
  esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, ESP32_CAN_OPT::onInterrupt, this, &_intrHandle);
}


ESP32_CAN_FUNC uint8_t ESP32_CAN_OPT::error_report() {
  if ( !error ) return 0;
  Serial.println("\n\n****************************************\n");
  Serial.print("*** Error Report: ");
  if ( ((error & 0xC0) >> 6) == 0 ) Serial.print("bit error");
  if ( ((error & 0xC0) >> 6) == 1 ) Serial.print("form error");
  if ( ((error & 0xC0) >> 6) == 2 ) Serial.print("stuff error");
  if ( ((error & 0xC0) >> 6) == 3 ) Serial.print("other type of error");
  Serial.printf("   Direction: %s   ", ((error & 0x20) ? "RX" : "TX"));
  if ( (error & 0x1F) == 0b00011 ) Serial.print("start of frame");
  if ( (error & 0x1F) == 0b00010 ) Serial.print("ID.28 to ID.21");
  if ( (error & 0x1F) == 0b00110 ) Serial.print("ID.20 to ID.18");
  if ( (error & 0x1F) == 0b00100 ) Serial.print("bit SRTR");
  if ( (error & 0x1F) == 0b00101 ) Serial.print("bit IDE");
  if ( (error & 0x1F) == 0b00111 ) Serial.print("ID.17 to ID.13");
  if ( (error & 0x1F) == 0b01111 ) Serial.print("ID.12 to ID.5");
  if ( (error & 0x1F) == 0b01110 ) Serial.print("ID.4 to ID.0");
  if ( (error & 0x1F) == 0b01100 ) Serial.print("bit RTR");
  if ( (error & 0x1F) == 0b01101 ) Serial.print("reserved bit 1");
  if ( (error & 0x1F) == 0b01001 ) Serial.print("reserved bit 0");
  if ( (error & 0x1F) == 0b01010 ) Serial.print("data field");
  if ( (error & 0x1F) == 0b01000 ) Serial.print("CRC sequence");
  if ( (error & 0x1F) == 0b11000 ) Serial.print("CRC delimiter");
  if ( (error & 0x1F) == 0b11001 ) Serial.print("acknowledge slot");
  if ( (error & 0x1F) == 0b11011 ) Serial.print("acknowledge delimiter");
  if ( (error & 0x1F) == 0b11010 ) Serial.print("end of frame");
  if ( (error & 0x1F) == 0b10010 ) Serial.print("intermission");
  if ( (error & 0x1F) == 0b10001 ) Serial.print("active error flag");
  if ( (error & 0x1F) == 0b10110 ) Serial.print("passive error flag");
  if ( (error & 0x1F) == 0b10011 ) Serial.print("tolerate dominant bits");
  if ( (error & 0x1F) == 0b10111 ) Serial.print("error delimiter");
  if ( (error & 0x1F) == 0b11100 ) Serial.print("overload flag");
  Serial.println("\n****************************************\n");
  return error;
}


ESP32_CAN_FUNC void IRAM_ATTR ESP32_CAN_OPT::handleInterrupt() {
  CAN_message_t msg;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  uint8_t interrupt = addr[REG_IR], cmr = 0;
  error = addr[REG_ECC];
/*
  I found a new bug in the hardware FIFO. If the ECC register reports errors
  (no relation to the RXERR/TXERR registers), that frame may be corrupt and
  partially written to FIFO while the pointer shifts, thereby reporting a valid
  frame when it is not. This register is only active if you have bad line issues.
  Bad cabling/termination, hot-plugging, and shorting the CANH/CANL lines for
  stability checks. This workaround discards 1x FIFO queue per interrupt if the
  ECC register is being flagged. If the line returns to normal then so does
  the ISR. Bug was found during a slave node setup with switch statement, and an
  un-called switch call was triggered very intermittantly over long periods of time.
  It became aparent more intermittant as I started shorting the line multiple times
  and seeing the switch statement being fired, even though I was not calling it!
  This has ressolved that issue.
*/
  if ( error ) {
    addr[REG_CMR] = 0xC;
    return;
  }

/*
  Always check the RMC (NOT JUST the IER receive interrupt bit).
  I have noticed the RMC sometimes returns 0 when an interrupt occurs.
  The RMC is the count of queues in the HW FIFO.
*/
  if ( (interrupt & (1U << 0)) && (addr[REG_RMC]) ) { /* Receive Interrupt */
    msg.flags.extended = (addr[REG_SFF] & 0x80) ? true : false;
    msg.flags.remote = (addr[REG_SFF] & 0x40) ? true : false;
    msg.len = (addr[REG_SFF] & 0x0f);
    if (msg.flags.extended) {
      msg.id = (addr[REG_EFF + 1] << 21) | (addr[REG_EFF + 2] << 13) | (addr[REG_EFF + 3] << 5) | (addr[REG_EFF + 4] >> 3);
      for (int i = 0; i < msg.len; i++) msg.buf[i] = addr[REG_EFF + 5 + i];
    } else {
      msg.id = (addr[REG_SFF + 1] << 3) | ((addr[REG_SFF + 2] >> 5) & 0x07);
      for (int i = 0; i < msg.len; i++) msg.buf[i] = addr[REG_SFF + 3 + i];
    }
    if (msg.flags.remote) msg.len = 0;
    cmr |= (1U << 2); /* Release Receive Buffer */
    struct2queueRx(msg);
  }
  if (interrupt & (1U << 0)) { /* transmit interrupt */
  }
  if (interrupt & (1U << 3)) {
    cmr |= (1U << 3); /* Clear Data Overrun */
  }
  addr[REG_CMR] |= cmr;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::struct2queueRx(const CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  memmove(buf, &msg, sizeof(msg));
  rxBuffer.push_back(buf, sizeof(CAN_message_t));
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::struct2queueTx(const CAN_message_t &msg) {
  uint8_t buf[sizeof(CAN_message_t)];
  memmove(buf, &msg, sizeof(msg));
  txBuffer.push_back(buf, sizeof(CAN_message_t));
}


ESP32_CAN_FUNC int ESP32_CAN_OPT::write(const CAN_message_t &msg) {
  if ( txBuffer.size() == txBuffer.capacity() ) return 0;
  struct2queueTx(msg);
  return 1;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::tx_task() {
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  if ( txBuffer.size() && ( addr[REG_SR] & (1U << 2) ) ) {
    CAN_message_t msg;
    uint8_t buf[sizeof(CAN_message_t)];
    txBuffer.pop_front(buf, sizeof(CAN_message_t));
    memmove(&msg, buf, sizeof(msg));
    if (msg.flags.extended) {
      addr[REG_EFF] = 0x80 | (msg.flags.remote ? 0x40 : 0x00) | msg.len;
      addr[REG_EFF + 1] = msg.id >> 21;
      addr[REG_EFF + 2] = msg.id >> 13;
      addr[REG_EFF + 3] = msg.id >> 5;
      addr[REG_EFF + 4] = msg.id << 3;
      for (int i = 0; i < msg.len; i++) addr[REG_EFF + 5 + i] = msg.buf[i];
    } else {
      addr[REG_SFF] = (msg.flags.remote ? 0x40 : 0x00) | msg.len;
      addr[REG_SFF + 1] = msg.id >> 3;
      addr[REG_SFF + 2] = msg.id << 5;
      for (int i = 0; i < msg.len; i++) addr[REG_SFF + 3 + i] = msg.buf[i];
    }
    addr[REG_CMR] = 1;
  }
}


ESP32_CAN_FUNC uint64_t ESP32_CAN_OPT::events() {
  if ( !_CAN->isEventsUsed ) _CAN->isEventsUsed = 1;
  _CAN_EVENTS_COMMON();
  return (uint64_t)(rxBuffer.size() << 12) | txBuffer.size();
}


ESP32_CAN_FUNC void IRAM_ATTR ESP32_CAN_OPT::_CAN_EVENTS_COMMON() {
  if ( rxBuffer.size() ) {
    CAN_message_t frame;
    uint8_t buf[sizeof(CAN_message_t)];
    rxBuffer.pop_front(buf, sizeof(CAN_message_t));
    memmove(&frame, buf, sizeof(frame));
    if ( _mainHandler ) _mainHandler(frame);

    if ( ext_output1 ) ext_output1(frame);
    if ( ext_output2 ) ext_output2(frame);
    if ( ext_output3 ) ext_output3(frame);

    CANListener *thisListener;
    CAN_message_t cl = frame;
    for (uint8_t listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++) {
      thisListener = listener[listenerPos];
      if (thisListener != nullptr) {
        if (thisListener->generalCallbackActive) thisListener->frameHandler (cl, -1, 0);
      }
    }
  }
}


ESP32_CAN_FUNC bool ESP32_CAN_OPT::attachObj (CANListener *listener) {
  for (uint8_t i = 0; i < SIZE_LISTENERS; i++) {
    if (this->listener[i] == nullptr) {
      this->listener[i] = listener;
      return true;
    }
  }
  return false;
}


ESP32_CAN_FUNC bool ESP32_CAN_OPT::detachObj (CANListener *listener) {
  for (uint8_t i = 0; i < SIZE_LISTENERS; i++) {
    if (this->listener[i] == listener) {
      this->listener[i] = nullptr;
      return true;
    }
  }
  return false;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::setFilter(uint32_t id, ESP32_CAN_IDE frame_type) {
  ESP32_CAN_TASK_SUSPEND;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  bool wasResetMode = !(addr[REG_MOD] & 0x1);
  addr[REG_MOD] |= 0x1;
  addr[REG_MOD] |= 0x8; /* single frame mode */
  if ( frame_type == ASSUMED_IDE) {
    if ( id <= 0x7FF ) frame_type = STD; // Standard frame
    else frame_type = EXT; // Extended frame
  }
  if ( frame_type == STD ) { // Standard frame
    id &= 0x7FF;
    uint32_t mask = ~((((id) ^ (id)) ^ 0x7FF) & 0x7FF);
    addr[REG_ACRn(0)] = (uint8_t)(id >> 3);
    addr[REG_ACRn(1)] = (uint8_t)(id << 5);
    addr[REG_ACRn(2)] = 0x0;
    addr[REG_ACRn(3)] = 0x0;
    addr[REG_AMRn(0)] = (mask >> 3);
    addr[REG_AMRn(1)] = (mask << 5);
    addr[REG_AMRn(2)] = 0xFF;
    addr[REG_AMRn(3)] = 0xFF;
  }
  else { // Extended frame
    id &= 0x1FFFFFFF;
    uint32_t mask = ~((((id) ^ (id)) ^ 0x1FFFFFFF) & 0x1FFFFFFF);
    addr[REG_ACRn(0)] = (uint8_t)((id << (3)) >> 24);
    addr[REG_ACRn(1)] = (uint8_t)((id << (3)) >> 16);
    addr[REG_ACRn(2)] = (uint8_t)((id << (3)) >> 8);
    addr[REG_ACRn(3)] = (uint8_t)((id << (3)) >> 0);
    addr[REG_AMRn(0)] = (uint8_t)((mask << (3)) >> 24);
    addr[REG_AMRn(1)] = (uint8_t)((mask << (3)) >> 16);
    addr[REG_AMRn(2)] = (uint8_t)((mask << (3)) >> 8);
    addr[REG_AMRn(3)] = (uint8_t)((mask << (3)) >> 0);
  }
  if ( wasResetMode ) addr[REG_MOD] &= ~0x1;
  ESP32_CAN_TASK_RESTORE;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::setFilter(uint32_t id1, uint32_t id2, ESP32_CAN_IDE frame_type) {
  ESP32_CAN_TASK_SUSPEND;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  bool wasResetMode = !(addr[REG_MOD] & 0x1);
  addr[REG_MOD] |= 0x1;
  addr[REG_MOD] &= ~0x8; /* dual frame mode */
  if ( frame_type == ASSUMED_IDE ) {
    if ( id1 <= 0x7FF && id2 <= 0x7FF ) { // Double Standard frames
      id1 &= 0x7FF;
      id2 &= 0x7FF;
      uint32_t mask = ~((((id1) ^ (id1)) ^ 0x7FF) & 0x7FF);
      addr[REG_ACRn(0)] = (uint8_t)(id1 >> 3);
      addr[REG_ACRn(1)] = ((uint8_t)(id1 << 5)) & 0xF0;
      addr[REG_ACRn(2)] = (uint8_t)(id2 >> 3);
      addr[REG_ACRn(3)] = ((uint8_t)(id2 << 5)) & 0xF0;
      addr[REG_AMRn(0)] = (mask >> 3);
      addr[REG_AMRn(1)] = ((mask << 5)) | 0xF;
      mask = ~((((id2) ^ (id2)) ^ 0x7FF) & 0x7FF);
      addr[REG_AMRn(2)] = (mask >> 3);
      addr[REG_AMRn(3)] = ((mask << 5) | 0xF);
    }
    else if ( id1 > 0x7FF && id2 > 0x7FF ) { // Double Extended frames
      id1 &= 0x1FFFFFFF;
      id2 &= 0x1FFFFFFF;
      uint32_t mask = ~((((id1) ^ (id1)) ^ 0x1FFFFFFF) & 0x1FFFFFFF);
      addr[REG_ACRn(0)] = (uint8_t)((id1 << (3)) >> 24);
      addr[REG_ACRn(1)] = (uint8_t)((id1 << (3)) >> 16);
      addr[REG_ACRn(2)] = (uint8_t)((id2 << (3)) >> 24);
      addr[REG_ACRn(3)] = (uint8_t)((id2 << (3)) >> 16);
      addr[REG_AMRn(0)] = (uint8_t)((mask << (3)) >> 24);
      addr[REG_AMRn(1)] = (uint8_t)((mask << (3)) >> 16);
      mask = ~((((id2) ^ (id2)) ^ 0x1FFFFFFF) & 0x1FFFFFFF);
      addr[REG_AMRn(2)] = (uint8_t)((mask << (3)) >> 24);
      addr[REG_AMRn(3)] = (uint8_t)((mask << (3)) >> 16);
    }
  }
  if ( wasResetMode ) addr[REG_MOD] &= ~0x1;
  ESP32_CAN_TASK_RESTORE;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::setFilter(uint32_t id1, uint32_t id2, uint32_t id3, ESP32_CAN_IDE frame_type) {
  ESP32_CAN_TASK_SUSPEND;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  bool wasResetMode = !(addr[REG_MOD] & 0x1);
  addr[REG_MOD] |= 0x1;
  addr[REG_MOD] |= 0x8; /* single frame mode */
  if ( frame_type == ASSUMED_IDE ) {
    if ( id1 <= 0x7FF ) frame_type = STD; // Standard frame
    else frame_type = EXT; // Extended frame
  }
  if ( frame_type == STD ) { // Standard frame
    id1 &= 0x7FF;
    id2 &= 0x7FF;
    uint32_t mask = ~((((id1 | id2 | id3) ^ (id1 & id2 &id3)) ^ 0x7FF) & 0x7FF);
    addr[REG_ACRn(0)] = (uint8_t)(id1 >> 3);
    addr[REG_ACRn(1)] = (uint8_t)(id1 << 5);
    addr[REG_ACRn(2)] = 0x0;
    addr[REG_ACRn(3)] = 0x0;
    addr[REG_AMRn(0)] = (mask >> 3);
    addr[REG_AMRn(1)] = (mask << 5);
    addr[REG_AMRn(2)] = 0xFF;
    addr[REG_AMRn(3)] = 0xFF;
  }
  else { // Extended frame
    id1 &= 0x1FFFFFFF;
    id2 &= 0x1FFFFFFF;
    uint32_t mask = ~((((id1 | id2 | id3) ^ (id1 & id2 & id3)) ^ 0x1FFFFFFF) & 0x1FFFFFFF);
    addr[REG_ACRn(0)] = (uint8_t)((id1 << (3)) >> 24);
    addr[REG_ACRn(1)] = (uint8_t)((id1 << (3)) >> 16);
    addr[REG_ACRn(2)] = (uint8_t)((id1 << (3)) >> 8);
    addr[REG_ACRn(3)] = (uint8_t)((id1 << (3)) >> 0);
    addr[REG_ACRn(0)] = (uint8_t)((mask << (3)) >> 24);
    addr[REG_ACRn(1)] = (uint8_t)((mask << (3)) >> 16);
    addr[REG_ACRn(2)] = (uint8_t)((mask << (3)) >> 8);
    addr[REG_ACRn(3)] = (uint8_t)((mask << (3)) >> 0);
  }
  if ( wasResetMode ) addr[REG_MOD] &= ~0x1;
  ESP32_CAN_TASK_RESTORE;
}


ESP32_CAN_FUNC void ESP32_CAN_OPT::setFilterRange(uint32_t id1, uint32_t id2, ESP32_CAN_IDE frame_type) {
  ESP32_CAN_TASK_SUSPEND;
  volatile uint32_t *addr = &(*(volatile uint32_t*)(REG_BASE));
  bool wasResetMode = !(addr[REG_MOD] & 0x1);
  addr[REG_MOD] |= 0x1;
  addr[REG_MOD] |= 0x8; /* single frame mode */
  if ( frame_type == ASSUMED_IDE ) {
    if ( id1 <= 0x7FF ) frame_type = STD; // Standard frame
    else frame_type = EXT; // Extended frame
  }
  if ( frame_type == STD ) { // Standard frame
    id1 &= 0x7FF;
    id2 &= 0x7FF;
    uint32_t stage1 = id1, stage2 = id1;
    for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
      stage1 |= i; stage2 &= i;
    }
    uint32_t mask = ~((((stage1) ^ (stage2)) ^ 0x7FF) & 0x7FF);
    addr[REG_ACRn(0)] = (uint8_t)(id1 >> 3);
    addr[REG_ACRn(1)] = (uint8_t)(id1 << 5);
    addr[REG_ACRn(2)] = 0x0;
    addr[REG_ACRn(3)] = 0x0;
    addr[REG_AMRn(0)] = (mask >> 3);
    addr[REG_AMRn(1)] = (mask << 5);
    addr[REG_AMRn(2)] = 0xFF;
    addr[REG_AMRn(3)] = 0xFF;
  }
  else { // Extended frame
    id1 &= 0x1FFFFFFF;
    id2 &= 0x1FFFFFFF;
    uint32_t stage1 = id1, stage2 = id1;
    for ( uint32_t i = id1 + 1; i <= id2; i++ ) {
      stage1 |= i; stage2 &= i;
    }
    uint32_t mask = ~((((stage1) ^ (stage2)) ^ 0x1FFFFFFF) & 0x1FFFFFFF);
    addr[REG_ACRn(0)] = (uint8_t)((id1 << (3)) >> 24);
    addr[REG_ACRn(1)] = (uint8_t)((id1 << (3)) >> 16);
    addr[REG_ACRn(2)] = (uint8_t)((id1 << (3)) >> 8);
    addr[REG_ACRn(3)] = (uint8_t)((id1 << (3)) >> 0);
    addr[REG_AMRn(0)] = (uint8_t)((mask << (3)) >> 24);
    addr[REG_AMRn(1)] = (uint8_t)((mask << (3)) >> 16);
    addr[REG_AMRn(2)] = (uint8_t)((mask << (3)) >> 8);
    addr[REG_AMRn(3)] = (uint8_t)((mask << (3)) >> 0);
  }
  if ( wasResetMode ) addr[REG_MOD] &= ~0x1;
  ESP32_CAN_TASK_RESTORE;
}


extern void __attribute__((weak)) ext_output1(const CAN_message_t &msg);
extern void __attribute__((weak)) ext_output2(const CAN_message_t &msg);
extern void __attribute__((weak)) ext_output3(const CAN_message_t &msg);
