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


#ifdef ARDUINO_ARCH_ESP32

#ifndef ESP32_CAN_H
#define ESP32_CAN_H

#include "circular_buffer.h"

#define REG_BASE                   0x3ff6b000
#define REG_MOD                    0x00
#define REG_CMR                    0x01
#define REG_SR                     0x02
#define REG_IR                     0x03
#define REG_IER                    0x04
#define REG_BTR0                   0x06
#define REG_BTR1                   0x07
#define REG_OCR                    0x08
#define REG_ALC                    0x0b
#define REG_ECC                    0x0c
#define REG_EWLR                   0x0d
#define REG_RXERR                  0x0e
#define REG_TXERR                  0x0f
#define REG_SFF                    0x10
#define REG_EFF                    0x10
#define REG_ACRn(n)                (0x10 + n)
#define REG_AMRn(n)                (0x14 + n)
#define REG_RMC                    0x1D
#define REG_CDR                    0x1F

#define ESP32_CAN_TASK_SUSPEND \
  if ( eTaskGetState(CANBUS_TASK) == eSuspended ) was_suspended = 1; \
  vTaskSuspend(CANBUS_TASK);


#define ESP32_CAN_TASK_RESTORE \
  if ( !was_suspended ) vTaskResume(CANBUS_TASK); \
  was_suspended = 0;


typedef struct CAN_message_t {
  uint32_t id = 0;          // can identifier
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = { 0 };       // data
} CAN_message_t;

typedef enum ESP32_CAN_RXQUEUE_TABLE {
  RX_SIZE_2 = (uint16_t)2,
  RX_SIZE_4 = (uint16_t)4,
  RX_SIZE_8 = (uint16_t)8,
  RX_SIZE_16 = (uint16_t)16,
  RX_SIZE_32 = (uint16_t)32,
  RX_SIZE_64 = (uint16_t)64,
  RX_SIZE_128 = (uint16_t)128,
  RX_SIZE_256 = (uint16_t)256,
  RX_SIZE_512 = (uint16_t)512,
  RX_SIZE_1024 = (uint16_t)1024
} ESP32_CAN_RXQUEUE_TABLE;

typedef enum ESP32_CAN_TXQUEUE_TABLE {
  TX_SIZE_2 = (uint16_t)2,
  TX_SIZE_4 = (uint16_t)4,
  TX_SIZE_8 = (uint16_t)8,
  TX_SIZE_16 = (uint16_t)16,
  TX_SIZE_32 = (uint16_t)32,
  TX_SIZE_64 = (uint16_t)64,
  TX_SIZE_128 = (uint16_t)128,
  TX_SIZE_256 = (uint16_t)256,
  TX_SIZE_512 = (uint16_t)512,
  TX_SIZE_1024 = (uint16_t)1024
} ESP32_CAN_TXQUEUE_TABLE;

typedef enum ESP32_CAN_IDE {
  NONE = 0,
  EXT = 1,
  RTR = 2,
  STD = 3,
  ACCEPT_ALL = 4,
  ASSUMED_IDE = 5,
  INACTIVE
} ESP32_CAN_IDE;

typedef enum ESP32_CAN_LISTEN_ONLY {
  NORMAL_MODE,
  LISTEN_ONLY
} ESP32_CAN_LISTEN_ONLY;

typedef void (*_ESP32_CAN_ptr)(const CAN_message_t &msg);

#define ESP32_CAN_CLASS template<ESP32_CAN_RXQUEUE_TABLE _rxSize = RX_SIZE_16, ESP32_CAN_TXQUEUE_TABLE _txSize = TX_SIZE_16>
#define ESP32_CAN_FUNC template<ESP32_CAN_RXQUEUE_TABLE _rxSize, ESP32_CAN_TXQUEUE_TABLE _txSize>
#define ESP32_CAN_OPT ESP32_CAN<_rxSize, _txSize>

#define SIZE_LISTENERS 4

class CANListener {
  public:
    CANListener () {;}
    virtual bool frameHandler (CAN_message_t &frame, int mailbox, uint8_t controller) { return false; }
    virtual void txHandler (int mailbox, uint8_t controller) {;} /* unused in Collin's library */
    void attachGeneralHandler (void) { generalCallbackActive = 1; }
    void detachGeneralHandler (void) { generalCallbackActive = 0; }
    bool generalCallbackActive = 0;
};

class ESP32_CAN_Base {
  public:
    virtual void handleInterrupt() = 0;
    virtual uint64_t events() = 0;
    virtual void _CAN_EVENTS_COMMON() = 0;
    virtual int messages_available() = 0;
    virtual int write(const CAN_message_t &msg) = 0;
    volatile bool isEventsUsed = 0;
    virtual void tx_task() = 0;
    virtual uint8_t error_report() = 0;
    volatile uint8_t error = 0;
};

static ESP32_CAN_Base* _CAN = nullptr;
extern void ext_output1(const CAN_message_t &msg); /* for external libraries */
extern void ext_output2(const CAN_message_t &msg);
extern void ext_output3(const CAN_message_t &msg);

ESP32_CAN_CLASS class ESP32_CAN : public ESP32_CAN_Base {
  public:
    ESP32_CAN();
    CANListener *listener[SIZE_LISTENERS];
    bool attachObj (CANListener *listener);
    bool detachObj (CANListener *listener);
    void begin();
    int write(const CAN_message_t &msg);
    void setTX(uint8_t pin);
    void setRX(uint8_t pin);
    uint64_t events();
    uint32_t setBaudRate(uint32_t baud, ESP32_CAN_LISTEN_ONLY listen_only = NORMAL_MODE);
    void onReceive(_ESP32_CAN_ptr handler);
    void setFilter(uint32_t id, ESP32_CAN_IDE frame_type = ASSUMED_IDE);
    void setFilter(uint32_t id1, uint32_t id2, ESP32_CAN_IDE frame_type = ASSUMED_IDE);
    void setFilter(uint32_t id1, uint32_t id2, uint32_t id3, ESP32_CAN_IDE frame_type = ASSUMED_IDE);
    void setFilterRange(uint32_t id1, uint32_t id2, ESP32_CAN_IDE frame_type = ASSUMED_IDE);
    void _CAN_EVENTS_COMMON();
    int messages_available() { return rxBuffer.size(); }
    uint8_t error_report();

  private:
    Circular_Buffer<uint8_t, (uint32_t)_rxSize, sizeof(CAN_message_t)> rxBuffer;
    Circular_Buffer<uint8_t, (uint32_t)_txSize, sizeof(CAN_message_t)> txBuffer;
    void struct2queueTx(const CAN_message_t &msg);
    void struct2queueRx(const CAN_message_t &msg);
    uint32_t currentBitrate = 500000;
    _ESP32_CAN_ptr _mainHandler;
    void handleInterrupt();
    void tx_task();
    static void onInterrupt(void* arg) { _CAN->handleInterrupt(); }
    intr_handle_t _intrHandle;
    volatile bool was_suspended = 0;
};

#include "ESP32_CAN.tpp"
#endif
#endif
