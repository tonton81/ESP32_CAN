# ESP32_CAN
ESP32 CAN library based off of FlexCAN_T4 -> https://github.com/tonton81/FlexCAN_T4

Library based off of FlexCAN_T4, but for ESP32.
Includes Circular_Buffer queues, and collin's Object Oriented CAN supported.
Automatic filtering is supported as well for up-to 3 IDs or Range based.

Supported features:
```
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
};
```
