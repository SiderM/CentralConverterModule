#ifndef Ibus_h
#define Ibus_h

#include "Arduino.h"
#include "IbusMessage.h"
#include "IbusNames.h"

class Ibus
{
public:
  void begin(HardwareSerial &userPort);
  void end();
  void write(uint8_t message[]);
  bool available();
  bool transmitWaiting();
  uint8_t length();
  IbusMessage readMessage();

private:
  HardwareSerial *serialPort;
  void clearBuffer();
  bool checkMessage();
  bool tx_msg_waiting = false;      // message waiting in transmit buffer
  bool rx_msg_waiting = false;      // message waiting in receive buffer
  uint8_t rx_buffer[0xFF] = {0x00}; // receive bufer
  uint8_t tx_buffer[0x10] = {0x00}; // transmit buffer
  uint8_t rx_bytes = 0;             // number of bytes in receive buffer
  uint8_t tx_bytes = 0;             // number of bytes in transmit buffer
  uint32_t t_last_rx_byte = 0;      // timestamp of last byte received
};

#endif
