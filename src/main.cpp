#include <Arduino.h>
#include "CANbus.h"
#include <Ibus.h>

// Инициализация классов:
Ibus ibus;
CANbus can(CAN1, DEF, RX_SIZE_64, TX_SIZE_64); // PA11 - CRX / PA12 - CTX

// Определение функций и переменных:
static CAN_message_t canRxMessage;
static CAN_message_t canTxMessage;
void resetCanMessage();
void canRead();
void ibusFiltered();
void initTimers();

char output[50] = {};

void setup()
{
  // DEBUG
  Serial1.begin(115200); // PA9 - TX / PA10 - RX

  // IBUS
  Serial3.begin(9600);
  ibus.begin(Serial3); // PB10 - IbusTX / PB11 - IbusRX

  // CANBUS
  can.begin(true);
  can.setBaudRate(500000); // 500KBPS
  // Фильтры
  // can.setMBFilterProcessing(MB0, 0x101u, 0x1FFu, STD);

  // Таймеры
  initTimers();
}

void loop()
{
  // Рабочий цикл:
  ibusFiltered();
}

// Реализация функций:
void resetCanMessage()
{
  canTxMessage.id = (0x00000000);
  canTxMessage.flags.extended = 1;
  canTxMessage.len = 8;
  canTxMessage.buf[0] = 0x00;
  canTxMessage.buf[1] = 0x00;
  canTxMessage.buf[3] = 0x00;
  canTxMessage.buf[4] = 0x00;
  canTxMessage.buf[5] = 0x00;
  canTxMessage.buf[6] = 0x00;
  canTxMessage.buf[7] = 0x00;
}

void canRead()
{
  if (can.read(canRxMessage))
  {
    Serial1.print("Channel:");
    Serial1.print(canRxMessage.bus);
    if (canRxMessage.flags.extended == false)
    {
      Serial1.print(" Standard ID:");
    }
    else
    {
      Serial1.print(" Extended ID:");
    }
    Serial1.print(canRxMessage.id, HEX);

    Serial1.print(" DLC: ");
    Serial1.print(canRxMessage.len);
    if (canRxMessage.flags.remote == false)
    {
      Serial1.print(" buf: ");
      for (int i = 0; i < canRxMessage.len; i++)
      {
        Serial1.print(" 0x");
        Serial1.print(canRxMessage.buf[i], HEX);
        if (i != (canRxMessage.len - 1))
          Serial1.print(" ");
      }
      Serial1.println();
    }
    else
    {
      Serial1.println(" Data: REMOTE REQUEST FRAME");
    }
  }
}

void ibusFiltered()
{
  if (ibus.available())
  {
    IbusMessage ibusRxMessage = ibus.readMessage();

    // 0x11 Ignition
    // 0x13 Sensors

    if (ibusRxMessage.b(0) == 0x13)
    {
      sprintf(output, "gear: 0x%x", ibusRxMessage.b(2));
      Serial1.println(output);
    }
  }
}

void initTimers()
{
  // #if defined(TIM1)
  //   TIM_TypeDef *Instance = TIM1;
  // #else
  //   TIM_TypeDef *Instance = TIM2;
  // #endif
  //   HardwareTimer *SendTimer = new HardwareTimer(Instance);
  //   SendTimer->setOverflow(50, HERTZ_FORMAT); // 50 Hz
  //   SendTimer->attachInterrupt(Callback);
  //   SendTimer->resume();
}