#include <Arduino.h>
#include "STM32_CAN.h"
#include <IbusTrx.h>

// Инициализация классов:
IbusTrx ibus;
STM32_CAN can(CAN1, DEF, RX_SIZE_64, TX_SIZE_64); // PA11 - CRX / PA12 - CTX

// Определение функций и переменных:
static CAN_message_t canRxMessage;
static CAN_message_t canTxMessage;
void resetCanMessage();
void canRead();
void ibusSniffer();
void ibusFiltered();
void initTimers();

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
  // ibusSniffer();
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

void ibusSniffer()
{
  if (ibus.available())
  {
    IbusMessage ibusRxMessage = ibus.readMessage();

    Serial1.print("Source: ");
    Serial1.print("0x");
    Serial1.print(ibusRxMessage.source(), HEX);
    Serial1.print(" Length: ");
    Serial1.print("0x");
    Serial1.print(ibusRxMessage.length(), HEX);
    Serial1.print(" Destination: ");
    Serial1.print("0x");
    Serial1.print(ibusRxMessage.destination(), HEX);
    Serial1.print(" Payload:");
    for (int i = 0; i < ibusRxMessage.length() - 1; i++)
    {
      Serial1.print(" 0x");
      Serial1.print(ibusRxMessage.b(i), HEX);
    }
    Serial1.print(" CRC");
    Serial1.println();
  }
}

void ibusFiltered()
{
  if (ibus.available())
  {
    IbusMessage ibusRxMessage = ibus.readMessage();

    if (ibusRxMessage.b(0) == 0x7a)
    {
      Serial1.println("Doors/LID Message");
      Serial1.print("Length: ");
      Serial1.print(ibusRxMessage.length());
      Serial1.print(" DATA:");
      for (int i = 0; i < ibusRxMessage.length() - 1; i++)
      {
        Serial1.print(" 0x");
        Serial1.print(ibusRxMessage.b(i), HEX);
      }
      Serial1.println();
    }
    else if (ibusRxMessage.b(0) == 0x19)
    {
      Serial1.println("Temperature Message");
      Serial1.print("Length: ");
      Serial1.print(ibusRxMessage.length());
      Serial1.print(" DATA:");
      for (int i = 0; i < ibusRxMessage.length() - 1; i++)
      {
        Serial1.print(" 0x");
        Serial1.print(ibusRxMessage.b(i), HEX);
      }
      Serial1.println();
    }
    else if (ibusRxMessage.b(0) == 0x11)
    {
      Serial1.println("Ignition Message");
      Serial1.print("Length: ");
      Serial1.print(ibusRxMessage.length());
      Serial1.print(" DATA:");
      for (int i = 0; i < ibusRxMessage.length() - 1; i++)
      {
        Serial1.print(" 0x");
        Serial1.print(ibusRxMessage.b(i), HEX);
      }
      Serial1.println();
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