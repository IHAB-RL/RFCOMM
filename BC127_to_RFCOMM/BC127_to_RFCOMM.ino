//sudo stty -F /dev/rfcomm26 921600 cs8 -icrnl -ixon -ixoff -opost -isig -icanon -echo ;

//sudo stty -F /dev/rfcomm1 921600 -icrnl -ixon -ixoff -opost -isig -icanon -echo;
//sudo cat /dev/rfcomm1 |  aplay -f S16_LE -c 2 -r 22050

#define DEBUG
#define BITS          32 // 16 or 32
#define FS            16000
#define BLOCK_SIZE    32 // in samples for one channel!!! (max value: 64 (for 32 Bit), 128 (for 16 Bit)

#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SparkFunbc127.h>
#include <I2S.h>
#include "StRingBuffer.h"
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

#define serialPort  Serial2
#ifdef DEBUG
#define debugPort  Serial
bool LED_on = HIGH;
#else
bool LED_on = LOW;
#endif
int ledRed = 21;
int ledGreen = 5;
int ledBlue = 6;
int resetPin = 20;

bool sendData = false;
long int Baud = 921600;
BC127 BTModu(&serialPort);
StRingBuffer serialIn = StRingBuffer(10);

int16_t volume = 0;
bool adaptiveBitShift = false;
const int additionalInt16Values = 3;
byte sample_buffer_receive[BLOCK_SIZE * BITS / 4];
byte sample_buffer_send[BLOCK_SIZE * 4 + (additionalInt16Values * 2)];
int16_t *ptrSend;
#if BITS == 32
int32_t *ptrReceive;
#else
int16_t *ptrReceive;
#endif

void setup()
{
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(resetPin, OUTPUT);
  String value;
#ifdef DEBUG
  debugPort.begin(9600);
#endif
  red();
  delay(250);
  if (!I2S.begin(I2S_PHILIPS_MODE, FS, BITS)) {
    debug("Failed to initialize I2S!");
    while (1); // do nothing
  }
  while (!sendData)
  {
    value = "";
    restoreBC127();
    serialPort.begin(9600);
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
    //while (!serialPort) {;}
    BTModu.stdGetParam("UART_CONFIG", &value);
    debug("GET UART_CONFIG -> " + value);
    String UART_CONFIG = String(Baud) + " OFF 0";
    debug("SET UART_CONFIG = " + UART_CONFIG);
    BTModu.stdSetParam("UART_CONFIG", UART_CONFIG);
    serialPort.flush();
    serialPort.end();
    serialPort.begin(Baud);
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
    while (!serialPort) {
      ;
    }
    BTModu.stdSetParam("PROFILES", "0 0 0 0 0 0 1 0 0 0 0 0");
    BTModu.writeConfig();
    BTModu.reset();
    delay(500);
    BTModu.stdGetParam("UART_CONFIG", &value);
    debug("GET UART_CONFIG -> " + value);
    if (value == UART_CONFIG)
    {
      BTModu.stdSetParam("HIGH_SPEED", "ON OFF");
      delay(200);
      waitForSPP();
    }
  }
}

void loop()
{
  if (I2S.available() >= BLOCK_SIZE * BITS / 4)
  {
    I2S.read(sample_buffer_receive, BLOCK_SIZE * BITS / 4);
    if (serialPort.available())
    {
      String value = serialIn.addChar(serialPort.read());
      if (value.substring(value.length() - 5) == "ERROR")
      {
        sendData = false;
        debugPort.println("Connection lost...");
        waitForSPP();
      }
#if BITS == 32
      else if (value.charAt(value.length() - 3) == 'V' &&  isDigit(value.charAt(value.length() - 1)))
      {
        int16_t newVol = int16_t(value.charAt(value.length() - 1) - '0');
        if (value.charAt(value.length() - 2) == '+')
          volume = newVol;
        else if (value.charAt(value.length() - 2) == '-')
          volume = -newVol;
        debug("Volume = " + String(volume));
      }
      else if (value.charAt(value.length() - 2) == 'B' &&  isDigit(value.charAt(value.length() - 1)))
      {
        adaptiveBitShift = bool(value.charAt(value.length() - 1) - '0');
        debug("Adaptive Bit Shift = " + String(adaptiveBitShift));
        if (adaptiveBitShift == false)
          volume = 0;
      }
#endif
    }
    ptrSend = (int16_t *)sample_buffer_send;
#if BITS == 32
    ptrReceive = (int32_t *)sample_buffer_receive;
    if (adaptiveBitShift)
    {
      int maxVal = 1;
      for (int count = 0; count < BLOCK_SIZE * 2; count++)
        if (ptrReceive[count] != 0)
          maxVal = max(maxVal, abs(ptrReceive[count]));
      volume = 30 - (int)log2(maxVal);
    }
    int tmpVol = min(max(0, 16 - volume), 31);
    for (int count = 0; count < BLOCK_SIZE * 2; count++)
      ptrSend[count + 1] = int16_t(ptrReceive[count] >> tmpVol);
#else
    ptrReceive = (int16_t *)sample_buffer_receive;
    for (int count = 0; count < BLOCK_SIZE * 2; count++)
      ptrSend[count + 1] = ptrReceive[count];
#endif
    sample_buffer_send[0] = 0x7F;
    sample_buffer_send[1] = 0xFF;
    ptrSend[BLOCK_SIZE * 2 + 1] = volume;
    sample_buffer_send[BLOCK_SIZE * 4 + 4] = 0x80;
    sample_buffer_send[BLOCK_SIZE * 4 + 5] = 0x00;
    serialPort.write(sample_buffer_send, BLOCK_SIZE * 4 + (additionalInt16Values * 2));
  }
}

void waitForSPP()
{
  //I2S.end();
  String value;
  blue();
  debug("waiting for SPP...");
  while (sendData == false)
  {
    value = "";
    BTModu.stdGetCommand("STATUS", "SPP", &value);
    if (value.indexOf("CONNECTED SPP") >= 0)
    {
      debug("SPP connected!");
      debug("ENTER_DATA_MODE 15...");
      BTModu.stdCmd("ENTER_DATA_MODE 15");
      delay(1000);
      serialPort.flush();
      while (serialPort.available())
        serialPort.read();
      serialIn.clear();
      sendData = true;
    }
    else
      delay(100);
  }
  green();
  //I2S.begin(I2S_PHILIPS_MODE, FS, BITS);
}

void red()
{
  LedsOff();
  digitalWrite(ledRed, LED_on);
}

void blue()
{
  LedsOff();
  digitalWrite(ledBlue, LED_on);
}

void green()
{
  LedsOff();
  digitalWrite(ledGreen, LED_on);
}

void LedsOff()
{
  digitalWrite(ledBlue, !LED_on);
  digitalWrite(ledRed, !LED_on);
  digitalWrite(ledGreen, !LED_on);
}

void debug(String strMessage)
{
#ifdef DEBUG
  debugPort.println(strMessage);
  delay(100);
#endif
}

void restoreBC127 ()
{
  debug("reset and restore...");
  digitalWrite(resetPin, LOW);
  delay(50);
  digitalWrite(resetPin, HIGH);
  delay(200);
  long Baudrates[] = {9600, 115200, 460800, 921600};
  for (int i = 0; i < 4; i++)
  {
    if (serialPort)
      serialPort.end();
    serialPort.begin(Baudrates[i]);
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
    while (!serialPort) {}
    BTModu.restore();
    delay(200);
  }
  delay(200);
  if (serialPort)
    serialPort.end();
  serialIn.addChar(' ');
  delay(200);
}


