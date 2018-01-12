

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include <MemeServoAPI.h>


#define INTERFACE_TYPE_IIC       0
#define INTERFACE_TYPE_UART      1
#define INTERFACE_TYPE_SOFT_UART 2

#define INTERFACE_TYPE           INTERFACE_TYPE_UART

const uint8_t ADDRESS_MASTER = 0x01;
const uint8_t MAX_ID_TO_SCAN = 16;
const int eeprom_addr = 0;


// -------------------------------------------------------
// user functions

#if INTERFACE_TYPE == INTERFACE_TYPE_IIC

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int nb_received)
{
//  SerialToConsole.print("received: ");

  uint8_t data;

  for (int i=0; i<nb_received; i++)
  {
    data = (uint8_t)Wire.read();
    MMS_OnData(data);
  }
}


void sendDataI2C(uint8_t addr, uint8_t *data, uint8_t size)
{
  Wire.beginTransmission(addr);  // transmit to device
  Wire.write(data, size);
  Wire.endTransmission();        // stop transmitting
}

#else

#if (INTERFACE_TYPE == INTERFACE_TYPE_SOFT_UART)
#include <SoftwareSerial.h>
#define RX_PIN 8
#define TX_PIN 9
SoftwareSerial SoftSerial(RX_PIN, TX_PIN);
#define SerialToDevice SoftSerial
#else
#define RX_PIN 0
#define TX_PIN 1
#define SerialToDevice Serial1
#endif

//#define SerialToConsole Serial


void recvDataUART()
{
  while (SerialToDevice.available() > 0)
  {
    uint8_t data = (uint8_t)SerialToDevice.read();
    MMS_OnData(data);
  }
}

void sendDataUART(uint8_t addr, uint8_t *data, uint8_t size)
{
  while (size-- > 0)
    SerialToDevice.write(*data++);
}

#endif


void errorHandler(uint8_t node_addr, uint8_t errno)
{
#ifdef SerialToConsole
  SerialToConsole.print("NODE: 0x");
  SerialToConsole.print(node_addr, HEX);
  SerialToConsole.print(", ERROR: 0x");
  SerialToConsole.println(errno, HEX);
#endif
}


const uint8_t led = 13;
const uint8_t keyLearn = 2;
const uint8_t keyRun = 3;


enum STATUS
{
  STATUS_INIT,
  STATUS_LEARNING,
  STATUS_RUNNING
};

STATUS status;


struct POSITION_INFO
{
  uint8_t servo_cnt;
  uint8_t servo_ids[8];
  uint8_t step_count;
  int32_t positions[250];
} position_info;

uint8_t curr_step;


volatile bool keyLearnPressed;
volatile bool keyRunPressed;


void keyLearnPressedCallback()
{
  keyLearnPressed = true;
}


void keyRunPressedCallback()
{
  keyRunPressed = true;
}

void blinkLed(uint8_t times)
{
  uint8_t i;
  
  for (i=0; i<times; i++)
  {
    digitalWrite(led, 0);
    delay(100);
    digitalWrite(led, 1);
    delay(100);
  }
}

void scanServos(uint8_t servo_ids[], uint8_t &servo_cnt)
{
  uint8_t i;
  uint8_t node_id;
  uint8_t errno;
  uint8_t max_servo_cnt = servo_cnt;

  servo_cnt = 0;

  for (node_id=0x02; node_id<=MAX_ID_TO_SCAN; node_id++)
  {
#ifdef SerialToConsole
    SerialToConsole.print(F("Trying servo: 0x"));
    SerialToConsole.print(node_id, HEX);
#endif

    uint8_t retry_times = 3;
    while (retry_times-- > 0)
    {
#ifdef SerialToConsole
      SerialToConsole.print(F("."));
#endif

      if ((errno = MMS_ResetError(node_id, errorHandler)) != MMS_RESP_TIMEOUT)
        break;
    }

    if (errno == MMS_RESP_TIMEOUT)
    {
#ifdef SerialToConsole
      SerialToConsole.println();
#endif
    }
    else
    {
/*
      do
      {
#ifdef SerialToConsole
        SerialToConsole.print(F("."));
#endif
        errno = MMS_SetZeroPosition(node_id, errorHandler);
      } while (errno != MMS_RESP_SUCCESS);
*/
#ifdef SerialToConsole
      SerialToConsole.print(F(" Found servo: 0x"));
      SerialToConsole.println(node_id, HEX);
#endif
      servo_ids[servo_cnt++] = node_id;
    }

    if (servo_cnt >= max_servo_cnt) break;
  }

#ifdef SerialToConsole
  SerialToConsole.print(F("Total servos found: ")); SerialToConsole.println(servo_cnt);
#endif
}



void setup()
{
#ifdef SerialToConsole
  SerialToConsole.begin(115200);           // start serial for output
  while (!SerialToConsole);
  SerialToConsole.println(("Start."));
#endif

  pinMode(led, OUTPUT);
  blinkLed(5);

#if (INTERFACE_TYPE == INTERFACE_TYPE_IIC)

  Wire.begin(ADDRESS_MASTER);     // join i2c bus
  Wire.onReceive(receiveEvent);   // register event
  MMS_SetProtocol(MMS_PROTOCOL_I2C, ADDRESS_MASTER, sendDataI2C);

#else

  pinMode(RX_PIN, INPUT_PULLUP);

#if (INTERFACE_TYPE == INTERFACE_TYPE_UART)
  SerialToDevice.begin(115200);
#else
  SerialToDevice.begin(115200);
  SerialToDevice.listen();
#endif

  MMS_SetProtocol(MMS_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART);  // For non-interrupt receive mode, specify receive function.

#endif

  pinMode(keyLearn, INPUT_PULLUP);
  pinMode(keyRun, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(keyLearn), keyLearnPressedCallback, LOW);
  attachInterrupt(digitalPinToInterrupt(keyRun), keyRunPressedCallback, LOW);

  MMS_SetCommandTimeOut(150);
  MMS_SetTimerFunction(millis, delay);

  position_info.servo_cnt = 0;
  position_info.step_count = 0;

  keyLearnPressed = false;
  keyRunPressed = false;

  status = STATUS_INIT;
  curr_step = 0;

  // Load from eeprom
  EEPROM.get(eeprom_addr, position_info);
  
  uint8_t servo_ids[8];
  uint8_t len = sizeof(servo_ids);
  scanServos(servo_ids, len);
  
  if (position_info.servo_cnt != len ||
      memcmp(position_info.servo_ids, servo_ids, len * sizeof(servo_ids[0]) != 0))
  {
#ifdef SerialToConsole
    SerialToConsole.println(F("Servos found are not the same as stored."));
#else
    blinkLed(5);
#endif

    position_info.servo_cnt = 0;
    position_info.step_count = 0;
  }
}


void loop()
{
  uint8_t errno;

  if (keyLearnPressed)
  {
    if (status != STATUS_LEARNING)
    {
      uint8_t i;

      // Scan servos first
      position_info.servo_cnt = sizeof(position_info.servo_ids);
      scanServos(position_info.servo_ids, position_info.servo_cnt);

      if (position_info.servo_cnt == 0)
      {
#ifdef SerialToConsole
        SerialToConsole.println(F("No servo found."));
#else
        blinkLed(5);
#endif
        while(true);  // halt
      }


      //
      // Learn

#ifdef SerialToConsole
      SerialToConsole.println(F("Start learning mode."));
#else
      blinkLed(3);
#endif

      status = STATUS_LEARNING;
      position_info.step_count = 0;

      // All servos got to zero
      for (i=0; i<position_info.servo_cnt; i++)
      {
        uint8_t servo_addr = position_info.servo_ids[i];

        do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_addr, 65535, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_SetTorqueLimit returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS);

        do
        {
          delay(100);
          errno = MMS_StartServo(servo_addr, MMS_MODE_ZERO, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_StartServo returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS);
      }

      // Wait servos in position
      for (i=0; i<position_info.servo_cnt; i++)
      {
        uint8_t servo_addr = position_info.servo_ids[i];
        uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
        uint8_t in_position = 0;

        do
        {
          delay(100);
          errno = MMS_GetControlStatus(servo_addr, &ctrl_status, &in_position, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_GetControlStatus returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS || ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position != 1);
      }

      // Enter learnin mode
      for (i=0; i<position_info.servo_cnt; i++)
      {
        uint8_t servo_addr = position_info.servo_ids[i];

        do
        {
          delay(100);
          errno = MMS_StartServo(servo_addr, MMS_MODE_LEARNING, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_StartServo returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS);

        do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_addr, 500, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_SetTorqueLimit returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS);
      }
    }
    else
    {
      if (sizeof(position_info.positions) / sizeof(position_info.positions[0]) - position_info.step_count * position_info.servo_cnt >= position_info.servo_cnt)
      {
        // We have enough space

        uint8_t i;

        // Save position for all servos
        for (i=0; i<position_info.servo_cnt; i++)
        {
          uint8_t servo_addr = position_info.servo_ids[i];
          int32_t pos;

          do
          {
            delay(100);
            errno = MMS_GetAbsolutePosition(servo_addr, &pos, errorHandler);
            if (errno != MMS_RESP_SUCCESS)
            {
#ifdef SerialToConsole
              SerialToConsole.print(F("MMS_GetAbsolutePosition returned: 0x"));
              SerialToConsole.print(errno, HEX);
              SerialToConsole.print(F(", Node: 0x"));
              SerialToConsole.println(servo_addr, HEX);
#else
              blinkLed(5);
#endif
            }
          } while (errno != MMS_RESP_SUCCESS);

          position_info.positions[position_info.step_count * position_info.servo_cnt + i] = pos;
        }

#ifdef SerialToConsole
        SerialToConsole.print(F("Frame "));
        SerialToConsole.print(position_info.step_count);
        SerialToConsole.print(F(": "));

        for (i=0; i<position_info.servo_cnt; i++)
        {
            SerialToConsole.print(position_info.positions[position_info.step_count * position_info.servo_cnt + i]);
            SerialToConsole.print(F(","));
        }
        SerialToConsole.println(F("#"));
#endif

        position_info.step_count++;
      }
    }

    keyLearnPressed = false;
  }

  if (keyRunPressed)
  {
    if (status != STATUS_RUNNING && position_info.step_count > 0)
    {
      uint8_t i;

      // Save steps first
      EEPROM.put(eeprom_addr, position_info);


#ifdef SerialToConsole
      SerialToConsole.println(F("Start running mode."));
#endif

      status = STATUS_RUNNING;
      curr_step = 0;

      // All servos got to zero
      for (i=0; i<position_info.servo_cnt; i++)
      {
        uint8_t servo_addr = position_info.servo_ids[i];

        do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_addr, 65535, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_SetTorqueLimit returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS);

        do
        {
          delay(100);
          errno = MMS_StartServo(servo_addr, MMS_MODE_ZERO, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_StartServo returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS);
      }

      // Wait servos in position
      for (i=0; i<position_info.servo_cnt; i++)
      {
        uint8_t servo_addr = position_info.servo_ids[i];
        uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
        uint8_t in_position = 0;

        do
        {
          delay(100);
          errno = MMS_GetControlStatus(servo_addr, &ctrl_status, &in_position, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
#ifdef SerialToConsole
            SerialToConsole.print(F("MMS_GetControlStatus returned: 0x"));
            SerialToConsole.print(errno, HEX);
            SerialToConsole.print(F(", Node: 0x"));
            SerialToConsole.println(servo_addr, HEX);
#else
            blinkLed(5);
#endif
          }
        } while (errno != MMS_RESP_SUCCESS || ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position != 1);
      }
    }

    keyRunPressed  = false;
  }

  if (status == STATUS_RUNNING)
  {
    uint8_t i;

    // Wait servos in position
    for (i=0; i<position_info.servo_cnt; i++)
    {
      uint8_t servo_addr = position_info.servo_ids[i];
      uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
      uint8_t in_position = 0;

      do
      {
        delay(100);
        errno = MMS_GetControlStatus(servo_addr, &ctrl_status, &in_position, errorHandler);
        if (errno != MMS_RESP_SUCCESS)
        {
#ifdef SerialToConsole
          SerialToConsole.print(F("MMS_GetControlStatus returned: 0x"));
          SerialToConsole.print(errno, HEX);
          SerialToConsole.print(F(", Node: 0x"));
          SerialToConsole.println(servo_addr, HEX);
#else
          blinkLed(5);
#endif
        }
      } while (errno != MMS_RESP_SUCCESS || ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || in_position != 1);
    }

#ifdef SerialToConsole
    SerialToConsole.print(F("curr_step: "));
    SerialToConsole.print(curr_step);
    SerialToConsole.print(F("/"));
    SerialToConsole.print(position_info.step_count - 1);
    SerialToConsole.print(F(", Frame: "));

    for (i=0; i<position_info.servo_cnt; i++)
    {
      SerialToConsole.print(position_info.positions[curr_step * position_info.servo_cnt + i]);
      SerialToConsole.print(F(","));
    }
    SerialToConsole.println(F("#"));
#endif

    // Move to frame
    for (i=0; i<position_info.servo_cnt; i++)
    {
      uint8_t servo_addr = position_info.servo_ids[i];
      int32_t pos = position_info.positions[curr_step * position_info.servo_cnt + i];

      do
      {
        delay(100);
        errno = MMS_ProfiledAbsolutePositionMove(servo_addr, pos, errorHandler);
        if (errno != MMS_RESP_SUCCESS)
        {
#ifdef SerialToConsole
          SerialToConsole.print(F("MMS_ProfiledAbsolutePositionMove returned: 0x"));
          SerialToConsole.print(errno, HEX);
          SerialToConsole.print(F(", Node: 0x"));
          SerialToConsole.println(servo_addr, HEX);
#else
          blinkLed(5);
#endif
        }
      } while (errno != MMS_RESP_SUCCESS);
    }

    if (++curr_step >= position_info.step_count)
    {
      curr_step = 0;
    }
  }
}

