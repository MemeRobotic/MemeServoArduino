/*
@作者：何斌，何杭军
@最后更新日期：2018年1月12日
@功能：本程序实现基于谜米的数字舵机的动作学习和运行功能。需要谜米的数字舵机支持
@版权：谜米机器人自动化（上海）有限公司
@说明：
	本程序源代码开发，如果有问题可以咨询stephenhe@memerobotics.com
	商务合作，请联系stephenhe@memerobotics.com
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include <MemeServoAPI.h>


#define INTERFACE_TYPE_IIC       0
#define INTERFACE_TYPE_UART      1
#define INTERFACE_TYPE_SOFT_UART 2

#define INTERFACE_TYPE           INTERFACE_TYPE_UART

const uint8_t ADDRESS_MASTER = 0x01;
const uint8_t MAX_ID_TO_SCAN = 16; //最多扫描到16号，因此最多支持的舵机数量是15个
const uint8_t Const_LEDFlashTimesError=5;//出错的时候，信号灯的闪烁次数
const uint8_t Const_LEDFlashTimesWorking=3;//正常工作的时候，信号灯的闪烁次数
const uint8_t Const_IntervalError=50;//出错的时候，信号灯闪烁的间隔
const uint8_t Const_IntervalWorking=100;//正常的时候，信号灯闪烁的间隔
const int eeprom_addr = 0;//EEPROM的开始地址
const uint8_t Const_RetryTimes=3; //舵机尝试连接的次数
const uint8_t Const_ErrorSignal=0; //异常信号，信号灯要按照异常方式闪烁
const uint8_t Const_WorkingSignal=1;//正常信号，信号灯要按照正常方式闪烁
const uint8_t Const_OtherSignal=-1; //其他信号，信号灯不用闪烁
// -------------------------------------------------------
// user functions

#if INTERFACE_TYPE == INTERFACE_TYPE_IIC

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
/*
@功能：IIC下接收数据事件
@说明：将接收到的数据发送给API的MMS_OnData处理,MMS_OnData实现舵机的协议
@参数：接收到的字节数
@返回值：无
*/
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

/*
@功能：IIC下发送数据
@说明：通过串口发送给舵机
@参数：addr发送地址，data发送数据，size数据的大小
@返回值：无
*/
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

//#define SerialToConsole Serial //only for console available, need to be comment off if running ardunio alone


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
		while (size-- > 0)SerialToDevice.write(*data++);
	}

#endif

/*
@功能：上位机的错误处理程序
@说明：本函数仅仅在控制台打印返回的错误信息，包括出错的节点和错误编码，用户可以自定义更加有针对性的错误处理方式，
@参数：node_addr出错的舵机地址，errno错误编码，请参考舵机控制API
@返回值：无
*/
void errorHandler(uint8_t node_addr, uint8_t errno)
{
	#ifdef SerialToConsole
	  SerialToConsole.print("NODE: 0x");
	  SerialToConsole.print(node_addr, HEX);
	  SerialToConsole.print(", ERROR: 0x");
	  SerialToConsole.println(errno, HEX);
	#endif
}


const uint8_t led = 13; //信号灯的接口位置
const uint8_t keyLearn = 2;//学习按键的接口位置
const uint8_t keyRun = 3;//运行按键的接口位置


enum STATUS
{
  STATUS_INIT,
  STATUS_LEARNING,
  STATUS_RUNNING,
  STATUS_NOSERVOS
};

STATUS status;

//Position_Info是学习以后的所有动作组的结构体，这个结构体会保存到EEPROM里面，避免每一次都要学习以后才能运行动作。
//注意EEPROM只能保存学习的一组动作，没有多组动作保存的功能
struct POSITION_INFO
{
  uint8_t servo_cnt;
  uint8_t servo_ids[8];
  uint8_t step_count;
  int32_t positions[250];
} position_info;

uint8_t curr_step; //当前运行或者学习的步数


volatile bool keyLearnPressed;
//状态控制，学习键按下的时候为True，按钮按下的时候，会通过中断触发CallBack函数，
//将该变量赋值为True。主循环检测到该变量的值，进行对应的逻辑处理。
//这个变量需要加锁，防止被中断和主程序同时修改。

volatile bool keyRunPressed;
//运行控制，运行键按下的时候为True，按钮按下的时候，会通过中断触发CallBack函数，
//将该变量赋值为True。主循环检测到该变量的值，进行对应的逻辑处理。
//这个变量需要加锁，防止被中断和主程序同时修改。

//CallBack函数，按键的时候要用attachInterrupt触发这个函数。
void keyLearnPressedCallback()
{
  keyLearnPressed = true;
}

//CallBack函数，按键的时候要用attachInterrupt触发这个函数。
void keyRunPressedCallback()
{
  keyRunPressed = true;
}

/*
@功能：LED灯闪烁
@说明：根据参数确定LED灯的闪烁次数和时间
@参数：LEDFlashTimes闪烁次数，LEDFlashInterval闪烁间隔
@返回值：无
*/
void blinkLed(uint8_t LEDFlashTimes,uint8_t LEDFlashInterval)
{
  uint8_t i;
  
  for (i=0; i<LEDFlashTimes; i++)
  {
    digitalWrite(led, 0);
    delay(LEDFlashInterval);
    digitalWrite(led, 1);
    delay(LEDFlashInterval);
  }
}

/*
@功能：在控制台打印信息并闪烁信号灯
@说明：根据参数确定LED灯的闪烁闪烁方式
@参数：messageStr需要在控制台打印的信息，Status_Signal闪烁方式，如果1表示工作正常的闪烁方式，如果是0表示工作异常的闪烁方式，如果其他值，则不闪烁
@返回值：无
*/

void FeedbackMessage(String messageStr,uint8_t Status_Signal)
{
	#ifdef SerialToConsole
	SerialToConsole.println(messageStr);
	#else
	if (Status_Signal==1)blinkLed(Const_LEDFlashTimesWorking,Const_IntervalWorking);
	else
		if(Status_Signal==0) blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
	#endif
}


/*
@功能：尝试连接舵机，如果成功返回True，否则返回False
@参数：
	RetryTimes，尝试连接的次数，超过该次数连接未成功，则表示不能连接该舵机
	node_id，尝试连接的舵机ID
@说明：
@返回值:True表示连接成功
*/

bool ScanOneServo(uint8_t RetryTimes, uint8_t node_id)
{
    uint8_t retry_times = RetryTimes;
    uint8_t errno;
    while (retry_times-- > 0)
    {
		#ifdef SerialToConsole
			SerialToConsole.print(F("."));
		#endif

      if ((errno = MMS_ResetError(node_id, errorHandler)) != MMS_RESP_TIMEOUT)
		  //用重置错误函数确认舵机是否可以连接，没有超时，则表示该ID有效，退出循环，否则超时的情况下，连续试3次
        break;
    }

    if (errno == MMS_RESP_TIMEOUT)//成功检测到舵机跳出循环或者超时退出讯函，因此需要判断是否超时退出循环
    {
		//超时，如果有控制台，打印出错信息
		#ifdef SerialToConsole
			SerialToConsole.println();
		#endif
		return false;
    }
    else //检测到舵机
    {

		//如果有控制台，在控制台显示扫描到的舵机ID
		#ifdef SerialToConsole
			  SerialToConsole.print(F(" Found servo: 0x"));
			  SerialToConsole.println(node_id, HEX);
		#endif
		return true; 
    }
	
}
/*
@功能：用重置错误的方法扫描串口的舵机，扫描到的舵机的ID返回到指定的数组中，并将扫描到的舵机数量返回到指定的变量中
@参数：
	舵机ID的列表，传入数组地址，如果扫描到舵机，则将舵机的ID依次存入数组
	舵机的数量，传入的是变量的引用，因此在程序内被修改，在传入的时候，作为舵机数量的上限。如果没有扫到舵机，置0，否则就是实际扫描到的舵机数量
@说明：
@返回值:无
*/
void scanServos(uint8_t servo_ids[], uint8_t &servo_cnt)
{
  uint8_t i;
  uint8_t node_id;
  uint8_t errno;
  uint8_t max_servo_cnt = servo_cnt;//最多可能的舵机数量

  servo_cnt = 0;//引用调用，外部变量在函数内被修改！！！

  for (node_id=0x02; node_id<=MAX_ID_TO_SCAN; node_id++)
  {
	#ifdef SerialToConsole
		SerialToConsole.print(F("Trying servo: 0x"));
		SerialToConsole.print(node_id, HEX);
	#endif

   //扫描到的舵机存入舵机列表数组
	if (ScanOneServo(Const_RetryTimes,node_id))	servo_ids[servo_cnt++] = node_id;

    if (servo_cnt >= max_servo_cnt) break; //如果超过设定的舵机上限，停止扫描
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
		blinkLed(Const_IntervalWorking,Const_LEDFlashTimesWorking);//Ready

	#if (INTERFACE_TYPE == INTERFACE_TYPE_IIC)

		Wire.begin(ADDRESS_MASTER);     // join i2c bus
		Wire.onReceive(receiveEvent);   // register event
		MMS_SetProtocol(MMS_PROTOCOL_I2C, ADDRESS_MASTER, sendDataI2C);

	#else
		pinMode(RX_PIN, INPUT_PULLUP);
		/* Original Code By Hebin
		#if (INTERFACE_TYPE == INTERFACE_TYPE_UART)
		SerialToDevice.begin(115200);
		#else
		SerialToDevice.begin(115200);
		SerialToDevice.listen();
		#endif
		MMS_SetProtocol(MMS_PROTOCOL_UART, ADDRESS_MASTER, sendDataUART, recvDataUART);  // For non-interrupt receive mode, specify receive function.
		*/
		SerialToDevice.begin(115200);	
		#if (INTERFACE_TYPE == INTERFACE_TYPE_SOFT_UART)
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

	keyLearnPressed = false;
	keyRunPressed = false;
	status = STATUS_INIT;
	curr_step = 0;
	InitPositionInfo(); 
}

/*
@功能：初始化结构体position_info
@参数：无
@说明：从EEPROM中复制之前存储的数据进入到position_info的结构体中，扫描现在的舵机，比较之前存储的信息和现在扫描的是否一致，如果数量一致并且ID一致，则认为EEPROM存储了和现在连接的舵机组匹配的动作组，否则清空position_info，等待学习新的动作。
@返回值:无
*/

void InitPositionInfo()
{
	position_info.servo_cnt = 0;
	position_info.step_count = 0; 
	EEPROM.get(eeprom_addr, position_info); // Load from eeprom into positon_info

	uint8_t servo_ids[8];
	uint8_t len = sizeof(servo_ids);
	scanServos(servo_ids, len);//scan servo info into len and servo_ids

	if (position_info.servo_cnt != len ||memcmp(position_info.servo_ids, servo_ids, len * sizeof(servo_ids[0]) != 0)) 
	//比较扫描到的舵机是否与原来EEROM中的舵机信息是否一致。数量不一致或者ID不一致都属于信息不符合
	{
		#ifdef SerialToConsole
			SerialToConsole.println(F("Servos found are not the same as stored."));
		#else
			blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
		#endif
		//信息不一致的时候，结构体的数据清0，但是EEPROM没有变动
		position_info.servo_cnt = 0;
		position_info.step_count = 0;
	}
}

void loop()
{
  uint8_t errno;

  if (keyLearnPressed)//通过中断的方式改变这个变量
  {
	if (status != STATUS_LEARNING)//是否第一次按Learn按钮，如果是，则进入学习模式，否则记录所有舵机位置
		{
			/*
			1，重新检测所有舵机
			2，所有舵机启动到0位
			3，等待所有舵机就位
			4，将所有舵机改成学习模式
			5，将所有舵机改成学习模式的力矩限值
			*/
			uint8_t i;
			// Scan servos first
			position_info.servo_cnt = sizeof(position_info.servo_ids);//Max Servos,default value is 8, pls check the structure definition of position_info
			scanServos(position_info.servo_ids, position_info.servo_cnt);//both position_info.servo_ids and position_info.servo_cnt will be modified by the called function scanServos

			if (position_info.servo_cnt == 0)
			{
				FeedbackMessage("No servo found.",Const_ErrorSignal);
				//while(true);  // halt , if no servo scanned, fatal error, the system will halt，verify with hebin	
			}
			//
			// Learn
			else
			{
				FeedbackMessage("Start learning mode.",Const_WorkingSignal);
				status = STATUS_LEARNING;
				position_info.step_count = 0;

				// All servos got to zero
				for (i=0; i<position_info.servo_cnt; i++)
				{
					uint8_t servo_addr = position_info.servo_ids[i];
					SetTorqueLimit(servo_addr,65535);
					StartServo(servo_addr, MMS_MODE_ZERO);//Start at Zero
				}

				// Wait servos in position
				for (i=0; i<position_info.servo_cnt; i++)
				{
					uint8_t servo_addr = position_info.servo_ids[i];
					uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
					uint8_t in_position = 0;
					GetControlStatus(servo_addr,&ctrl_status,&in_position);        
				}

				// Enter learnin mode
				for (i=0; i<position_info.servo_cnt; i++)
				{
					uint8_t servo_addr = position_info.servo_ids[i];
					StartServo(servo_addr,MMS_MODE_LEARNING);
					SetTorqueLimit(servo_addr,500);       
				}	
				
			}	
		}
    else //不是第一次按下学习按钮，需要记录一帧的信息
		{
			RecordOneFrame();		
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
			SetTorqueLimit(servo_addr,65535);
			StartServo(servo_addr, MMS_MODE_ZERO);        
		}
		#ifdef SerialToConsole
			SerialToConsole.println(F("All servos got to zero."));
		#endif

		// Wait servos in position
		for (i=0; i<position_info.servo_cnt; i++)
		{
			uint8_t servo_addr = position_info.servo_ids[i];
			uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
			uint8_t in_position = 0;
			GetControlStatus(servo_addr, &ctrl_status, &in_position);        
		}
		#ifdef SerialToConsole
			SerialToConsole.println(F("All servos got in position."));
		#endif
    }

    keyRunPressed  = false;
  }

  if (status == STATUS_RUNNING)
  {
        // Wait servos in position
    uint8_t i;
    for (i=0; i<position_info.servo_cnt; i++)
    {
		uint8_t servo_addr = position_info.servo_ids[i];
		uint8_t ctrl_status = MMS_CTRL_STATUS_NO_CONTROL;
		uint8_t in_position = 0;
		GetControlStatus(servo_addr, &ctrl_status, &in_position);       
    } 
	MoveToFrame(curr_step);
    if (++curr_step >= position_info.step_count)
    {
		curr_step = 0;
    }
  }
}

/*
@功能：调用API设定指定ID的扭矩限值。
@参数：servo_addr 需要设定的舵机ID；TorqueLimit设定的扭矩，
@说明：正常运行设置为65535，学习模式设置为500
@返回值：无
*/
void SetTorqueLimit(uint8_t servo_addr,uint16_t TorqueLimit)
{
	uint8_t errno;
	do
        {
          delay(100);
          errno = MMS_SetTorqueLimit(servo_addr, TorqueLimit, errorHandler);
          if (errno != MMS_RESP_SUCCESS)
          {
			#ifdef SerialToConsole
				SerialToConsole.print(F("MMS_SetTorqueLimit returned: 0x"));
				SerialToConsole.print(errno, HEX);
				SerialToConsole.print(F(", Node: 0x"));
				SerialToConsole.println(servo_addr, HEX);
			#else
				blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
			#endif
          }
        } while (errno != MMS_RESP_SUCCESS);
	
}
/*
@功能：调用API启动舵机，
@参数：servo_addr 需要设定的舵机ID；mode启动模式
@说明：
@返回值：无
*/
void StartServo(uint8_t servo_addr,uint8_t mode)
{
	uint8_t errno;
	do
	{
		delay(100);
		errno = MMS_StartServo(servo_addr, mode, errorHandler);
		if (errno != MMS_RESP_SUCCESS)
		{
			#ifdef SerialToConsole
				SerialToConsole.print(F("MMS_StartServo returned: 0x"));
				SerialToConsole.print(errno, HEX);
				SerialToConsole.print(F(", Node: 0x"));
				SerialToConsole.println(servo_addr, HEX);
			#else
				blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
			#endif
		}
	} while (errno != MMS_RESP_SUCCESS);	
}
/*
@功能：调用API获得状态信息，确定舵机就位
@参数：servo_addr舵机ID，Status舵机状态，in_position是否就位）
@说明：
@返回值：无
*/
void GetControlStatus(uint8_t servo_addr, uint8_t *ctrl_status, uint8_t *in_position)
{
	uint8_t errno;
	do
	{
		delay(100);
		errno = MMS_GetControlStatus(servo_addr, ctrl_status, in_position, errorHandler);
		if (errno != MMS_RESP_SUCCESS)
		{
			#ifdef SerialToConsole
				SerialToConsole.print(F("MMS_GetControlStatus returned: 0x"));
				SerialToConsole.print(errno, HEX);
				SerialToConsole.print(F(", Node: 0x"));
				SerialToConsole.println(servo_addr, HEX);
			#else
				blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
			#endif
		}
	} while (errno != MMS_RESP_SUCCESS || *ctrl_status != MMS_CTRL_STATUS_POSITION_CONTROL || *in_position != 1);
}
/*
@功能：调用API获得位置信息，用于记录学习到的帧
@参数：servo_addr舵机ID，pos位置信息的引用调用，会在函数内被修改）
@说明：
@返回值：无
*/

void GetAbsolutePosition(uint8_t servo_addr, int32_t *pos)
//Pay attention to *pos,need to verify with Hebin
{
	uint8_t errno;
	do
	{
		delay(100);
		errno = MMS_GetAbsolutePosition(servo_addr, pos, errorHandler);
		if (errno != MMS_RESP_SUCCESS)
		{
			#ifdef SerialToConsole
				SerialToConsole.print(F("MMS_GetAbsolutePosition returned: 0x"));
				SerialToConsole.print(errno, HEX);
				SerialToConsole.print(F(", Node: 0x"));
				SerialToConsole.println(servo_addr, HEX);
			#else
				blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
			#endif
		}
	} while (errno != MMS_RESP_SUCCESS);	
}

/*
@功能：调用API将舵机运动到指定位置
@参数：servo_addr舵机ID，pos目标位置信息
@说明：运动的加速度和速度参数由舵机内部默认的加速度和速度确定
@返回值：无
*/

void ProfiledAbsolutePositionMove(uint8_t servo_addr, int32_t pos)
{
	uint8_t errno;
	do
	{
		delay(100);
		errno = MMS_ProfiledAbsolutePositionMove(servo_addr, pos, errorHandler); //第i个舵机运动到该帧指定位置
		if (errno != MMS_RESP_SUCCESS)
		{
			#ifdef SerialToConsole
				SerialToConsole.print(F("MMS_ProfiledAbsolutePositionMove returned: 0x"));
				SerialToConsole.print(errno, HEX);
				SerialToConsole.print(F(", Node: 0x"));
				SerialToConsole.println(servo_addr, HEX);
			#else
				blinkLed(Const_LEDFlashTimesError,Const_IntervalError);
			#endif
		}
	} while (errno != MMS_RESP_SUCCESS);
	
}

/*
@功能：记录特定步的所有信息
@参数：
@说明：在学习模式下，所有舵机的位置调整准确以后，按下学习按钮，这个函数会被调用，用以记录所有的舵机位置和步数
@返回值：无
*/

void RecordOneFrame()
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
				GetAbsolutePosition(servo_addr,&pos);    
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

/*
@功能：将所有舵机运行到curr_step指定的帧，帧信息存储在position_info结构体中
@参数：curr_step，即将运行到的帧数
@说明：如果有控制台，先在控制台打印当前要运行的帧的信息
@返回值：无
*/
void MoveToFrame(uint8_t curr_step)
{
	// Move to frame
	uint8_t i;
	#ifdef SerialToConsole //条件编译，有定义控制台输出的时候打印
		SerialToConsole.print(F("curr_step: "));
		SerialToConsole.print(curr_step);
		SerialToConsole.print(F("/"));
		SerialToConsole.print(position_info.step_count - 1);
		SerialToConsole.print(F(", Frame: "));
		for (i=0; i<position_info.servo_cnt; i++) //打印该帧的所有舵机的运行目标位置，用逗号分隔。结束用#
		{
			SerialToConsole.print(position_info.positions[curr_step * position_info.servo_cnt + i]);
			SerialToConsole.print(F(","));
		}
		SerialToConsole.println(F("#"));
	#endif

	for (i=0; i<position_info.servo_cnt; i++)
	{
		uint8_t servo_addr = position_info.servo_ids[i];
		int32_t pos = position_info.positions[curr_step * position_info.servo_cnt + i];
                ProfiledAbsolutePositionMove(servo_addr, pos);
	}
}
