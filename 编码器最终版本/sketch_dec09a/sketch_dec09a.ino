/************************************************************
                SmartCar12th
      (c) Copyright 2017 HUST RENESAS LAB,The Department of CSE, HUST
                          All Rights Reserved
  Filename                :   SmartCar12th.c
  Programmer(s)           :   Cai zhi
  Description             :
  Modification History    :
  01a 2017-10-3 20:34:57
****************************************************************/
#include <Servo.h>
#include <MsTimer2.h>
Servo myservo;  //声明myservo为 Servo类的一个实例，在这之后就可以通过myservo调用Servo类的函数

#define  SERVO_CENTER      80
#define  MAX_ANGLE         50
#define  Threshold         700
#define  PWM_INSIDE_R      (pwmdutyTemp * InsideFDiff  / 100)
//#define TEST
#define ENCODER_USED
#define TABLE_DIMENSION    60

const int gl_speedDifference[TABLE_DIMENSION]  = {100, 98, 97, 95, 93, 92, 90, 88, 87, 85,
                                                  84, 82, 81, 79, 78, 76, 75, 73, 72, 71,
                                                  69, 68, 66, 65, 64, 62, 61, 59, 58, 57,
                                                  55, 54, 52, 51, 50, 48, 47, 45, 44, 42,
                                                 41, 39, 38, 36, 35, 33, 32, 30, 29, 27,
                                                  25, 24, 22, 20, 19, 17, 15, 13, 11, 9
                                                 };

int MOTOR_LEFT  = 5;  //左电机PWM引脚定义
int MOTOR_RIGHT = 6;  //右电机PWM引脚定义
int MOTOR_DIR = 9;
int SERVO       = 10;  //舵机PWM引脚定义


//传感器输入端口定义，D7对应用于 发车 的传感器
int D1 = A0;
int D2 = A1;
int D3 = A2;
int D4 = A3;
int D5 = A4;
int D6 = 0;//(之前是2)
int D7 = A5;//(之前是3PWM)
int D8 = 7;//(之前是11PWM)
int D9 = 8;//(之前是11PWM)
int D10 = 12;
int D11 = 13;
int D12 = 4;


//全局变量初始化
static int gl_cMotorPWM     = 0;  //电机PWM值(0-100)
static int gl_cServoAngle   = 0;  //舵机角度值(-90-90度)
static int gl_servo_counter = 0;  //舵机控制周期计数值(例程中没有用到)
int gl_uiDigitalSensor      = 0;  //当前传感器状态对应数值
int gl_uiLastDigitalSensor  = 0;  //上一次传感器状态对应数值
int gl_ucSensorNumber       = 0;  //检测到黑线的传感器数量
int gl_iPattern             = 0;  //运行模式
int gl_iNowErr              = 0;  //当前误差

//设置不同偏差对应的电机占空比(例程是通过控制电机PWM，而不是控制电机转速，所以要考虑长直道一直加速的问题)
int gl_iStraightSpeed = 60;
int gl_iR600Speed     = 40;
int gl_iR450Speed     = 40;

int StraightSpeed = 320;//310///300;//290//0.9//300//320//300;
int R600Speed     = 270;//270//250//260//0.9//280//280//250;
int R450Speed     = 230;//220//180//240//0.9//240//240//180;
//12.05 int gl_iSensorDistance[11] = {45, 36, 24, 12, 5, 0, -5, -12, -24, -36, -45};  //定义不同位置传感器对应的偏差量，用于计算当前偏差，具体使用见GetErr()函数
int gl_iSensorDistance[11] = {45, 38, 30, 20, 4, 0, -4, -20, -30, -38, -45}; 
volatile long pluse_count = 0;
volatile int last_pluse_count = 0;
volatile int delta_count = 0;
volatile int car_speed = 0;
int ExpectSpeed = 0;
int SpeedErr = 0;
float    speed_Kp_value=0.0 ,speed_Ki_value=0.0;
#define T 0.004
#define COUNTER 7816
/************************************************************************

   函数功能         初始化
   输入参数         NONE
   返回值           NONE

 ************************************************************************/
void setup()
{
  myservo.attach(SERVO);  //将SERVO引脚连接到myservo
  Serial.begin(115200);
  //Serial.println();
  pinMode(2, INPUT_PULLUP);
  pinMode(D6, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), pluse_counter, RISING );

  MsTimer2::set(4, get_speed); // 10ms period
  MsTimer2::start();
}

void pluse_counter()
{
  pluse_count++;
}

//10ms执行一次
void get_speed()
{
  delta_count = pluse_count - last_pluse_count;
  car_speed = delta_count * 100 / T / COUNTER; //COUNTER对应小车前进一米的脉冲数,T为时间间隔
  last_pluse_count = pluse_count;
/*
 Serial.print(ExpectSpeed);      //the first variable for plotting
  Serial.print(",");              //seperator
  Serial.print(car_speed);          //the second variable for plotting including line break
  Serial.print(",");              //seperator
  Serial.println(gl_cMotorPWM);*/
/*
  char strOut[200];
  sprintf(strOut, "gl_uiDigitalSensor: %u\r gl_ucSensorNumber:%u\r gl_iPattern:%u\r  gl_iNowErr:%d\r\n",
        gl_uiDigitalSensor,gl_ucSensorNumber,gl_iPattern,gl_iNowErr);
  Serial.println(strOut);*/
}


/************************************************************************

   函数功能         计算角度对应占空比(16bit)并设限制舵机最大打角，保护舵机，防止舵机堵转
   输入参数         angle
   返回值           NONE

 ************************************************************************/
void SetServoAngle(int angle)
{
  if (angle > MAX_ANGLE)
  {
    angle = MAX_ANGLE;
  }
  else if (angle < -MAX_ANGLE)
  {
    angle = 0 - MAX_ANGLE;
  }
  myservo.write(SERVO_CENTER + angle);  //输出角度SERVO_CENTER + angle
}

/************************************************************************

   函数功能         差速计算并通过PWMOut2输出电机对应pwm(两驱方案以外侧后轮为基准，内轮乘以<1的系数实现减速)
   输入参数         pwmduty,angle
   返回值           NONE

 ************************************************************************/
void PWMOut(int pwmduty, int angle)
{
  char InsideFDiff = 0;     // Speed differential proportion of inside front wheel     positive PWM
  int pwmdutyTemp = 0;
  int angleTemp = 0;

  pwmdutyTemp = pwmduty;
  angleTemp   = angle;

  if (angleTemp > 0)
  {
    if (angleTemp > (TABLE_DIMENSION - 1))  //防止寻址溢出
    {
      angleTemp = TABLE_DIMENSION - 1;
    }
    InsideFDiff  = gl_speedDifference[angleTemp];
    analogWrite(MOTOR_LEFT, pwmdutyTemp * 255  / 100);
    analogWrite(MOTOR_RIGHT, PWM_INSIDE_R * 255 / 100);
  }
  else
  {
    angleTemp = 0 - angleTemp;
    if (angleTemp > (TABLE_DIMENSION - 1))
    {
      angleTemp = TABLE_DIMENSION - 1;
    }
    InsideFDiff  = gl_speedDifference[angleTemp];
    analogWrite(MOTOR_LEFT, PWM_INSIDE_R * 255  / 100);
    analogWrite(MOTOR_RIGHT, pwmdutyTemp * 255  / 100);
  }
}


/************************************************************************

   函数功能         获得传感器状态
   输入参数         NONE
   返回值           DSensorValue

 ************************************************************************/
int GetDigitalSensorValue(void)
{
  int DSensorValue = 0;
  int Value[11] = {0};

  Value[0] = analogRead(D1) > Threshold; //D1对应最左侧传感器
  Value[1] = analogRead(D2) > Threshold;
  Value[2] = analogRead(D3) > Threshold;
  Value[3] = analogRead(D4) > Threshold;
  Value[4] = analogRead(D5) > Threshold;
  //Value[5] = analogRead(D6) > Threshold;
  Value[5] = digitalRead(D6);
  //Value[6] = digitalRead(D7);
  Value[6] = digitalRead(D8);
  Value[7] = digitalRead(D9);
  Value[8] = digitalRead(D10);
  Value[9] = digitalRead(D11);
  Value[10] = digitalRead(D12);


  DSensorValue |= (!(Value[10] & 0x01));
  DSensorValue |= (!(Value[9] & 0x01))  << 1;
  DSensorValue |= (!(Value[8] & 0x01))  << 2;
  DSensorValue |= (!(Value[7] & 0x01))  << 3;
  DSensorValue |= (!(Value[6] & 0x01))  << 4;
  DSensorValue |= (!(Value[5] & 0x01))  << 5;
  DSensorValue |= (!(Value[4] & 0x01))  << 6;
  DSensorValue |= (!(Value[3] & 0x01))  << 7;
  DSensorValue |= (!(Value[2] & 0x01))  << 8;
  DSensorValue |= (!(Value[1] & 0x01))  << 9;
  DSensorValue |= (!(Value[0] & 0x01))  << 10;
  return DSensorValue;
}

/************************************************************************

   函数功能         获得检测到黑线的传感器的数量
   输入参数         digital_sensor
   返回值           NONE

 ************************************************************************/
void GetDigitalSensorNum(int digital_sensor)
{
  int i = 0;
  unsigned char sensor_counter = 0;

  for (i = 0; i < 11; i ++)
  {
    sensor_counter += (digital_sensor & 0x01);
    digital_sensor >>= 1;
  }
  gl_ucSensorNumber = sensor_counter;
}

void GetExpectPWM(int SpeedErr)
{
    static int Sum_SpeedErr = 0;
    Sum_SpeedErr +=SpeedErr;
    if(SpeedErr>0)
    {
       digitalWrite(MOTOR_DIR,0);
    }
    else
    {
       digitalWrite(MOTOR_DIR,1);   
    }
  
    if(SpeedErr > 100)
       gl_cMotorPWM = 98;
    else if(SpeedErr > 80)
      gl_cMotorPWM = 98;
    else if(SpeedErr > 60)
      gl_cMotorPWM = 98;      
    else if(SpeedErr > 40)
      gl_cMotorPWM = 40; 
    else if(SpeedErr > 20)
      gl_cMotorPWM = 20; 
    else if(SpeedErr > 0)
      gl_cMotorPWM = 10;
    else if(SpeedErr > -20)   
      gl_cMotorPWM = 20;         
    else if(SpeedErr > -40)   
      gl_cMotorPWM = 40; 
    else if(SpeedErr > -60)   
      gl_cMotorPWM = 98;
    else if(SpeedErr > -80)   
      gl_cMotorPWM = 98;
    else if(SpeedErr > -100)   
      gl_cMotorPWM = 98;   

    /*  //pi控制
    if(SpeedErr > 100||SpeedErr < -100)
       gl_cMotorPWM = 98;
    else
    {
       gl_cMotorPWM = Get_Abs_Expected_PWM(SpeedErr,&Sum_SpeedErr);
       if(gl_cMotorPWM<0)
          gl_cMotorPWM = -gl_cMotorPWM;
       if(gl_cMotorPWM>98)
          gl_cMotorPWM = 98;
    }*/
}

int Get_Abs_Expected_PWM(int speed_error,int *total_error)
{   
    int      s_speed_PWM = 0;
//    float    speed_Kp_value=0.0 ,speed_Ki_value=0.0;
    float    speed_Kp = 1.0;
    float    speed_Ki = 0.1;
    
    int    s_error_total = *total_error;    
    //速度上不去可能是因为积分器值不够大 ,太大后不稳定
    if(s_error_total >= 500)
    {
            s_error_total = 500;
    }
    else if(s_error_total <= -500)
    {
            s_error_total = -500;
    }  
    
    //积分清零  30
    if((speed_error>=20)&&(s_error_total<0))
    {
        s_error_total=0;
    }
    else if((speed_error<=-20)&&(s_error_total>0))
    {
        s_error_total=0;
    }
    *total_error = s_error_total;       //更新
    
    /*
    speed_Kp = 1.00;  //0.9
    speed_Ki = 0.1; //0.03
    */  
    
    if(speed_error >= 50)
    {
            speed_Kp = 0.80;  //0.80
            speed_Ki = 0.05;  //0.05
    }
    else if(speed_error >= 20) //25
    {
            speed_Kp = 0.64;    //0.64
            speed_Ki = 0.08;  //0.06
    }
    else if(speed_error >= 0)
    {
            speed_Kp = 0.56;  //0.56
            speed_Ki = 0.08;  //0.08
    }
    else if(speed_error >= -20)
    {
            speed_Kp = 0.56;  //0.56
            speed_Ki = 0.08;  //0.08
    }
    else if(speed_error >= -50)
    {
            speed_Kp = 0.64;  //0.64
            speed_Ki = 0.08;  //0.06
    }
    else
    {
            speed_Kp = 0.80;  //0.80
            speed_Ki = 0.05;    //0.05
    }        
    
    speed_Kp_value = (float) speed_error * speed_Kp;    //float导致的溢出
    speed_Ki_value = (float) s_error_total * speed_Ki;

    s_speed_PWM = (int)((speed_Kp_value + speed_Ki_value));

    return s_speed_PWM;
}
/************************************************************************

   函数功能         Control
   输入参数         NONE
   返回值           NONE

 ************************************************************************/
void Control(void)
{
  int servoAngleTemp = 0;

  //计算当前偏差
  gl_iNowErr = GetErr();
  servoAngleTemp = gl_iNowErr;
  gl_cServoAngle = servoAngleTemp;

  //设置舵机角度
  SetServoAngle(gl_cServoAngle);
  if (servoAngleTemp < 0)
  {
    servoAngleTemp = 0 - servoAngleTemp;
  }

  //设置电机速度
  #ifdef  ENCODER_USED //如果使用编码器
    
    if ((servoAngleTemp >= 0) && (servoAngleTemp < 5))
    {
      ExpectSpeed = StraightSpeed;
    }
    else if (servoAngleTemp < 20)
    {
      ExpectSpeed = R600Speed;
    }
    else
    {
      ExpectSpeed = R450Speed;
    }
    SpeedErr = ExpectSpeed - car_speed;
    GetExpectPWM(SpeedErr);
    if(gl_iPattern == 9)
      gl_cMotorPWM= 0;
    PWMOut(gl_cMotorPWM, gl_cServoAngle);
    
  #else //如果不使用编码器
    if ((servoAngleTemp >= 0) && (servoAngleTemp < 5))
    {
      gl_cMotorPWM = gl_iStraightSpeed;
      PWMOut(gl_cMotorPWM, 0);
    }
    else if (servoAngleTemp < 20)
    {
      gl_cMotorPWM = gl_iR600Speed;
      PWMOut(gl_cMotorPWM, gl_cServoAngle);
    }
    else
    {
      gl_cMotorPWM = gl_iR450Speed;
      PWMOut(gl_cMotorPWM, gl_cServoAngle);
    }
  #endif
}


/************************************************************************

   函数功能         GetErr
   输入参数         NONE
   返回值           Err
   修改时间:
   01a 2017-10-21 14:41:48

 ************************************************************************/
int GetErr(void)
{
  int i                    = 0;
  int iPositionErr         = 0;
  static int s_iLastErr    = 0;
  unsigned int sensor      = 0;
  unsigned char lightCount = 0;

  sensor = gl_uiDigitalSensor; //将传感器状态赋给临时变量，防止操作过程被误修改

  for (i = 0; i < 11; i ++)
  {
    if (sensor & 0x01)
    {
      /* if the light is too far from last m ,do not count*/
      if (((gl_iSensorDistance[i] - gl_iNowErr) < 20) && ((gl_iSensorDistance[i] - gl_iNowErr) > -20))
      {
        iPositionErr += gl_iSensorDistance[i];
        lightCount++;
      }
    }
    sensor >>= 1;
  }

  if (lightCount > 0)
  {
    s_iLastErr = iPositionErr / lightCount;
    return (iPositionErr / lightCount);
  }
  else
  {
    return s_iLastErr;
  }
}


void loop()
{
  //如果是在测试模式下，则执行以下代码(#ifdef至#else中的代码)
  //测试模式是用于测试舵机中心值，因为舵机中心值90°与机械中心值不一定重合，所以需要校准
  //并将校准后实际的舵机中心值对应的宏SERVO_CENTER做相应的修改
#ifdef TEST
  char strOut[75];
  int incomingByte = 0;
  sprintf(strOut, "SERVO_CENTER: %u\r\n", gl_cServoAngle + SERVO_CENTER);
  Serial.println(strOut);
  Serial.print("a：-1 d：+1 w：+5 s：-5 ");

  // send data only when you receive data:
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    incomingByte = Serial.read();
    switch (incomingByte)
    {
      case 'a':
        gl_cServoAngle += -1;
        break;
      case 'd':
        gl_cServoAngle += 1;
        break;
      case 'w':
        gl_cServoAngle += 5;
        break;
      case 's':
        gl_cServoAngle += -5;
        break;
      default: break;
    }
  }
  SetServoAngle(gl_cServoAngle);

#else
  //计算传感器的状态对应数值，计算检测到黑线的传感器数量，并通过串口显示出来，方便调试。
  gl_uiDigitalSensor = GetDigitalSensorValue();
  GetDigitalSensorNum(gl_uiDigitalSensor);

  char strOut[200];
  sprintf(strOut, "gl_uiDigitalSensor: %u\r gl_ucSensorNumber:%u\r gl_iPattern:%u\r  gl_iNowErr:%d\r\n",
       gl_uiDigitalSensor,gl_ucSensorNumber,gl_iPattern,gl_iNowErr);
  Serial.println(strOut);
 
  //char strOut[200];
  //sprintf(strOut, "pluse_count : %u\r\n", pluse_count);
  //sprintf(strOut, "car_speed:%u\r\n",car_speed);
  //Serial.println(strOut);
  
  //sprintf(strOut, "car_speed:%d\r  ExpectSpeed:%d\r gl_cMotorPWM:%d\r \n speed_Kp_value:%f\r speed_Ki_value:%f\r\n",car_speed,ExpectSpeed,gl_cMotorPWM,speed_Kp_value,speed_Ki_value);
  //Serial.println(strOut);
  
  switch (gl_iPattern)
  {
    case 0:
      if (!(analogRead(D7) > Threshold))
      {
        gl_iPattern = 11;
      }
      break;

    case 11://正常运行模式
      Control();
      if ((gl_uiLastDigitalSensor == 0x0001) && (gl_uiDigitalSensor == 0x0000))
      {
        gl_iPattern = 21;
      }
      else if ((gl_uiLastDigitalSensor == 0x0400) && (gl_uiDigitalSensor == 0x0000))
      {
        gl_iPattern = 22;
      }
      /*else if (pluse_count / COUNTER>35)
      {
        gl_iPattern = 9;
      }*/
      break;

    case 21://右侧传感器丢线
      analogWrite(MOTOR_LEFT, 100);
      analogWrite(MOTOR_RIGHT, 0);
      if (gl_uiDigitalSensor & 0x0003) //最右侧两个传感器之一重新检测到黑线
      {
        gl_iPattern = 11; //回到正常运行状态
      }
      break;

    case 22://左侧传感器丢线
      analogWrite(MOTOR_LEFT, 0);
      analogWrite(MOTOR_RIGHT, 100);
      if (gl_uiDigitalSensor & 0x0600) //最左侧两个传感器之一重新检测到黑线
      {
        gl_iPattern = 11;
      }
      break;

    case 9://停车
      Control();
      gl_cMotorPWM = 0;
      PWMOut(gl_cMotorPWM, 0);
      break;
  }
  gl_uiLastDigitalSensor = gl_uiDigitalSensor;
#endif
}

