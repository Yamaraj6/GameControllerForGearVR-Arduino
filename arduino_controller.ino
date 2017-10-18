#include <SoftwareSerial.h>                   //importuj bibliotekę SoftwareSerial
//ACCELEROMETER/GYROSCOPE LIBS
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

SoftwareSerial Bluetooth(0, 1);
MPU6050 mpu(0x68);

#define BT_BEGIN_SIGN '<'
#define BT_END_SIGN '>'
#define BT_BREAK_SIGN ';'

// F1 - little finger, F2 - ring f., F3 - middle f.
// F4 - index f., F5 - thumb
///////////////////////////   FLEX SENSORS PINS   ///////////////////////////
const int FLEX_PIN[] = { 6, 3, 2, 1, 0};
float d_flex_sensor[5];
int i_flex_min[5];
int i_flex_max[5];

//////////////////////////////   MOTORS PINS   //////////////////////////////
const int MOTOR_PIN[] = { 8, 4, 5, 7, 9};
float d_motor_power = 0.05;

/////////////////////////   ACCELEROMETER/GYROSCOPE   ////////////////////////
#define INTERRUPT_PIN 2
VectorInt16 v3_acc;
VectorInt16 v3_last_acc;
VectorInt16 v3_now_acc;
VectorInt16 v3_linear_acc;
VectorInt16 v3_world_acc;
Quaternion q_gyro;      // [w, x, y, z]
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat v3_gravity;
bool b_calibrate = false;
bool b_calibration_successful = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//BLUETOOTH
String s_send_data;
String s_recived_data = "0.00;0.00;0.00;0.00;0.00;0";
float d_recived_data[6];


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
  Wire.begin();
  // Wire.setClock(400000);

  Serial.begin(38400);
  Bluetooth.begin(38400);  //ZMIENIĆ NA 30K -50K
  pinMode(INTERRUPT_PIN, INPUT);
  for (int i = 0 ; i < 5 ; i++)
  {
    pinMode(FLEX_PIN[i], INPUT);
    pinMode(MOTOR_PIN[i], OUTPUT);
    d_flex_sensor[i] = 0;
    i_flex_min[i] = 1000;
    i_flex_max[i] = 0;
  }
  SetupMPU();
}

void SetupMPU()
{
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(144);
  mpu.setYGyroOffset(44);
  mpu.setZGyroOffset(58);
  mpu.setXAccelOffset(-475);
  mpu.setYAccelOffset(-378);
  mpu.setZAccelOffset(1319);

  if (devStatus == 0) // make sure it worked (returns 0 if so)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;  // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
    Serial.println(F("DMP Initialization failed"));
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  ReceiveData();
  if (b_calibrate || Serial.read()=='a')
    Calibrate();
  
  AccelGyroUpdate();
  FlexSensorsRead();
  SendData();
  //PrintMPU(3);
  //MotorsControl();
}

void ReceiveData()
{
  if (Bluetooth.available())
  {
    String _sReceivedData = "";
    char c = Bluetooth.read();
    while (c != BT_BEGIN_SIGN && Bluetooth.available())
      c = Bluetooth.read();
    c = Bluetooth.read();
    while (c != BT_END_SIGN && Bluetooth.available())
    {
      _sReceivedData += c;
      c = Bluetooth.read();
    }
    if (_sReceivedData.length() > 0)
    {
      s_recived_data = _sReceivedData;
      ConvertRecivedData();
      Serial.println("RECEIVED DATA: " + _sReceivedData);
    }
  }
}

void ConvertRecivedData()
{
  String _sTemp;
  int j = 0;
  for (int i = 0; i < 6 ; i++)
  {
    _sTemp = "";
    while (s_recived_data[j] != BT_BREAK_SIGN && j < s_recived_data.length())
    {
      _sTemp += s_recived_data[j];
      j++;
    }
    d_recived_data[i] = _sTemp.toFloat();
    j++;
  }
  b_calibrate = d_recived_data[5];
}

void SendData()
{
  String _sDataToSend = "";
  _sDataToSend += BT_BEGIN_SIGN;
  _sDataToSend += q_gyro.x;
  _sDataToSend += ';';
  _sDataToSend += q_gyro.y;
  _sDataToSend += ';';
  _sDataToSend += q_gyro.z;
  _sDataToSend += ';';
  _sDataToSend += q_gyro.w;
  _sDataToSend += ';';
  _sDataToSend += v3_acc.x-v3_gravity.x;
  _sDataToSend += ';';
  _sDataToSend += v3_acc.y-v3_gravity.y;
  _sDataToSend += ';';
  _sDataToSend += v3_acc.z-v3_gravity.z;
  _sDataToSend += ';';
  for (int i = 0; i < 5; i++)
  {
    _sDataToSend += d_flex_sensor[i];
    _sDataToSend += ';';
  }
  _sDataToSend += b_calibration_successful;
  _sDataToSend += BT_END_SIGN;
  if (b_calibration_successful)
    {
      b_calibration_successful = false;
      b_calibrate = false;
    }
  Serial.println(_sDataToSend);
}

void FlexSensorsRead()
{ 
  for (int i = 0; i < 5 ; i++)
  {
    d_flex_sensor[i] = analogRead(FLEX_PIN[i]);
    CalibrateFlexSensors(d_flex_sensor[i], i);
    d_flex_sensor[i] = (d_flex_sensor[i] - i_flex_min[i]) / (i_flex_max[i] - i_flex_min[i]);
    if(d_flex_sensor[i]<0)
      d_flex_sensor[i]=0;
    else if(d_flex_sensor[i]>1)
      d_flex_sensor[i]=1;
  }
}

void CalibrateFlexSensors(float dFlexSensor, int iNumber)
{ for(int i=0; i<5;i++)
 { 
    i_flex_min[i]=100;
    i_flex_max[i]=250;
 }
/*  if (dFlexSensor < i_flex_min[iNumber])
    i_flex_min[iNumber] = dFlexSensor;
  if (dFlexSensor > i_flex_max[iNumber])
    i_flex_max[iNumber] = dFlexSensor;*/
}

void Calibrate()
{
  SetupMPU();       // Gyro calibration
  v3_acc.x = 0;
  v3_acc.y = 0;
  v3_acc.z = 0;
  mpu.dmpGetQuaternion(&q_gyro, fifoBuffer);
  mpu.dmpGetAccel(&v3_now_acc, fifoBuffer);
  mpu.dmpGetGravity(&v3_gravity, &q_gyro);
  mpu.dmpGetLinearAccel(&v3_linear_acc, &v3_now_acc, &v3_gravity);
  mpu.dmpGetLinearAccelInWorld(&v3_world_acc, &v3_linear_acc, &q_gyro);
 for(int i=0; i<5;i++)
 { 
    i_flex_min[i]=100;
    i_flex_max[i]=250;
 }
     // mpu.dmpGetEuler(euler, &q_gyro);
     // mpu.dmpGetYawPitchRoll(ypr, &q_gyro, &v3_gravity);
  v3_last_acc=v3_world_acc;
  b_calibration_successful = true;
}

void AccelGyroUpdate()
{
  if (dmpReady)
  {
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02)
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q_gyro, fifoBuffer);
      mpu.dmpGetAccel(&v3_now_acc, fifoBuffer);
      mpu.dmpGetGravity(&v3_gravity, &q_gyro);
    mpu.dmpGetLinearAccel(&v3_linear_acc, &v3_now_acc, &v3_gravity);
    mpu.dmpGetLinearAccelInWorld(&v3_world_acc, &v3_linear_acc, &q_gyro);
    v3_acc=v3_world_acc;
 /*   String a1="";
    a1+=v3_linear_acc.x;
    a1+="\t";    
    a1+=v3_linear_acc.y;
    a1+="\t";    
    a1+=v3_linear_acc.z;
    a1+="\t";    
    a1+=v3_world_acc.x;
    a1+="\t";    
    a1+=v3_world_acc.y;
    a1+="\t";    
    a1+=v3_world_acc.z;
    a1+="\t";    
    a1+=v3_gravity.x;
    a1+="\t";    
    a1+=v3_gravity.y;
    a1+="\t";    
    a1+=v3_gravity.z;
    Serial.println(a1);
     /* v3_acc.x = v3_now_acc.x - v3_last_acc.x;
      v3_acc.y = v3_now_acc.y - v3_last_acc.y;
      v3_acc.z = v3_now_acc.z - v3_last_acc.z;
      v3_last_acc.x = v3_now_acc.x;
      v3_last_acc.y = v3_now_acc.y;
      v3_last_acc.z = v3_now_acc.z;*/
    }
  }
}

void MotorsControl()
{
  if (d_motor_power < 0.95)
    d_motor_power += 0.1;
  else
    d_motor_power = 0.05;
  SetMotorState(MOTOR_PIN[0], fGetMotorSetting(0)); //0 * 5 + 1
  SetMotorState(MOTOR_PIN[1], fGetMotorSetting(2));
  SetMotorState(MOTOR_PIN[2], fGetMotorSetting(6));
  SetMotorState(MOTOR_PIN[3], fGetMotorSetting(11));
  SetMotorState(MOTOR_PIN[4], fGetMotorSetting(16));
}

void SetMotorState(int iMotorPin, float dMotorSetting)
{
  if (dMotorSetting >= d_motor_power)
    digitalWrite(iMotorPin, HIGH);
  else
    digitalWrite(iMotorPin, LOW);
}

float fGetMotorSetting(int iStartPosition)
{
  String _sTemp;
  while (s_recived_data[iStartPosition] != BT_BREAK_SIGN && iStartPosition < s_recived_data.length())
  {
    _sTemp += s_recived_data[iStartPosition];
    iStartPosition++;
  }
  return _sTemp.toFloat();
}

void PrintMPU(int iPrintNumber)
{
  int _iPrintNumber = iPrintNumber;
  if (_iPrintNumber == 0) {
    Serial.print("ACCELERATION:\t");
    Serial.print(v3_acc.x);
    Serial.print("\t");
    Serial.print(v3_acc.y);
    Serial.print("\t");
    Serial.println(v3_acc.z);
  }
  if (_iPrintNumber == 1) {
    Serial.print("GRAVITY:\t");
    Serial.print(v3_acc.x);
    Serial.print("\t");
    Serial.print(v3_acc.y);
    Serial.print("\t");
    Serial.println(v3_acc.z);
  }
  if (_iPrintNumber == 2) {
    Serial.print("GYRO:\t");
    Serial.print(q_gyro.x);
    Serial.print("\t");
    Serial.print(q_gyro.y);
    Serial.print("\t");
    Serial.print(q_gyro.z);
    Serial.print("\t");
    Serial.println(q_gyro.w);
  }
  if (_iPrintNumber == 3) {
    Serial.print("EULER (IN DEGREES)\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
  }
}

