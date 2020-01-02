#include <ESP32Servo.h> //라이브러리 불러오기
#include "BluetoothSerial.h"
#include <Wire.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#define X 0
#define Y 1
#define Z 2
#define PITCH 0
#define ROLL 1
#define YAW 2
#define NORMAL_MODE 1
#define CALIBRATION_MODE 2
#define STOP_MODE 3

typedef struct
{
  float target_angle = 0.0;
  float angle_in = 0.0;
  float rate_in;
  float stabilize_kp = 1;
  float stabilize_ki = 0;
  float rate_kp = 1;
  float rate_ki = 0;
  float stabilize_iterm;
  float rate_iterm;
  float final_output;
}DataForPID;

BluetoothSerial SerialBT;
Servo* bldc;
// 모터의 핀 번호
int PIN_NUM[4] = {32, 33, 25, 26};
// mpu센서의 주소값 - i2c 통신을 위해선 슬레이브 주소가 필요함
const int MPU_addr = 0x68;
// 센서에서 받아올 데이터를 이 변수들에 저장한다. raw data 
int16_t Ac[3]; // 측정값이 들어갈 변수
int16_t Gy[3]; // 측정값이 들어갈 변수
int16_t Tmp; 
float dt; // 센서 값 계산을 위해서 delta time을 계산해야한다. 그 값을 담기 위한 변수 

// 가속도와 자이로 값을 상보필터에 적용시키기 위하여 단위를 통일해야 한다. 
float accel_angle[3]; // 단위가 적용된 가속도값  
float filtered_angle[3]; // 상보필터 적용된 각도
float baseAc[3];
float baseGy[3];
float gyro[3]; // 단위가 적용된 자이로

unsigned long t_now, t_prev; // 시간 측정을 위한 변수
float throttle = 180.0;
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;
float base_target_angle[3];

DataForPID dataForPid[3];
int cnt = 0;
int Drone_Status = CALIBRATION_MODE;

void initMPU6050();
void calibAccelGyro();
void initDT();
void initYPR();
void readAccelGyro(); 
void calcDT(); // 시간 간격 계산
void calcAccelYPR(); // 가속도 센서 처리 루틴
void calcGyroYPR();
void calcFilteredYPR();
void calcYPRtoDualPID();
void calcMotorSpeed();   
void CheckSerialCommand(char command);
void RunMotor();
void SendDataToSerial();

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  initMPU6050();
  calibAccelGyro(); // 센서 보정
  initDT(); // 시간 간격 정보 처리
  initYPR();
  
  bldc = new Servo[4];
  for(int i = 0; i < 4; i++)
  {
    bldc[i].setPeriodHertz(50);
    bldc[i].attach(PIN_NUM[i]);   
  }
  SerialBT.begin("ESP32test"); //Bluetooth device name

  while(!SerialBT.available());
  
  serialFlush();
  
  SerialBT.println("bluetooth connected");
}



void loop()
{
  // loop for posture
  readAccelGyro(); 
  calcDT(); // 시간 간격 계산
  calcAccelYPR(); // 가속도 센서 처리 루틴
  calcGyroYPR();
  calcFilteredYPR();
  calcYPRtoDualPID();
  calcMotorSpeed();   
  if (Drone_Status == CALIBRATION_MODE)
    RunMotor(throttle);
  else if (Drone_Status == NORMAL_MODE)
    RunMotor();

  cnt++;
  if( cnt % 100 == 0)
  {
    SendDataToSerial();
    cnt = 0;
  }
  // 매번 가속도 센서의 값을 해석해서 rpy에 대한 각도를 구해야 한다.
  // 초음파센서 거리계산
  //Get_Ultrasonic_Distance();
  // loop for posture end
  
  //
  if (SerialBT.available()) 
  {
    int temp = SerialBT.parseInt();
    SerialBT.println(temp);
    serialFlush();
    
    if (temp == 0 || temp == 20 || temp == 180)
    {
      SerialBT.println("CALIB");
      Drone_Status = CALIBRATION_MODE;
      throttle = temp;
    }
    else
    {
      SerialBT.println("NORMAL");
      Drone_Status = NORMAL_MODE;
      throttle = temp;
    }
  }
}
void serialFlush()
{
  while(SerialBT.available() > 0) {
    char t = SerialBT.read();
  }
}   
void initMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // pwr -mgmt - 1 레지스터의 주소값
  Wire.write(0);
  Wire.endTransmission(true); // false 시 연결 유지
}

void initDT()
{ t_prev = millis(); }

void calcDT()
{
    t_now = millis();
    dt = (t_now - t_prev) / 1000.0; // 밀리초 단위이기 때문
    t_prev = t_now;
}

void initYPR()
{
  for (int i = 0; i < 10; i++)
  {
    readAccelGyro();
    calcDT();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();

    for(int i = 0; i < 3; i++) // ROLL PITCH YAW
      base_target_angle[i] += filtered_angle[i];
    
    delay(100);
  }
  
  for(int i = 0; i < 3; i++) // ROLL PITCH YAW
  {
    base_target_angle[i] /= 10;
    dataForPid[i].target_angle = base_target_angle[i];
  }
}

void calcMotorSpeed()
{
  motorA_speed = throttle + dataForPid[YAW].final_output + dataForPid[ROLL].final_output + dataForPid[PITCH].final_output;
  motorB_speed = throttle - dataForPid[YAW].final_output - dataForPid[ROLL].final_output + dataForPid[PITCH].final_output;
  motorC_speed = throttle + dataForPid[YAW].final_output - dataForPid[ROLL].final_output - dataForPid[PITCH].final_output;
  motorD_speed = throttle - dataForPid[YAW].final_output + dataForPid[ROLL].final_output - dataForPid[PITCH].final_output;
  
  if (motorA_speed <= 25) motorA_speed = 25;
  if (motorB_speed <= 25) motorB_speed = 25;
  if (motorC_speed <= 25) motorC_speed = 25;
  if (motorD_speed <= 25) motorD_speed = 25;
  if (motorA_speed >= 179) motorA_speed = 170;
  if (motorB_speed >= 179) motorB_speed = 170;
  if (motorC_speed >= 179) motorC_speed = 170;
  if (motorD_speed >= 179) motorD_speed = 170;
}
void calcFilteredYPR() //상보필터
{
  const float ALPHA = 0.96;
  float tmp_angle[3]; //임시 각도

  for(int i = 0; i < 3; i++)
  {
    tmp_angle[i] = filtered_angle[i] + gyro[i] * dt; 
    // 이전 보정 각도 + 현재 자이로 센서를 이용해 얻은 각도(gyro_x * dt)
  }
  
  filtered_angle[X] = ALPHA * tmp_angle[X] + (1.0 - ALPHA) * accel_angle[X];
  filtered_angle[Y] = ALPHA * tmp_angle[Y] + (1.0 - ALPHA) * accel_angle[Y];
  filtered_angle[Z] = tmp_angle[Z];
}


void readAccelGyro()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  
  for(int i = 0; i < 3; i++)
    Ac[i] = Wire.read() << 8 | Wire.read();
    
  Tmp = Wire.read() << 8 | Wire.read();

  for(int i = 0; i < 3; i++)
    Gy[i] = Wire.read() << 8 | Wire.read();
  
}

  
void calibAccelGyro() 
{
  float sumAc[3] = {0.0, };
  float sumGy[3] = {0.0, };
  
  //가속도 자이로 센서를 읽음
  readAccelGyro();
 
  //읽어들였으면 이제 읽은 값을 토대로 평균값을 구하면 됨
  for(int i = 0; i < 10; i++)
  {
    readAccelGyro();
    for(int j = 0; j < 3; j++)
    {
      sumAc[j] += Ac[j];
      sumGy[j] += Gy[j];
    }    
    delay(100);//0.1초
  }
  //맨 처음 기본 센서 값들을 보여지고 그다음에 평균값을 구하는 함수
  for(int i = 0; i< 3; i++)
  {
    baseAc[i] = sumAc[i] / 10;
    baseGy[i] = sumGy[i] / 10;
  }
}

void RunMotor()
{
  bldc[0].write(motorA_speed);
  bldc[1].write(motorB_speed);
  bldc[2].write(motorC_speed);
  bldc[3].write(motorD_speed);
}

void RunMotor(float throttle)
{
  bldc[0].write(throttle);
  bldc[1].write(throttle);
  bldc[2].write(throttle);
  bldc[3].write(throttle);
}

void RunCalibrationMode()
{
  Drone_Status = CALIBRATION_MODE;
  RunMotor(180.0);
}
void RunStopMode()
{
  Drone_Status = STOP_MODE;
}
void RunNormalMode()
{
    Drone_Status = NORMAL_MODE;
}

void CheckSerialCommand(char command)
{
  SerialBT.write(10);
  switch(command)
  {
    case '2': throttle = 20.0; break;
    case '3': throttle = 30.0; break;
    case '4': throttle = 40.0; break;
    case '5': throttle = 50.0; break;
    case '6': throttle = 60.0; break;
    case '7': throttle = 70.0; break;
    case '8': throttle = 80.0; break;
    case '9': throttle = 90.0; break;
    case 'a': throttle = 100.0; break;
    case 'b': throttle = 110.0; break;
    case 'c': throttle = 120.0; break;
    case 'd': throttle = 130.0; break;
    case 'e': throttle = 140.0; break;
    case 'f': throttle = 150.0; break;
    case 'C': RunCalibrationMode();
    case 'S': RunStopMode();
    case 'N': RunNormalMode();
    default : break;
  }
}

void calcAccelYPR() //가속도 센서 처리 루틴
{
  float accel_x, accel_y, accel_z; //x, y, z 축에 대한 각도 저장 변수
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;
 
  accel_x = Ac[X] - baseAc[X];
  accel_y = Ac[Y] - baseAc[Y];
  accel_z = Ac[Z] + (16384 - baseAc[Z]);
 
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle[X] = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;
 
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle[Y] = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;
 
  accel_angle[Z] = 0;
}

void dualPID(DataForPID& DFP)
{
  float angle_error;
  float desire_rate;
  float rate_error;
  float stabilize_pterm;
  float rate_pterm;

  angle_error = DFP.target_angle - DFP.angle_in;
  stabilize_pterm = DFP.stabilize_kp * angle_error;
  DFP.stabilize_iterm += DFP.stabilize_ki * angle_error *dt;
  desire_rate = stabilize_pterm;
  rate_error = desire_rate - DFP.rate_in;
  rate_pterm = DFP.rate_kp * rate_error;
  DFP.rate_iterm += DFP.rate_ki * rate_error * dt;
  DFP.final_output = rate_pterm + DFP.rate_iterm + DFP.stabilize_iterm;
}


void calcYPRtoDualPID()
{
  for(int i = 0; i < 3; i++)
  {
    dataForPid[i].angle_in = filtered_angle[i];
    dataForPid[i].rate_in = gyro[i];
    dualPID(dataForPid[i]);
  }
}

void calcGyroYPR()
{
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131; // 각속도를 저장하는 변수
  for(int i = 0; i < 3; i++)
  {
    gyro[i] = (Gy[i] - baseGy[i]) / GYROXYZ_TO_DEGREES_PER_SEC;
  }
 // gyro_angle_x += gyro_x * dt;
 // gyro_angle_y += gyro_y * dt;
 // gyro_angle_z += gyro_z * dt;
 /*주석을 하는 이유
  * 이 부분은 자이로 센서 자체적으로 회전각을 구하는 부분,
  * 이제는 PID 제어를 통해 회전각을 구할것이다.
  */
}

void SendDataToSerial()
{

  int g = Gy[Z];
  Serial.print("<");
  Serial.print(g);
  Serial.print(">");
  Serial.print("<");
  Serial.print(filtered_angle[X], 2);
  Serial.print(F("r:"));
  Serial.print(filtered_angle[Y], 2);
  Serial.print(F("p:"));
  Serial.print(filtered_angle[Z], 2);
  Serial.print(F("y:"));

  Serial.print(dataForPid[ROLL].final_output, 2);
  Serial.print(F("R:"));
  Serial.print(dataForPid[PITCH].final_output, 2);
  Serial.print(F("P:"));
  Serial.print(dataForPid[YAW].final_output, 2);
  Serial.print(F("Y:"));
  Serial.println(">");
}
