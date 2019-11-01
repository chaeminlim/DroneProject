#include <Wire.h>

// mpu센서의 주소값 - i2c 통신을 위해선 슬레이브 주소가 필요함
const int MPU_addr = 0x68;
// 센서에서 받아올 데이터를 이 변수들에 저장한다. raw data 
int16_t  AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 
 // 센서 값 계산을 위해서 delta time을 계산해야한다. 그 값을 담기 위한 변수
float dt;
// 가속도와 자이로 값을 상보필터에 적용시키기 위하여 단위를 통일해야 한다. 
float accel_angle_x, accel_angle_y, accel_angle_z;  
// float gyro_angle_x, gyro_angle_y, gyro_angle_z; // calc 자이로 각도
float filtered_angle_x, filtered_angle_y, filtered_angle_z; // 상보필터 적용된 각도

float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;

float gyro_x, gyro_y, gyro_z;

unsigned long t_now, t_prev;

extern float roll_output, pitch_output, yaw_output;

float roll_target_angle = 0.0;
float roll_angle_in = 0.0;
float roll_rate_in;
float roll_stabilize_kp = 1;
float roll_stabilize_ki = 0;
float roll_rate_kp = 1;
float roll_rate_ki = 0;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in = 0.0;
float pitch_rate_in;
float pitch_stabilize_kp = 1;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 1;
float pitch_rate_ki = 0;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in = 0.0;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 1;
float yaw_rate_ki = 0;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;
/*
float roll_target_angle = 0.0;
float roll_prev_angle = 0.0;
float roll_kp = 1;
float roll_ki = 0;
float roll_kd = 0;
float roll_iterm;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_prev_angle = 0.0;
float pitch_kp = 1;
float pitch_ki = 0;
float pitch_kd = 0;
float pitch_iterm;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_prev_angle = 0.0;
float yaw_kp = 1;
float yaw_ki = 0;
float yaw_kd = 0;
float yaw_iterm;
float yaw_output;
*/

// 초음파 센서를 위한 핀 세팅

int echoPin1 = 10;
int echoPin2 = 11;
int echoPin3 = 12;
int echoPin4 = 13;
int trigPin1 = 14;
int trigPin2 = 15;
int trigPin3 = 16;
int trigPin4 = 17;

void setup() 
{
  Wire.begin();
  Serial.begin(115200);

  initMPU6050();
  calibAccelGyro(); // 센서 보정
  initDT(); // 시간 간격 정보 처리
  initYPR();
  
/*
  Ultrasonic_PinSetting();
*/

}
#define STOP 0
#define START 1

int flag = 0;
int cnt = 0;


void loop() 
{
    char c = Serial.read();
    switch (c)
    {
        case '$':
        flag = START; break;
        case '@':
        flag = STOP; break;
    } 

  if (flag == START)
  {
    readAccelGyro(); 
    calcDT(); // 시간 간격 계산
    calcAccelYPR(); // 가속도 센서 처리 루틴
    calcGyroYPR();
    calcFilteredYPR();
    //calcYPRtoStdPID();
    calcYPRtoDualPID();
    calcMotorSpeed();   
    
    cnt++;
    if( cnt % 4 == 0)
      SendDataToProcessing();
      cnt = 0;
    // 매번 가속도 센서의 값을 해석해서 rpy에 대한 각도를 구해야 한다.
    // 초음파센서 거리계산
    //Get_Ultrasonic_Distance();
  }
}

void Ultrasonic_PinSetting()
{
  pinMode(trigPin1, OUTPUT); pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT); pinMode(trigPin4, OUTPUT);
  pinMode(echoPin1, INPUT); pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT); pinMode(echoPin4, INPUT);
}

float Get_Ultrasonic_Distance(int trig, int echo)
{
  // 초음파를 보낸다. 다 보내면 echo가 HIGH 상태로 대기하게 된다.
  digitalWrite(trig, LOW);
  digitalWrite(echo, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  unsigned long duration = pulseIn(echo, HIGH); 
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  float distance = ((float)(340 * duration) / 10000) / 2;  

  // 단위 cm 
  return distance;
}


void initMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // pwr -mgmt - 1 레지스터의 주소값
  Wire.write(0);
  Wire.endTransmission(true); // false 시 연결 유지
}

void readAccelGyro()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}


void calibAccelGyro() 
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  
  //가속도 자이로 센서를 읽음
  readAccelGyro();
 
  //읽어들였으면 이제 읽어드린 값을 토대로 평균값을 구하면 됨
  for(int i = 0; i< 10; i++)
  {
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);//0.1초
  }
  //맨 처음 기본 센서 값들을 보여지고 그다음에 평균값을 구하는 함수
  baseAcX = sumAcX / 10;
  baseAcY = sumAcY / 10;
  baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10;
  baseGyY = sumGyY / 10;
  baseGyZ = sumGyZ / 10;
}


void initDT()
{ t_prev = millis(); }

void calcDT()
{
    t_now = millis();
    dt = (t_now - t_prev) / 1000.0; // 밀리초 단위이기 때문
    t_prev = t_now;
}

void calcAccelYPR() //가속도 센서 처리 루틴
{
  float accel_x, accel_y, accel_z; //x, y, z 축에 대한 각도 저장 변수
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;
 
  accel_x = AcX - baseAcX;
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);
 
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;
 
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;
 
  accel_angle_z = 0;
}

void calcGyroYPR()
{
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131; // 각속도를 저장하는 변수

  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;

 // gyro_angle_x += gyro_x * dt;
 // gyro_angle_y += gyro_y * dt;
 // gyro_angle_z += gyro_z * dt;
 /*주석을 하는 이유
  * 이 부분은 자이로 센서 자체적으로 회전각을 구하는 부분,
  * 이제는 PID 제어를 통해 회전각을 구할것이다.
  */
}
void calcFilteredYPR() //상보필터
{
   const float ALPHA = 0.96;
   float tmp_angle_x, tmp_angle_y, tmp_angle_z; //임시 각도

   tmp_angle_x = filtered_angle_x + gyro_x * dt; // 이전 보정 각도 + 현재 자이로 센서를 이용해 얻은 각도(gyro_x * dt)
   tmp_angle_y = filtered_angle_y + gyro_y * dt;
   tmp_angle_z = filtered_angle_z + gyro_z * dt;

   filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
   filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
   filtered_angle_z = tmp_angle_z;
}

/*
void stdPID(float& setpoint, float& input, float& prev_input, float& kp, float& ki, float& kd, float& iterm, float& output)
{
  float error;
  float dInput;
  float pterm, dterm;

  error = setpoint - input; 
  dInput = input - prev_input;
  prev_input = input;

  pterm = kp * error;
  iterm += ki * error * dt;
  dterm = -kd * (dInput / dt);

  output = pterm + iterm + dterm;
}

void calcYPRtoStdPID()
{
  stdPID(roll_target_angle, filtered_angle_y, roll_prev_angle, roll_kp, roll_ki, roll_kd, roll_iterm, roll_output);
  stdPID(pitch_target_angle, filtered_angle_x, pitch_prev_angle, pitch_kp, pitch_ki, pitch_kd, pitch_iterm, pitch_output);
  stdPID(yaw_target_angle, filtered_angle_z, yaw_prev_angle, yaw_kp, yaw_ki, yaw_kd, yaw_iterm, yaw_output);
}
*/
float throttle = 100.0;
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;


float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;

void initYPR()
{
  for (int i = 0; i < 10; i++)
  {
    readAccelGyro();
    calcDT();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();

    base_roll_target_angle += filtered_angle_y;
    base_pitch_target_angle += filtered_angle_x;
    base_yaw_target_angle += filtered_angle_z;
    
    delay(100);
  }
  base_roll_target_angle /= 10;
  base_pitch_target_angle /= 10;
  base_yaw_target_angle /= 10;

  roll_target_angle = base_roll_target_angle;
  pitch_target_angle = base_pitch_target_angle;
  yaw_target_angle = base_yaw_target_angle;
}

void SendDataToProcessing()
{

  Serial.print("<");
  Serial.print(filtered_angle_y, 2);
  Serial.print(F("r"));
  Serial.print(filtered_angle_x, 2);
  Serial.print(F("p"));
  Serial.print(filtered_angle_z, 2);
  Serial.print(F("y"));

  Serial.print(roll_output, 2);
  Serial.print(F("R"));
  Serial.print(pitch_output, 2);
  Serial.print(F("P"));
  Serial.print(yaw_output, 2);
  Serial.print(F("Y"));
  Serial.print(">");
}

void dualPID(
  float target_angle, 
  float angle_in, 
  float rate_in, 
  float stabilize_kp, 
  float stabilize_ki, 
  float rate_kp, 
  float rate_ki,
  float& stabilize_iterm,
  float& rate_iterm,
  float& output)
{
  float angle_error;
  float desire_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;

  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error *dt;
  
  desire_rate = stabilize_pterm;

  rate_error = desire_rate - rate_in;

  rate_pterm = rate_kp * rate_error;
  rate_iterm += rate_ki * rate_error * dt;

  output = rate_pterm + rate_iterm + stabilize_iterm;
}

void calcYPRtoDualPID()
{
  roll_angle_in = filtered_angle_y;
  roll_rate_in = gyro_y;
  dualPID(
    roll_target_angle,
    roll_angle_in,
    roll_rate_in,
    roll_stabilize_kp,
    roll_stabilize_ki,
    roll_rate_kp,
    roll_rate_ki,
    roll_stabilize_iterm,
    roll_rate_iterm,
    roll_output
  );
  pitch_angle_in = filtered_angle_x;
  pitch_rate_in = gyro_x;
  dualPID(
    pitch_target_angle,
    pitch_angle_in,
    pitch_rate_in,
    pitch_stabilize_kp,
    pitch_stabilize_ki,
    pitch_rate_kp,
    pitch_rate_ki,
    pitch_stabilize_iterm,
    pitch_rate_iterm,
    pitch_output
  );
  yaw_angle_in = filtered_angle_z;
  yaw_rate_in = gyro_z;
  dualPID(
    yaw_target_angle,
    yaw_angle_in,
    yaw_rate_in,
    yaw_stabilize_kp,
    yaw_stabilize_ki,
    yaw_rate_kp,
    yaw_rate_ki,
    yaw_stabilize_iterm,
    yaw_rate_iterm,
    yaw_output
  );
}

void calcMotorSpeed()
{
  
  motorA_speed = throttle + yaw_output + roll_output + pitch_output;
  motorB_speed = throttle - yaw_output - roll_output + pitch_output;
  motorC_speed = throttle + yaw_output - roll_output - pitch_output;
  motorD_speed = throttle - yaw_output + roll_output - pitch_output;

  if (motorA_speed < 0) motorA_speed = 0;
  if (motorB_speed < 0) motorA_speed = 0;
  if (motorC_speed < 0) motorA_speed = 0;
  if (motorD_speed < 0) motorA_speed = 0;
  if (motorA_speed > 180) motorA_speed = 180;
  if (motorB_speed > 180) motorA_speed = 180;
  if (motorC_speed > 180) motorA_speed = 180;
  if (motorD_speed > 180) motorA_speed = 180;


}

extern float base_roll_target_angle;
extern float base_pitch_target_angle;
extern float base_yaw_target_angle;
