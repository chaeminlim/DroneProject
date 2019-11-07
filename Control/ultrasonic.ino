/*
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
*/
