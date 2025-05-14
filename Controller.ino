#include <Encoder.h>
#include <DeltaKinematics.h>
#include <Fuzzy.h>
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter bo_loc1(0.1, 0.1, 0.01);
SimpleKalmanFilter bo_loc2(0.1, 0.1, 0.01);
SimpleKalmanFilter bo_loc3(0.1, 0.1, 0.01);


const float e = 77.17;   // khoảng cách giữa các khớp nối end effector (PlatformTri)
const float f = 175.0;   // khoảng cách giữa các khớp nối khung trên (BassTri)
const float re = 297.0;  // chiều dài thanh nối dưới (RodLength)
const float rf = 170.0;  // chiều dài thanh nối trên (ArmLength)

DeltaKinematics DK(e, f, re, rf);

Fuzzy *fuzzy1 = new Fuzzy();
Fuzzy *fuzzy2 = new Fuzzy();
Fuzzy *fuzzy3 = new Fuzzy();

// === Khai báo các chân điều khiển động cơ ===
const int stepPins[3] = {2, 6, 10};   // Chân điều khiển bước
const int dirPins[3]  = {3, 7, 11};   // Chân điều khiển hướng
const int enablePins[3] = {4, 8, 12}; // Chân điều khiển bật/tắt động cơ

// === Thay thế các #define chân điều khiển động cơ ===

// Động cơ 1
#define DIR1 dirPins[0]   // Chân điều khiển hướng của động cơ 1
#define PWM1 stepPins[0]  // Chân điều khiển bước (PWM) của động cơ 1
#define EN1 enablePins[0] // Chân điều khiển bật/tắt của động cơ 1

// Động cơ 2
#define DIR2 dirPins[1]   // Chân điều khiển hướng của động cơ 2
#define PWM2 stepPins[1]  // Chân điều khiển bước (PWM) của động cơ 2
#define EN2 enablePins[1] // Chân điều khiển bật/tắt của động cơ 2

// Động cơ 3
#define DIR3 dirPins[2]   // Chân điều khiển hướng của động cơ 3
#define PWM3 stepPins[2]  // Chân điều khiển bước (PWM) của động cơ 3
#define EN3 enablePins[2] // Chân điều khiển bật/tắt của động cơ 3

#define switch1Pin  50
#define switch2Pin  41
#define switch3Pin  39

#define Kp 6.67
#define Kd 0.02
#define Ki 3.33

float CPR_ENCODER = 4000;

Encoder encoderMotor1(2, 3);
Encoder encoderMotor2(18, 19);
Encoder encoderMotor3(20, 21);

float T;
double position1, positionSet1, position2, positionSet2, position3, positionSet3;
double E01, E11, E21, E02, E12, E22, E03, E13, E23, E1e, E2e, E3e, DE1, DE2, DE3;
double alpha, beta, gamma;
double Output1, LastOutput1, Output2, LastOutput2, Output3, LastOutput3;
uint8_t dirMotor1, dirMotor2, dirMotor3;
unsigned long now1, now2, now3;

String data;
double receivedData[4];

void setup()
{
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(18,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(20,INPUT_PULLUP);
  pinMode(21,INPUT_PULLUP);

  pinMode(switch1Pin,INPUT_PULLUP);
  pinMode(switch2Pin,INPUT_PULLUP);
  pinMode(switch3Pin,INPUT_PULLUP);

  pinMode(DIR1,OUTPUT); 
  pinMode(PWM1,OUTPUT);
  pinMode(EN1,OUTPUT);
  pinMode(DIR2,OUTPUT); 
  pinMode(PWM2,OUTPUT);
  pinMode(EN2,OUTPUT);
  pinMode(DIR3,OUTPUT); 
  pinMode(PWM3,OUTPUT);
  pinMode(EN3,OUTPUT);
    
  // TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  
  E01=0; 
  E11=0; 
  E21=0;
  E02=0; 
  E12=0; 
  E22=0;
  E03=0; 
  E13=0; 
  E23=0;
  Output1=0;
  Output2=0; 
  Output3=0; 
  LastOutput1=0;
  LastOutput2=0;
  LastOutput3=0;
  T=0.01;
  dirMotor1=0;
  dirMotor2=0;
  dirMotor3=0;
  now1=0;
  now2=0;
  now3=0;
  Fuzzy_init();

  Serial.begin(115200);

  origin();
  position1 = 0; 
  position2 = 0;
  position3 = 0;

}

void loop()
{
  if (Serial.available() > 0) {
    data = Serial.readString();
    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = data.indexOf(',', commaIndex2 + 1);

    receivedData[0] = data.substring(0, commaIndex1).toDouble();
    receivedData[1] = data.substring(commaIndex1 + 1, commaIndex2).toDouble();
    receivedData[2] = data.substring(commaIndex2 + 1, commaIndex3).toDouble();
    receivedData[3] = data.substring(commaIndex3 + 1).toDouble();

    for (int i = 0; i < 2001; i++) {
      double x, y, z;

      if (receivedData[1] == 1) {
        x = 50 * sin(0.002 * PI * i);
        y = 50 * cos(0.002 * PI * i);
        z = -250;
      } else if (receivedData[1] == 2) {
        x = (50-0.01*i)*cos(0.002*pi*i);
        y = (50-0.01*i)*sin(0.002*pi*i);
        z = -250;
      } else if (receivedData[1] == 3) {
        x = 50 * sin(0.002 * PI * i);
        y = 50 * cos(0.002 * PI * i);
        z = -250 + 0.005*i;
      } else if (receivedData[1] == 4) {
        x = 50*cos(0.002*pi*i+pi);
        y = 50*sin(0.002*pi*i+pi);
        z = -5*sin(0.008*pi*i)-245;
      }
      DK.inverse(x, y, z);
      if (receivedData[0] == 0) PID();
      else if (receivedData[0] == 1) Fuzzy();
      runMotor(DK.a, DK.b, DK.c);
      Serial.println(String(DK.a) + ", " + String(position1) + ", " + String(DK.b) + ", " + String(position2) + ", " + String(DK.c) + ", " + String(position3));
    }
    Serial.println("Stop");
  }
  
}

bool robotIsAtOrigin() 
{
  // Đọc trạng thái của các công tắc hành trình
  bool switch1State = digitalRead(switch1Pin);
  bool switch2State = digitalRead(switch2Pin);
  bool switch3State = digitalRead(switch3Pin);
  // Trả về true nếu cả ba công tắc đều đóng
  return (switch1State == LOW  && switch2State == LOW && switch3State == LOW );
}

void origin() 
{
  // Đặt hướng quay lên cho tất cả driver
  while (!robotIsAtOrigin()){
    digitalWrite(DIR1, HIGH); // Lên
    digitalWrite(DIR2, HIGH); // Lên
    digitalWrite(DIR3, HIGH); // Lên   
    // Xoay driver để di chuyển lên
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, HIGH);
    digitalWrite(PWM3, HIGH);

    stop(1000);

    if (digitalRead(switch1Pin) == LOW){
      digitalWrite(PWM1, HIGH);
    } else {
      digitalWrite(PWM1, LOW);
    }
    if (digitalRead(switch2Pin) == LOW){
      digitalWrite(PWM2, HIGH);
    } else {
      digitalWrite(PWM2, LOW);
    }
    if (digitalRead(switch3Pin) == LOW){
      digitalWrite(PWM3, HIGH);
    } else {
      digitalWrite(PWM3, LOW);
    }

    stop(1000);
  }
  
  stop(250000);
  rotate(40);
  encoderMotor1.write(0);
  encoderMotor2.write(0);
  encoderMotor3.write(0);
  

}

void rotate(double degrees) {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    double stepsToRotate = degrees * 3200/360;

    for (double i = 0; i < stepsToRotate; i++) {
      digitalWrite(PWM1, HIGH);
      digitalWrite(PWM2, HIGH);
      digitalWrite(PWM3, HIGH);

      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, LOW);
      digitalWrite(PWM3, LOW);

      stop(1000);
    }  
}

void runMotor(double degree1, double degree2, double degree3)
{
  positionSet1 = degree1;
  positionSet2 = degree2;
  positionSet3 = degree3;

  if(LastOutput1!=0)
    if(micros()-now1>abs(LastOutput1))
    {
      digitalWrite(DIR1, dirMotor1);
      digitalWrite(PWM1, 1);
      digitalWrite(PWM1, 0);
      now1=micros();
    }

  if(LastOutput2!=0)
    if(micros()-now2>abs(LastOutput2))
    {
      digitalWrite(DIR2, dirMotor2);
      digitalWrite(PWM2, 1);
      digitalWrite(PWM2, 0);
      now2=micros();
    }

  if(LastOutput3!=0)
    if(micros()-now3>abs(LastOutput3))
    {
      digitalWrite(DIR3, dirMotor3);
      digitalWrite(PWM3, 1);
      digitalWrite(PWM3, 0);
      now3=micros();
    }  
}

void stop(unsigned long time)
{
  unsigned long timeNow = micros();
  while (true) if (micros() - timeNow > time) break;
  return;
}

void PID()
{ 
  alpha = 2*T*Kp + Ki*T*T + 2*Kd;
  beta = T*T*Ki - 4*Kd - 2*T*Kp;
  gamma = 2*Kd;
  position1 = bo_loc1.updateEstimate(encoderMotor1.read())*360/CPR_ENCODER;
  // position1 = encoderMotor1.read()*360/CPR_ENCODER;
  E01 = positionSet1 - position1;
  Output1 = (alpha*E01 + beta*E11 + gamma*E21 + 2*T*LastOutput1)/(2*T);
  LastOutput1 = Output1;
  
  E21=E11;
  E11=E01;
  if(LastOutput1>0) dirMotor1=0;
  else dirMotor1=1;
  if(LastOutput1>150) LastOutput1=150;
  if(LastOutput1<-150) LastOutput1=-150;
  // if(LastOutput1<5 && LastOutput1>-5) LastOutput1=0;
  // else if(LastOutput1<0) LastOutput1=abs(LastOutput1);


  position2 = bo_loc2.updateEstimate(encoderMotor2.read())*360/CPR_ENCODER;
  E02 = positionSet2 - position2;
  Output2 = (alpha*E02 + beta*E12 + gamma*E22 + 2*T*LastOutput2)/(2*T);
  LastOutput2 = Output2;
  
  E22=E12;
  E12=E02;
  if(LastOutput2>0) dirMotor2=0;
  else dirMotor2=1;
  if(LastOutput2>150) LastOutput2=150;
  if(LastOutput2<-150) LastOutput2=-150;
  // if(LastOutput2<5 && LastOutput2>-5) LastOutput2=0;
  // else if(LastOutput2<0) LastOutput2=abs(LastOutput2);


  position3 = bo_loc3.updateEstimate(encoderMotor3.read())*360/CPR_ENCODER;
  E03 = positionSet3 - position3;
  Output3 = (alpha*E03 + beta*E13 + gamma*E23 + 2*T*LastOutput3)/(2*T);
  LastOutput3 = Output3;
  
  E23=E13;
  E13=E03;
  if(LastOutput3>0) dirMotor3=0;
  else dirMotor3=1;
  if(LastOutput3>150) LastOutput3=150;
  if(LastOutput3<-150) LastOutput3=-150;
  // if(LastOutput3<5 && LastOutput3>-5) LastOutput3=0;
  // else if(LastOutput3<0) LastOutput3=abs(LastOutput3);

}

void Fuzzy_init()
{
  FuzzySet *e_N = new FuzzySet(-1, -1, -0.5, 0.01);
  FuzzySet *e_P = new FuzzySet(-0.01, 0.5, 1, 1);

  FuzzySet *de_N = new FuzzySet(-1, -1, -0.5, 0.25);
  FuzzySet *de_P = new FuzzySet(-0.25, 0.50, 1, 1);

  FuzzySet *u_N = new FuzzySet(-1, -1, -0.5, -0.01);
  FuzzySet *u_Z = new FuzzySet(-0.5, 0, 0, 0.5);
  FuzzySet *u_P = new FuzzySet(0.01, 0.5, 1, 1);

  FuzzyInput *e_input = new FuzzyInput(1);
  e_input -> addFuzzySet(e_N);
  e_input -> addFuzzySet(e_P);
  fuzzy1 -> addFuzzyInput(e_input);
  fuzzy2 -> addFuzzyInput(e_input);
  fuzzy3 -> addFuzzyInput(e_input);

  FuzzyInput *de_input = new FuzzyInput(2);
  de_input -> addFuzzySet(de_N);
  de_input -> addFuzzySet(de_P);
  fuzzy1 -> addFuzzyInput(de_input);
  fuzzy2 -> addFuzzyInput(de_input);
  fuzzy3 -> addFuzzyInput(de_input);

  FuzzyOutput *u_output = new FuzzyOutput(1);
  u_output -> addFuzzySet(u_N);
  u_output -> addFuzzySet(u_Z);
  u_output -> addFuzzySet(u_P);

  fuzzy1 -> addFuzzyOutput(u_output);
  fuzzy2 -> addFuzzyOutput(u_output);
  fuzzy3 -> addFuzzyOutput(u_output);

  FuzzyRuleAntecedent *NN = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *N = new FuzzyRuleConsequent();
  NN -> joinWithAND(e_N, de_N);
  N -> addOutput(u_N);
  FuzzyRule *rule1 = new FuzzyRule(1, NN, N);
  fuzzy1 -> addFuzzyRule(rule1);
  fuzzy2 -> addFuzzyRule(rule1);
  fuzzy3 -> addFuzzyRule(rule1);

  FuzzyRuleAntecedent *NP = new FuzzyRuleAntecedent();
  FuzzyRuleAntecedent *PN = new FuzzyRuleAntecedent();
  FuzzyRuleAntecedent *ZZ = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *Z = new FuzzyRuleConsequent();
  NP -> joinWithAND(e_N, de_P);
  PN -> joinWithAND(e_P, de_N);
  ZZ -> joinWithOR(NP, PN);
  Z -> addOutput(u_Z);
  FuzzyRule *rule2 = new FuzzyRule(2, ZZ, Z);
  fuzzy1 -> addFuzzyRule(rule2);
  fuzzy2 -> addFuzzyRule(rule2);
  fuzzy3 -> addFuzzyRule(rule2);

  FuzzyRuleAntecedent *PP = new FuzzyRuleAntecedent();
  FuzzyRuleConsequent *P = new FuzzyRuleConsequent();
  PP -> joinWithAND(e_P, de_P);
  P -> addOutput(u_P);
  FuzzyRule *rule3 = new FuzzyRule(3, PP, P);
  fuzzy1 -> addFuzzyRule(rule3);
  fuzzy2 -> addFuzzyRule(rule3);
  fuzzy3 -> addFuzzyRule(rule3);

}

void Fuzzy()
{
  position1 = bo_loc1.updateEstimate(encoderMotor1.read())*360/CPR_ENCODER;
  E01 = (positionSet1 - position1)/10;
  DE1 = (E01 - E1e)/T/150;
  fuzzy1 -> setInput(1, E01);
  fuzzy1 -> setInput(2, DE1);
  fuzzy1 -> fuzzify();
  Output1 = fuzzy1 -> defuzzify(1);
  LastOutput1 = Output1*20;
  if(LastOutput1>0) dirMotor1=0;
  else dirMotor1=1;
  // if(LastOutput1<100 && LastOutput1>-100) LastOutput1=0;
  // if(LastOutput1<0) LastOutput1=abs(LastOutput1);
  E1e = E01;

  position2 = bo_loc2.updateEstimate(encoderMotor2.read())*360/CPR_ENCODER;
  E02 = (positionSet2 - position2)/10;
  DE2 = (E02 - E2e)/T/150;
  fuzzy2 -> setInput(1, E02);
  fuzzy2 -> setInput(2, DE2);
  fuzzy2 -> fuzzify();
  Output2 = fuzzy2 -> defuzzify(1);
  LastOutput2 = Output2*20;
  if(LastOutput2>0) dirMotor2=0;
  else dirMotor2=1;
  // if(LastOutput2<100 && LastOutput2>-100) LastOutput2=0;
  // if(LastOutput2<0) LastOutput2=abs(LastOutput2);
  E2e = E02;

  position3 = bo_loc3.updateEstimate(encoderMotor3.read())*360/CPR_ENCODER;
  E03 = (positionSet3 - position3)/10;
  DE3 = (E03 - E3e)/T/150;
  fuzzy3 -> setInput(1, E03);
  fuzzy3 -> setInput(2, DE3);
  fuzzy3 -> fuzzify();
  Output3 = fuzzy3 -> defuzzify(1);
  LastOutput3 = Output3*20;
  if(LastOutput3>0) dirMotor3=0;
  else dirMotor3=1;
  // if(LastOutput3<100 && LastOutput3>-100) LastOutput3=0;
  // if(LastOutput3<0) LastOutput3=abs(LastOutput3);
  E3e = E03;

}


