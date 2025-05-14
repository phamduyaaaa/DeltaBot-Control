# Kịch bản thử nghiệm

## === Thông số cơ khí ===
   
e = 77.17 (Khoảng cách giữa các khớp nối của end effector)   
f = 175.0 (Khoảng cách giữa các khớp nối của khung dưới)   
re = 297.0 (Chiều dài thanh nối dưới)   
rf = 170.0 (Chiều dài thanh nối trên)   

## === Setup Chân ===
   
#define DIR1 29   
#define PWM1 9   
#define EN1 27   
#define DIR2 35   
#define PWM2 7   
#define EN2 33   
#define DIR3 37   
#define PWM3 5   
#define EN3 31   
   
## === PID ===
    
#define Kp 6.67    
#define Kd 0.02    
#define Ki 3.33    

## === Công tắc hành trình ===

#define switch1Pin 50   
#define switch2Pin 41   
#define switch3Pin 39   
   
## === Fuzzy ===
   
FuzzySet *e_N = new FuzzySet(-1, -1, -0.5, 0.01);   
FuzzySet *e_P = new FuzzySet(-0.01, 0.5, 1, 1);   
FuzzySet *de_N = new FuzzySet(-1, -1, -0.5, 0.25);   
FuzzySet *de_P = new FuzzySet(-0.25, 0.50, 1, 1);   
   
## === Encoder ===

float CPR_ENCODER = 4000;   



