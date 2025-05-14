# Kịch bản thử nghiệm

<div align="center">

<h1 style="font-size: 50px;">Thông số cơ khí ⚡ </h1>

</div>
   
e = 77.17 (Khoảng cách giữa các khớp nối của end effector)   
f = 175.0 (Khoảng cách giữa các khớp nối của khung dưới)   
re = 297.0 (Chiều dài thanh nối dưới)   
rf = 170.0 (Chiều dài thanh nối trên)   

<div align="center">

<h1 style="font-size: 50px;">Setup Chân ⚡</h1>

</div>
   
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
   
<div align="center">

<h1 style="font-size: 50px;">PID ⚡</h1>

</div>
    
#define Kp 6.67    
#define Kd 0.02    
#define Ki 3.33    

<div align="center">

<h1 style="font-size: 50px;">Công tắc hành trình ⚡</h1>

</div>

#define switch1Pin 50   
#define switch2Pin 41   
#define switch3Pin 39   
   
<div align="center">

<h1 style="font-size: 50px;">Fuzzy ⚡</h1>

</div>
   
FuzzySet *e_N = new FuzzySet(-1, -1, -0.5, 0.01);   
FuzzySet *e_P = new FuzzySet(-0.01, 0.5, 1, 1);   
FuzzySet *de_N = new FuzzySet(-1, -1, -0.5, 0.25);   
FuzzySet *de_P = new FuzzySet(-0.25, 0.50, 1, 1);   
   
<div align="center">

<h1 style="font-size: 50px;">Encoder ⚡</h1>

</div>

float CPR_ENCODER = 4000;   



