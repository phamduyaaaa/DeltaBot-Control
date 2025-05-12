#include <DeltaKinematics.h>
#include <StringSplitter.h> // Thư viện hỗ trợ phân tích chuỗi

// === Khai báo chân điều khiển động cơ ===
const int stepPins[3] = {2, 6, 10};
const int dirPins[3]  = {3, 7, 11};
const int enablePins[3] = {4, 8, 12};

// === Thông số hình học robot Delta (tùy chỉnh theo thiết kế thực tế) ===
const float e = 77.17;   // khoảng cách giữa các khớp nối end effector (PlatformTri)
const float f = 175.0;  // khoảng cách giữa các khớp nối khung trên (BassTri)
const float re = 297.0;  // chiều dài thanh nối dưới (RodLength)
const float rf = 170.0;  // chiều dài thanh nối trên (ArmLength)

const float stepAngle = 0.9; // Độ mỗi bước >> 1.8 <<

DeltaKinematics delta(rf, re, f, e); // Khởi tạo đối tượng DeltaKinematics

String inputString = "";

// === SETUP ===
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);
  }
  Serial.println("Nhap vi tri x y z (mm), vi du: 0 0 -150");
}

// === LOOP ===
void loop() {
  if (Serial.available()) {
    char inChar = Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        StringSplitter splitter(inputString, ' ', 3);
        if (splitter.getItemCount() == 3) {
          float coords[3];
          bool valid = true;
          for (int i = 0; i < 3; i++) {
            String segment = splitter.getItemAtIndex(i);
            if (isValidNumber(segment)) {
              coords[i] = segment.toFloat();
            } else {
              valid = false;
              break;
            }
          }

          if (valid) {
            int result = delta.inverse(coords[0], coords[1], coords[2]);
            if (result == 1) { // Kiểm tra nếu không có lỗi
              float angles[3];
              angles[0] = delta.a;
              angles[1] = delta.b;
              angles[2] = delta.c;
              moveMotors(angles);
            } else {
              Serial.println("Ngoai tam lam viec hoac loi!");
            }
          } else {
            Serial.println("Loi: Dinh dang sai. Vi du dung: 0 0 -150");
          }
        } else {
          Serial.println("Loi: Can 3 gia tri x y z.");
        }
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }
}

// === VALIDATOR === (Giữ nguyên hàm này)
bool isValidNumber(String s) {
  if (s.length() == 0) return false;
  if (s.charAt(0) == '-' || s.charAt(0) == '+') s = s.substring(1);
  bool dotFound = false;
  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == '.') {
      if (dotFound) return false;
      dotFound = true;
    } else if (!isDigit(c)) {
      return false;
    }
  }
  return true;
}

// === ĐIỀU KHIỂN ĐỘNG CƠ === (Giữ nguyên hàm này)
void moveMotors(float targetAngles[3]) {
  static float currentAngles[3] = {0, 0, 0}; // Giữ giá trị góc hiện tại giữa các lần di chuyển
  for (int i = 0; i < 3; i++) {
    float delta = targetAngles[i] - currentAngles[i];
    int steps = round(abs(delta) / stepAngle); // Làm tròn số bước
    digitalWrite(dirPins[i], delta > 0 ? HIGH : LOW);

    for (int j = 0; j < steps; j++) {
      digitalWrite(stepPins[i], HIGH);
      delayMicroseconds(800);
      digitalWrite(stepPins[i], LOW);
      delayMicroseconds(800);
    }
    currentAngles[i] = targetAngles[i];
  }
  Serial.print("Da di chuyen toi goc: ");
  Serial.print(targetAngles[0]); Serial.print(" ");
  Serial.print(targetAngles[1]); Serial.print(" ");
  Serial.println(targetAngles[2]);
}
