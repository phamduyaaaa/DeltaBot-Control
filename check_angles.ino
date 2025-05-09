const int stepPins[3]   = {2, 6, 10};
const int dirPins[3]    = {3, 7, 11};
const int enablePins[3] = {4, 8, 12};

float currentAngles[3] = {0, 0, 0};
float stepAngle = 1.8;

String inputString = "";

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);
  }
  Serial.println("Nhap 3 goc cach nhau bang dau cach, vi du: 30 -45 60");
}

void loop() {
  // Nhập dữ liệu từ Serial
  if (Serial.available()) {
    char inChar = Serial.read();

    // Nếu là xuống dòng => xử lý input
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        float angles[3];
        if (parseAngles(inputString, angles)) {
          moveMotors(angles);
        } else {
          Serial.println("Loi: Dinh dang sai. Vi du dung: 30 -45 60");
        }
        inputString = "";
      }
    } else {
      inputString += inChar;
    }
  }
}

bool parseAngles(String str, float outAngles[3]) {
  str.trim();  // Xóa khoảng trắng đầu/cuối
  int index = 0;

  for (int i = 0; i < 3; i++) {
    int spaceIndex = str.indexOf(' ');
    String token;
    if (spaceIndex != -1) {
      token = str.substring(0, spaceIndex);
      str = str.substring(spaceIndex + 1);
    } else {
      token = str;
      str = "";
    }

    token.trim();
    if (token.length() == 0 || !isValidNumber(token)) return false;

    outAngles[i] = token.toFloat();
    index++;
  }

  return (index == 3);
}

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

void moveMotors(float targetAngles[3]) {
  for (int i = 0; i < 3; i++) {
    float delta = targetAngles[i] - currentAngles[i];
    int steps = abs(delta) / stepAngle;
    digitalWrite(dirPins[i], delta > 0 ? HIGH : LOW);

    for (int j = 0; j < steps; j++) {
      digitalWrite(stepPins[i], HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPins[i], LOW);
      delayMicroseconds(1000);
    }

    currentAngles[i] = targetAngles[i];
  }

  Serial.print("Da di chuyen toi: ");
  Serial.print(targetAngles[0]); Serial.print(" ");
  Serial.print(targetAngles[1]); Serial.print(" ");
  Serial.println(targetAngles[2]);
}
