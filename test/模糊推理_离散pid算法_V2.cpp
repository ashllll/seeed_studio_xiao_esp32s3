#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <U8g2lib.h>

// MAX31865引脚定义
#define CS_PIN 44
#define MOS_PIN 43
#define MAX_TEMPERATURE 80.0 // 最大安全温度
#define MIN_TEMPERATURE 0.0  // 最小安全温度

// OLED显示屏初始化
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 6, /* data=*/ 5);

// MAX31865传感器初始化
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN);

// MOS管控制引脚
const int mosPin = MOS_PIN;

// PID参数初始化
float Kp = 10.0;
float Ki = 0.22;
float Kd = 100.0;
float setpoint = 37.0; // 目标温度
float integral = 0.0;
float previous_error = 0.0;

// 模糊逻辑参数
float deltaKp = 0.0;
float deltaKi = 0.0;
float deltaKd = 0.0;

// 模糊集合定义。PS_S定义冲突，s
enum FuzzySet { NB, NM, NS, ZO, PS_s, PM, PB };

// 三角形隶属度函数
float triangle(float x, float a, float b, float c) {
  return max(min((x - a) / (b - a), (c - x) / (c - b)), 0.0f);
}

// 隶属度计算
FuzzySet fuzzify(float x) {
  if (x <= -2) return NB;
  if (x <= -1) return NM;
  if (x <= 0) return NS;
  if (x <= 1) return ZO;
  if (x <= 2) return PS_s;
  if (x <= 3) return PM;
  return PB;
}

// 模糊规则查找
void fuzzyRule(FuzzySet e, FuzzySet c, float &deltaKp, float &deltaKi, float &deltaKd) {
  // 模糊规则表
  const float rules[7][7][3] = {
    { {0.3, -3, 0.03}, {0.3, -2, 0.02}, {0.3, -1, 0.01}, {0.3, 0, 0}, {0.2, 1, -0.01}, {0.1, 2, -0.02}, {0.0, 3, -0.03} },
    { {0.2, -3, 0.03}, {0.2, -2, 0.02}, {0.2, -1, 0.01}, {0.2, 0, 0}, {0.1, 1, -0.01}, {0.0, 2, -0.02}, {-0.1, 3, -0.03} },
    { {0.1, -3, 0.03}, {0.1, -2, 0.02}, {0.1, -1, 0.01}, {0.1, 0, 0}, {0.0, 1, -0.01}, {-0.1, 2, -0.02}, {-0.2, 3, -0.03} },
    { {0.0, -3, 0.03}, {0.0, -2, 0.02}, {0.0, -1, 0.01}, {0.0, 0, 0}, {-0.1, 1, -0.01}, {-0.2, 2, -0.02}, {-0.3, 3, -0.03} },
    { {-0.1, -3, 0.03}, {-0.1, -2, 0.02}, {-0.1, -1, 0.01}, {-0.1, 0, 0}, {-0.2, 1, -0.01}, {-0.3, 2, -0.02}, {-0.4, 3, -0.03} },
    { {-0.2, -3, 0.03}, {-0.2, -2, 0.02}, {-0.2, -1, 0.01}, {-0.2, 0, 0}, {-0.3, 1, -0.01}, {-0.4, 2, -0.02}, {-0.5, 3, -0.03} },
    { {-0.3, -3, 0.03}, {-0.3, -2, 0.02}, {-0.3, -1, 0.01}, {-0.3, 0, 0}, {-0.4, 1, -0.01}, {-0.5, 2, -0.02}, {-0.6, 3, -0.03} }
  };

  deltaKp = rules[e][c][0];
  deltaKi = rules[e][c][1];
  deltaKd = rules[e][c][2];
}

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);

  // 初始化显示屏
  if (!u8g2.begin()) {
    Serial.println("Failed to initialize OLED display!");
    while (1); // 死循环，等待用户干预
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "Initializing...");
  u8g2.sendBuffer();

  // 初始化MAX31865
  if (!max31865.begin(MAX31865_3WIRE)) { // 三线模式
    Serial.println("Failed to initialize MAX31865!");
    while (1); // 死循环，等待用户干预
  }

  // 初始化MOS管控制引脚
  pinMode(mosPin, OUTPUT);
  digitalWrite(mosPin, LOW);
}

void loop() {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 1000;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // 读取温度
    float temperature = max31865.temperature(100, 430); // PT100, 430 Ohm reference resistor
    if (isnan(temperature)) {
      handleTemperatureError();
      return;
    }

    // 检查温度是否在安全范围内
    if (temperature > MAX_TEMPERATURE || temperature < MIN_TEMPERATURE) {
      Serial.println("Temperature out of safe range!");
      analogWrite(mosPin, 0); // 关闭加热器
      return;
    }

    // 计算误差和误差变化率
    float error = setpoint - temperature;
    float derivative = error - previous_error;

    // 模糊化处理
    FuzzySet fuzzyError = fuzzify(error);
    FuzzySet fuzzyDerivative = fuzzify(derivative);

    // 模糊推理调整PID参数
    fuzzyRule(fuzzyError, fuzzyDerivative, deltaKp, deltaKi, deltaKd);

    // 调整PID参数
    Kp = constrain(Kp + deltaKp, 0.0, 100.0);
    Ki = constrain(Ki + deltaKi, 0.0, 10.0);
    Kd = constrain(Kd + deltaKd, 0.0, 1000.0);

    // PID计算
    integral += error;
    integral = constrain(integral, -100, 100); // 积分限幅
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // 控制MOS管（PTC加热）
    int pwmOutput = constrain(map(output, 0, 100, 0, 255), 0, 255); // 假设output范围是0到100
    analogWrite(mosPin, pwmOutput);

    // 在串口监视器上打印温度值
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" C, Output: ");
    Serial.print(output);
    Serial.print(", Kp: ");
    Serial.print(Kp);
    Serial.print(", Ki: ");
    Serial.print(Ki);
    Serial.print(", Kd: ");
    Serial.println(Kd);

    // 在OLED显示屏上显示温度
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 10);
    u8g2.print("Temp: ");
    u8g2.print(temperature);
    u8g2.print(" C");
    u8g2.setCursor(0, 25);
    u8g2.print("PWM: ");
    u8g2.print(pwmOutput);
    u8g2.sendBuffer();
  }
}

void handleTemperatureError() {
  Serial.println("Error reading temperature!");
  // 尝试重新读取温度
  float temperature = max31865.temperature(100, 430);
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature again. Shutting down heater.");
    analogWrite(mosPin, 0); // 关闭加热器
    return;
  }
}
