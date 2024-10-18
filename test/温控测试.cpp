#include <U8g2lib.h>
#include <Adafruit_MAX31865.h>
#include <QuickPID.h>
#include <OneButton.h>

// 定义常量
const int CS_PIN = 44;
const int MOS_PIN = 43;
const int BUTTON_PIN_UP = 2;
const int BUTTON_PIN_DOWN = 3;
const int BUTTON_PIN_START = 4;
const int BUTTON_PIN_STOP = 5;
const float MIN_TEMPERATURE = 0.0;
const float MAX_TEMPERATURE = 80.0;
const unsigned long UPDATE_INTERVAL = 1000; // 用于更新的间隔

// OLED 显示屏定义
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 6, /* data=*/ 5);

// MAX31865 传感器定义
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN);

// PID 参数
float setpoint = 80.0; 
float input = 0.0;
float output = 0.0;
float Kp = 20.0;
float Ki = 0.5;
float Kd = 50.0;
QuickPID myPID(&input, &output, &setpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// 加热状态
volatile bool startHeating = false;

// OneButton 按钮定义
OneButton buttonUp(BUTTON_PIN_UP, true);
OneButton buttonDown(BUTTON_PIN_DOWN, true);
OneButton buttonStart(BUTTON_PIN_START, true);
OneButton buttonStop(BUTTON_PIN_STOP, true);

// 函数声明
void buttonUpClick();
void buttonDownClick();
void buttonStartClick();
void buttonStopClick();
void updateDisplay(float temperature);
void adjustPIDParameters(float error);

void setup() {
  Serial.begin(115200);
  u8g2.begin();
  if (!max31865.begin(MAX31865_3WIRE)) {
    Serial.println("MAX31865传感器初始化失败");
    // 闪烁LED或其他方式通知用户错误，并尝试重试初始化
    while (!max31865.begin(MAX31865_3WIRE)) {
      delay(1000);
    }
  }
  pinMode(MOS_PIN, OUTPUT);

  // 配置OneButton按钮
  buttonUp.attachClick(buttonUpClick);
  buttonDown.attachClick(buttonDownClick);
  buttonStart.attachClick(buttonStartClick);
  buttonStop.attachClick(buttonStopClick);

  // 配置PID控制器
  myPID.SetMode(QuickPID::Control::automatic);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_INTERVAL) { 
    lastUpdate = now;
    float temperature = max31865.temperature(100.0, 430.0);
    input = temperature; // 更新PID输入
    if (startHeating) {
      myPID.Compute(); // 计算PID输出
      analogWrite(MOS_PIN, output);
      adjustPIDParameters(setpoint - temperature); // 根据误差调整PID参数
    } else {
      analogWrite(MOS_PIN, 0);
    }
    updateDisplay(temperature);
  }

  // 处理按钮事件
  buttonUp.tick();
  buttonDown.tick();
  buttonStart.tick();
  buttonStop.tick();
}

// 按钮点击事件处理函数
void buttonUpClick() {
  setpoint = constrain(setpoint + 1.0, MIN_TEMPERATURE, MAX_TEMPERATURE);
}

void buttonDownClick() {
  setpoint = constrain(setpoint - 1.0, MIN_TEMPERATURE, MAX_TEMPERATURE);
}

void buttonStartClick() {
  startHeating = true;
}

void buttonStopClick() {
  startHeating = false;
}

void updateDisplay(float temperature) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("Temp: ");
  u8g2.print(temperature);
  u8g2.print("C");
  u8g2.setCursor(0, 20);
  u8g2.print("Setpoint: ");
  u8g2.print(setpoint);
  u8g2.print("C");
  u8g2.setCursor(0, 30);
  u8g2.print("Heating: ");
  u8g2.print(startHeating ? "ON" : "OFF");
  u8g2.sendBuffer();
}

void adjustPIDParameters(float error) {
  // 根据误差动态调整PID参数
  if (abs(error) > 5.0) {
    // 如果误差较大，增加Kp和Kd，减少Ki
    Kp += 0.1;
    Ki -= 0.01;
    Kd += 0.5;
  } else {
    // 如果误差较小，减少Kp和Kd，增加Ki
    Kp -= 0.1;
    Ki += 0.01;
    Kd -= 0.5;
  }
  // 确保PID参数在合理范围内
  Kp = constrain(Kp, 10.0, 30.0);
  Ki = constrain(Ki, 0.1, 1.0);
  Kd = constrain(Kd, 20.0, 60.0);
  // 更新PID控制器参数
  myPID.SetTunings(Kp, Ki, Kd);
}
