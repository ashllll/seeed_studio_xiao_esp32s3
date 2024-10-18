#include <U8g2lib.h>
#include <Adafruit_MAX31865.h>
#include <QuickPID.h>
#include <AiEsp32RotaryEncoder.h>

// 定义引脚和常量
#define CS_PIN  44
#define MOS_PIN 14
#define MIN_TEMPERATURE 0.0f
#define MAX_TEMPERATURE 500.0f
#define MAX_HEATING_TEMPERATURE 150.0f
#define UPDATE_INTERVAL 200
#define DEBOUNCE_INTERVAL 50
#define DOUBLE_CLICK_INTERVAL 300
#define ENCODER_A 2
#define ENCODER_B 3
#define ENCODER_SW 4
#define OVERHEAT_THRESHOLD 160.0f
#define MAX_TEMPERATURE_CHANGE 5.0f
#define HEATING_TIMEOUT 60000 // 60 seconds

// 使用PROGMEM存储字符串
const char ERROR_MSG[] PROGMEM = "Error: ";
const char SET_MSG[] PROGMEM = "SET";
const char ON_MSG[] PROGMEM = "ON";
const char OFF_MSG[] PROGMEM = "OFF";

// 初始化OLED显示器 (SSD1306 128x64, SDA=5, SCL=6)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// 初始化MAX31865温度传感器
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN);

// PID控制相关变量
float setpoint = 80.0f; // 目标温度
float input = 0.0f;     // 当前温度
float output = 0.0f;    // PID输出
float Kp = 20.0f;       // 比例系数
float Ki = 0.5f;        // 积分系数
float Kd = 50.0f;       // 微分系数
// 初始化PID控制器
QuickPID myPID(&input, &output, &setpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// 控制和状态变量
volatile bool startHeating = false; // 是否开始加热
bool errorState = false;            // 是否处于错误状态
String currentError = "";           // 当前错误信息

// 初始化旋转编码器
AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_A, ENCODER_B, ENCODER_SW, -1);
unsigned long lastClickTime = 0;        // 上次点击时间
unsigned long lastButtonPressTime = 0;  // 上次按钮按下时间
bool lastButtonState = HIGH;            // 上次按钮状态
bool waitingForSecondClick = false;     // 是否等待第二次点击

// PID调整相关变量
float previousError = 0.0f;
float errorSum = 0.0f;

// 动画相关变量
float displayedTemperature = 0.0f;
float displayedPWM = 0.0f;
const float animationSpeed = 0.2f; // 动画速度，可调整

// 显示更新相关变量
char prevTempStr[8] = "";
char prevPercentStr[5] = "";
int prevBarWidth = -1;
float prevSetpoint = -1;
bool prevHeatingStatus = false;

// 新增变量
float lastTemperature = 0.0f;
unsigned long heatingStartTime = 0;
unsigned long lastValidTemperatureTime = 0;

// 函数声明
void updateDisplay(float temperature, float pwm, bool isHeating);
void adjustPIDParameters(float error);
void displayError(const char* message);
bool checkSensorConnection();
bool checkOverheat(float temperature);
bool checkTemperatureChange(float temperature);
bool checkHeatingElement();
void performSystemCheck();

// 旋转编码器中断服务函数
void IRAM_ATTR readEncoderISR() {
  encoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);  // 设置默认字体

  // 初始化MAX31865温度传感器
  if (!max31865.begin(MAX31865_3WIRE)) {
    displayError(PSTR("MAX31865 init failed"));
    while (true); // 初始化失败，停止执行
  }

  // 设置MOS管引脚为PWM输出
  pinMode(MOS_PIN, OUTPUT);
  ledcAttachPin(MOS_PIN, 1);
  ledcSetup(1, 2500, 8);

  // 初始化旋转编码器
  encoder.begin();
  encoder.setup(readEncoderISR);
  encoder.setBoundaries(MIN_TEMPERATURE, MAX_TEMPERATURE, false);
  encoder.setAcceleration(0);

  // 设置PID控制器
  myPID.SetMode(QuickPID::Control::automatic);
  myPID.SetOutputLimits(0, 255);

  performSystemCheck();
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  // 读取编码器位置并更新目标温度
  long encoderValue = encoder.readEncoder();
  setpoint = constrain(encoderValue, MIN_TEMPERATURE, MAX_HEATING_TEMPERATURE);

  // 处理编码器按钮点击事件
  bool buttonState = digitalRead(ENCODER_SW);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW && (now - lastButtonPressTime) > DEBOUNCE_INTERVAL) {
      if (waitingForSecondClick && (now - lastButtonPressTime) <= DOUBLE_CLICK_INTERVAL) {
        startHeating = !startHeating;
        waitingForSecondClick = false;
        if (startHeating) {
          heatingStartTime = now; // 仅在开始加热时记录时间
        }
      } else {
        waitingForSecondClick = true;
        lastButtonPressTime = now;
      }
    }
    lastButtonState = buttonState;
  }

  // 检查是否超过双击间隔
  if (waitingForSecondClick && (now - lastButtonPressTime) > DOUBLE_CLICK_INTERVAL) {
    waitingForSecondClick = false;
    startHeating = true;
    heatingStartTime = now; // 记录时间
  }

  // 定期更新温度和显示
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;

    errorState = false;
    currentError = "";

    if (!checkSensorConnection()) {
      currentError = PSTR("Sensor disconnected");
      errorState = true;
      startHeating = false;
    } else {
      float temperature = max31865.temperature(100.0, 430.0);
      
      if (checkOverheat(temperature)) {
        currentError = PSTR("Overheat detected");
        errorState = true;
        startHeating = false;
      } else if (checkTemperatureChange(temperature)) {
        currentError = PSTR("Abnormal temp change");
        errorState = true;
        startHeating = false;
      } else if (checkHeatingElement()) {
        currentError = PSTR("Heating element fault");
        errorState = true;
        startHeating = false;
      } else if (startHeating && (now - heatingStartTime > HEATING_TIMEOUT)) {
        currentError = PSTR("Heating timeout");
        errorState = true;
        startHeating = false;
      }

      if (startHeating && !errorState) {
        input = temperature;
        myPID.Compute(); // 计算PID输出

        // 确保输出在有效范围内，避免输入过高
        output = (temperature >= MAX_HEATING_TEMPERATURE) ? 0 : output;
        
        ledcWrite(1, constrain(output, 0, 255)); // 设置PWM输出并确保在范围内
        adjustPIDParameters(setpoint - temperature); // 调整PID参数
      } else {
        ledcWrite(1, 0); // 停止加热
        output = 0;
      }

      if (errorState) {
        displayError(currentError.c_str());
      } else {
        updateDisplay(temperature, output, startHeating); // 更新显示
      }
      
      lastTemperature = temperature; // 更新最后的有效温度
      lastValidTemperatureTime = now;
    }
  }
}

void updateDisplay(float temperature, float pwm, bool isHeating) {
  bool needFullUpdate = false;

  displayedTemperature += (temperature - displayedTemperature) * animationSpeed;
  displayedPWM += (pwm - displayedPWM) * animationSpeed;

  char tempStr[8];
  snprintf_P(tempStr, sizeof(tempStr), PSTR("%.1f"), displayedTemperature);

  char percentStr[5];
  snprintf_P(percentStr, sizeof(percentStr), PSTR("%d%%"), static_cast<int>((displayedPWM / 255.0f) * 100.0f));

  int barWidth = static_cast<int>((displayedPWM / 255.0f) * 100);

  // 检查是否需要全屏更新
  if (strcmp(tempStr, prevTempStr) != 0 || strcmp(percentStr, prevPercentStr) != 0 || 
      barWidth != prevBarWidth || setpoint != prevSetpoint || isHeating != prevHeatingStatus) {
    needFullUpdate = true;
  }

  if (needFullUpdate) {
    u8g2.clearBuffer();

    // 显示设定温度
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 10);
    u8g2.print(static_cast<int>(setpoint));

    // 显示SET字符
    u8g2.setCursor(0, 25);
    u8g2.print(SET_MSG);

    // 显示加热状态
    u8g2.setCursor(0, 40);
    u8g2.print(isHeating ? ON_MSG : OFF_MSG);

    // 显示当前温度（居中对齐，放大字体）
    u8g2.setFont(u8g2_font_ncenB24_tr); // 使用更大的字体
    int tempWidth = u8g2.getStrWidth(tempStr);
    u8g2.setCursor(((128 - tempWidth) / 2) + 9, 35); // 右移9像素，上移5像素（从40改为35）
    u8g2.print(tempStr);
    u8g2.setFont(u8g2_font_ncenB08_tr); // 恢复小字体
    u8g2.print("C");

    // 计算进度条的位置
    int barStartX = ((128 - 100) / 2) + 9;
    
    // 显示百分比（对齐进度条左侧）
    u8g2.setCursor(barStartX - 18, 60);
    u8g2.print(percentStr);

    // 绘制进度条（居中对齐，总宽度100像素，右移9像素）
    u8g2.drawFrame(barStartX, 50, 100, 10);
    if (barWidth > 0) {
      u8g2.drawBox(barStartX, 50, barWidth, 10);
    }

    u8g2.sendBuffer();

    // 更新先前的值
    strcpy(prevTempStr, tempStr);
    strcpy(prevPercentStr, percentStr);
    prevBarWidth = barWidth;
    prevSetpoint = setpoint;
    prevHeatingStatus = isHeating;
  }
}

void adjustPIDParameters(float error) {
  static unsigned long lastAdjustTime = 0;
  unsigned long now = millis();

  // 限制调整频率
  if (now - lastAdjustTime < 500) {
    return;
  }
  lastAdjustTime = now;

  float errorChange = error - previousError;
  previousError = error;

  // 根据误差大小调整PID参数
  if (abs(error) > 5.0f) {
    Kp += 0.2f;
    Ki -= 0.02f;
    Kd += 1.0f;
  } else {
    Kp -= 0.2f;
    Ki += 0.02f;
    Kd -= 1.0f;
  }

  // 累积误差并调整PID参数
  errorSum += error;
  errorSum = constrain(errorSum, -100.0f, 100.0f);
  Kp += errorSum * 0.01f;
  Kd += errorChange * 0.1f;

  // 限制PID参数范围
  Kp = constrain(Kp, 10.0f, 30.0f);
  Ki = constrain(Ki, 0.1f, 1.0f);
  Kd = constrain(Kd, 20.0f, 60.0f);

  // 更新PID控制器参数
  myPID.SetTunings(Kp, Ki, Kd);
}

void displayError(const char* message) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print(ERROR_MSG);
  u8g2.setCursor(0, 25);
  u8g2.print(message);
  u8g2.sendBuffer();
}

bool checkSensorConnection() {
  uint16_t rtd = max31865.readRTD();
  if (rtd == 0 || rtd == 65535) {
    return false;
  }
  return true;
}

bool checkOverheat(float temperature) {
  return temperature > OVERHEAT_THRESHOLD;
}

bool checkTemperatureChange(float temperature) {
  return abs(temperature - lastTemperature) > MAX_TEMPERATURE_CHANGE;
}

bool checkHeatingElement() {
  unsigned long now = millis();
  return startHeating && now - lastValidTemperatureTime > 5000 && input < setpoint - 5.0f;
}

void performSystemCheck() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print(F("System check..."));
  u8g2.sendBuffer();

  delay(1000);

  if (!checkSensorConnection()) {
    displayError(PSTR("Sensor check failed"));
    return;
  }

  ledcWrite(1, 128);
  delay(1000);
  ledcWrite(1, 0);

  float testTemp = max31865.temperature(100.0, 430.0);
  if (testTemp < 0 || testTemp > MAX_HEATING_TEMPERATURE) {
    displayError(PSTR("Temp reading invalid"));
    return;
  }

  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print(F("System check OK"));
  u8g2.sendBuffer();
  delay(1000);
}