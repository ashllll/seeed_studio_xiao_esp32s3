#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <U8g2lib.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(44, 9, 8, 7);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
#define RNOMINAL 100.0

// Initial PID parameters
float Kp = 2.0, Ki = 0.1, Kd = 0.1;
float integral = 0;
float lastError = 0;
float maxIntegral = 10; // 积分饱和限值
float errorThreshold = 1.0; // 误差门限值

// Temperature setpoint
float setpoint = 80.0;
int mosPin = 43; // 修改MOS管控制引脚为43

bool isHeating = false;
unsigned long startTime;
float lastTemperature;
float prevError1 = 0, prevError2 = 0;

// U8G2 OLED setup
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Function prototypes
void startHeating();
int pidControl(float setpoint, float temperature);
void drawDisplay();
void autoTunePID(float currentTemp, float setpoint);
void handleFault(uint8_t fault);
int mapToPWM(float pidOutput);
void checkTemperature(float temperature);
void temperatureTask(void *parameter);
void displayTask(void *parameter);

// Shared variables
float currentTemperature = 0;
int pwmOutput = 0;

// Auto-tune variables
bool autoTuning = true;
float ultimateGain = 0;
float oscillationPeriod = 0;
unsigned long lastTime = 0;
bool increasing = true;

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE或4WIRE as necessary

  pinMode(mosPin, OUTPUT);

  // Initialize the OLED display
  u8g2.begin();

  // Create tasks
  xTaskCreatePinnedToCore(temperatureTask, "Temperature Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "Display Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // The loop function is empty because all work is done in tasks
}

void startHeating() {
  isHeating = true;
  startTime = millis();
  lastTemperature = thermo.temperature(RNOMINAL, RREF);

  Serial.println("Heating started");
}

int pidControl(float setpoint, float temperature) {
  unsigned long now = millis();
  float error = setpoint - temperature;

  // 防止积分项过大，并引入抗积分饱和技术
  if (abs(error) > errorThreshold) {
    integral += error * (now - startTime) / 1000.0;
    if (integral > maxIntegral) {
      integral = maxIntegral;
    } else if (integral < -maxIntegral) {
      integral = -maxIntegral;
    }
  }

  float derivative = (error - 2 * prevError1 + prevError2) / ((now - startTime) / 1000.0);
  float output = Kp * error + Ki * integral + Kd * derivative;

  // 更新误差
  prevError2 = prevError1;
  prevError1 = error;

  // 打印详细信息用于调试
  Serial.print("Error: "); Serial.println(error);
  Serial.print("Kp * error: "); Serial.println(Kp * error);
  Serial.print("Ki * integral: "); Serial.println(Ki * integral);
  Serial.print("Kd * derivative: "); Serial.println(Kd * derivative);

  output = constrain(output, 0, 255); // 确保输出在0到255之间
  lastError = error;
  startTime = now;

  // 映射到8段PWM值
  return mapToPWM(output);
}

int mapToPWM(float pidOutput) {
  const int segments = 8;
  const int segmentSize = 255 / segments;
  
  int pwmValue = (int)(pidOutput / segmentSize);
  pwmValue = constrain(pwmValue, 0, segments - 1); // 确保pwmValue在0到7之间
  return pwmValue * segmentSize;
}

void autoTunePID(float currentTemp, float setpoint) {
  // Ziegler-Nichols方法自校准PID参数
  static float maxTemp = -1000, minTemp = 1000;
  static unsigned long highTime, lowTime;

  if (autoTuning) {
    // Adjust the heating power to find the ultimate gain and oscillation period
    pwmOutput = increasing ? 255 : 0;
    analogWrite(mosPin, pwmOutput);

    if (currentTemp > maxTemp) {
      maxTemp = currentTemp;
      highTime = millis();
    } else if (currentTemp < minTemp) {
      minTemp = currentTemp;
      lowTime = millis();
    }

    if (increasing && currentTemp > setpoint + 10) {
      increasing = false;
      oscillationPeriod = (lowTime - lastTime) / 1000.0;
      ultimateGain = (4.0 / (maxTemp - minTemp)) * (setpoint / (pwmOutput - 0));
      Serial.println("Switching to decreasing");
    } else if (!increasing && currentTemp < setpoint - 10) {
      increasing = true;
      oscillationPeriod = (highTime - lastTime) / 1000.0;
      ultimateGain = (4.0 / (maxTemp - minTemp)) * (setpoint / (0 - pwmOutput));
      Serial.println("Switching to increasing");
    }

    lastTime = millis();

    if (oscillationPeriod > 0) {
      // Calculate PID parameters using Ziegler-Nichols method
      Kp = 0.6 * ultimateGain;
      Ki = 2 * Kp / oscillationPeriod;
      Kd = Kp * oscillationPeriod / 8;

      // Print calculated PID parameters
      Serial.print("Ultimate Gain: "); Serial.println(ultimateGain);
      Serial.print("Oscillation Period: "); Serial.println(oscillationPeriod);
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Ki: "); Serial.println(Ki);
      Serial.print("Kd: "); Serial.println(Kd);

      autoTuning = false;
    }
  }
}

void drawDisplay() {
  u8g2.clearBuffer(); // Clear the internal memory

  // Draw current temperature
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("Current Temp: ");
  u8g2.print(currentTemperature, 1);
  u8g2.print(" C");

  // Draw target temperature
  u8g2.setCursor(0, 25);
  u8g2.print("Target Temp: ");
  u8g2.print(setpoint, 1);
  u8g2.print(" C");

  // Draw PID state
  u8g2.setCursor(0, 40);
  u8g2.print("PID Output: ");
  u8g2.print(pwmOutput);

  // Draw temperature status bar
  int tempBarWidth = map(currentTemperature, 0, 100, 0, 64); // Map temperature to bar width
  u8g2.drawFrame(0, 50, 64, 10); // Draw the frame of the bar
  u8g2.drawBox(0, 50, tempBarWidth, 10); // Draw the filled part of the bar

  // Draw PWM status bar
  int pwmBarWidth = map(pwmOutput, 0, 255, 0, 64); // Map PWM output to bar width
  u8g2.drawFrame(64, 50, 64, 10); // Draw the frame of the bar
  u8g2.drawBox(64, 50, pwmBarWidth, 10); // Draw the filled part of the bar

  u8g2.sendBuffer(); // Transfer internal memory to the display
}

void handleFault(uint8_t fault) {
  Serial.print("Fault 0x"); Serial.println(fault, HEX);
  if (fault & MAX31865_FAULT_HIGHTHRESH) {
    Serial.println("RTD High Threshold");
  }
  if (fault & MAX31865_FAULT_LOWTHRESH) {
    Serial.println("RTD Low Threshold");
  }
  if (fault & MAX31865_FAULT_REFINLOW) {
    Serial.println("REFIN- > 0.85 x Bias");
  }
    if (fault & MAX31865_FAULT_REFINHIGH) {
    Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
  }
  if (fault & MAX31865_FAULT_RTDINLOW) {
    Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
  }
  if (fault & MAX31865_FAULT_OVUV) {
    Serial.println("Under/Over voltage");
  }
}

void checkTemperature(float temperature) {
  if (temperature < 15.0 || temperature > 100.0) {
    Serial.println("Temperature out of range (15-100 degrees). Shutting down PWM and restarting...");
    analogWrite(mosPin, 0); // 关闭PWM输出
    delay(1000); // 延迟一秒以确保PWM完全关闭
    ESP.restart(); // 重启ESP32
  }
}

void temperatureTask(void *parameter) {
  while (1) {
    uint16_t rtd = thermo.readRTD();
    uint8_t fault = thermo.readFault();

    if (fault) {
      handleFault(fault);
      thermo.clearFault();
      delay(1000);
      continue;
    }

    float ratio = rtd;
    ratio /= 32768;
    float resistance = RREF * ratio;
    float temperature = thermo.temperature(RNOMINAL, RREF);

    // 检查温度是否异常
    checkTemperature(temperature);

    // 滤波处理
    static float lastTemperature = temperature;
    temperature = 0.9 * lastTemperature + 0.1 * temperature;
    lastTemperature = temperature;

    currentTemperature = temperature;

    if (!isHeating) {
      // Start heating
      startHeating();
    } else {
      // Auto-tune PID parameters
      autoTunePID(temperature, setpoint);

      // PID control
      pwmOutput = pidControl(setpoint, temperature);

      // Print PID output for debugging
      Serial.print("PID Output: "); Serial.println(pwmOutput);

      // Control the MOSFET with PWM
      analogWrite(mosPin, pwmOutput);
    }

    delay(100); // Adjust the delay as needed for optimal performance
  }
}

void displayTask(void *parameter) {
  while (1) {
    drawDisplay();
    delay(100); // Adjust the delay as needed for optimal performance
  }
}