#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include "GeneticAlgorithm.h"

// 引脚定义
#define CS_PIN 44
#define MOS_PIN 43

// OLED显示屏初始化
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);

// MAX31865初始化
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN);

// PT100是一个3线RTD传感器
#define RREF 430.0
#define RNOMINAL 100.0

// PID控制器
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void controlTask(void *pvParameters);

void setup() {
  Serial.begin(115200);

  // 初始化MAX31865
  max31865.begin(MAX31865_3WIRE);

  // 初始化MOS管控制引脚
  pinMode(MOS_PIN, OUTPUT);

  // 初始化OLED显示屏
  u8g2.begin();

  // 设置目标温度
  Setpoint = 60.0;

  // 初始化PID控制器
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // 限制输出范围为0到255

  // 创建遗传算法任务，运行在核心0
  xTaskCreatePinnedToCore(
    geneticAlgorithmTask,  // 任务函数
    "GeneticAlgorithm",    // 任务名称
    10000,                 // 堆栈大小
    NULL,                  // 任务参数
    1,                     // 优先级
    NULL,                  // 任务句柄
    0                      // 核心ID
  );

  // 创建主控制任务，运行在核心1
  xTaskCreatePinnedToCore(
    controlTask,           // 任务函数
    "Control",             // 任务名称
    10000,                 // 堆栈大小
    NULL,                  // 任务参数
    1,                     // 优先级
    NULL,                  // 任务句柄
    1                      // 核心ID
  );
}

void loop() {
  // 空的loop函数，因为所有的操作都在任务中进行
}

// 控制任务
void controlTask(void *pvParameters) {
  while (true) {
    // 读取当前温度
    Input = max31865.temperature(RNOMINAL, RREF);

    // 计算PID输出
    myPID.Compute();

    // 抗饱和控制
    if (Output > 255) {
      Output = 255;
    } else if (Output < 0) {
      Output = 0;
    }

    // 控制MOS管的PWM输出
    analogWrite(MOS_PIN, Output);

    // 更新OLED显示屏
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "CT:"); // Current Temperature
    u8g2.setCursor(30, 10);
    u8g2.print(Input);
    u8g2.drawStr(0, 30, "TT:"); // Target Temperature
    u8g2.setCursor(30, 30);
    u8g2.print(Setpoint);
    u8g2.drawStr(0, 50, "PWM:");
    u8g2.setCursor(30, 50);
    u8g2.print(Output);
    u8g2.sendBuffer();

    // 延时一段时间
    delay(1000);
  }
}