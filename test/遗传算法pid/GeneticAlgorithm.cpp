#include <Arduino.h>
#include <PID_v1.h>
#include "GeneticAlgorithm.h"

// 遗传算法参数
const int POP_SIZE = 10;  // 种群大小
const int NUM_GEN = 20;   // 代数
const float MUTATION_RATE = 0.1; // 变异率

// 种群
Individual population[POP_SIZE];

// 声明外部PID实例
extern PID myPID;

// 初始化种群
void initPopulation() {
  for (int i = 0; i < POP_SIZE; i++) {
    population[i].Kp = random(0, 10);
    population[i].Ki = random(0, 10);
    population[i].Kd = random(0, 10);
    population[i].fitness = 0;
  }
}

// 评估适应度
void evaluateFitness() {
  for (int i = 0; i < POP_SIZE; i++) {
    population[i].fitness = calculateFitness(population[i].Kp, population[i].Ki, population[i].Kd);
  }
}

// 计算个体的适应度
double calculateFitness(double Kp, double Ki, double Kd) {
  // 模拟一段时间内的系统响应，计算误差平方和
  double fitness = 0;
  double currentTemperature = 0;
  for (int t = 0; t < 100; t++) {
    // 假设使用一些方法得到当前温度和误差
    double error = 60.0 - currentTemperature; // 假设目标温度为60
    fitness += error * error;
    currentTemperature += Kp * error + Ki * error * t + Kd * (error - currentTemperature);
  }
  return -fitness; // 适应度为误差的负值，越小越好
}

// 选择
void selection() {
  // 简单的轮盘赌选择
  double totalFitness = 0;
  for (int i = 0; i < POP_SIZE; i++) {
    totalFitness += population[i].fitness;
  }
  Individual newPopulation[POP_SIZE];
  for (int i = 0; i < POP_SIZE; i++) {
    double r = random(0, totalFitness);
    double cumulativeFitness = 0;
    for (int j = 0; j < POP_SIZE; j++) {
      cumulativeFitness += population[j].fitness;
      if (cumulativeFitness >= r) {
        newPopulation[i] = population[j];
        break;
      }
    }
  }
  memcpy(population, newPopulation, sizeof(population));
}

// 交叉
void crossover() {
  for (int i = 0; i < POP_SIZE; i += 2) {
    if (random(0, 1) < 0.9) { // 90%的概率进行交叉
      double alpha = random(0, 1);
      double Kp1 = alpha * population[i].Kp + (1 - alpha) * population[i + 1].Kp;
      double Ki1 = alpha * population[i].Ki + (1 - alpha) * population[i + 1].Ki;
      double Kd1 = alpha * population[i].Kd + (1 - alpha) * population[i + 1].Kd;
      double Kp2 = alpha * population[i + 1].Kp + (1 - alpha) * population[i].Kp;
      double Ki2 = alpha * population[i + 1].Ki + (1 - alpha) * population[i].Ki;
      double Kd2 = alpha * population[i + 1].Kd + (1 - alpha) * population[i].Kd;
      population[i].Kp = Kp1;
      population[i].Ki = Ki1;
      population[i].Kd = Kd1;
      population[i + 1].Kp = Kp2;
      population[i + 1].Ki = Ki2;
      population[i + 1].Kd = Kd2;
    }
  }
}

// 变异
void mutate() {
  for (int i = 0; i < POP_SIZE; i++) {
    if (random(0, 1) < MUTATION_RATE) {
      population[i].Kp += random(-1, 1);
      population[i].Ki += random(-1, 1);
      population[i].Kd += random(-1, 1);
    }
  }
}

// 获取最优个体
Individual getBestIndividual() {
  Individual best = population[0];
  for (int i = 1; i < POP_SIZE; i++) {
    if (population[i].fitness > best.fitness) {
      best = population[i];
    }
  }
  return best;
}

// 更新PID参数函数定义
void updatePIDParams(double Kp, double Ki, double Kd) {
  // 更新全局PID参数
  myPID.SetTunings(Kp, Ki, Kd);
}

// 遗传算法任务
void geneticAlgorithmTask(void *pvParameters) {
  initPopulation();

  for (int gen = 0; gen < NUM_GEN; gen++) {
    evaluateFitness();
    selection();
    crossover();
    mutate();
  }

  Individual best = getBestIndividual();
  updatePIDParams(best.Kp, best.Ki, best.Kd);

  // 任务结束后删除自己
  vTaskDelete(NULL);
}