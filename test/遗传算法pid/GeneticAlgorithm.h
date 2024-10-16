#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

// 声明PID实例
extern PID myPID;

struct Individual {
  double Kp;
  double Ki;
  double Kd;
  double fitness;
};

void initPopulation();
void evaluateFitness();
double calculateFitness(double Kp, double Ki, double Kd);
void selection();
void crossover();
void mutate();
Individual getBestIndividual();
void updatePIDParams(double Kp, double Ki, double Kd);
void geneticAlgorithmTask(void *pvParameters);

#endif // GENETIC_ALGORITHM_H