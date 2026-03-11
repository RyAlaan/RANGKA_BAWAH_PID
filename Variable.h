#ifndef VARIABLE_H
#define VARIABLE_H
#include "Header.h"

/* ---- KINEMATICS CONSTANTS ---- */
const float a1_ideal = 45; //Sudut tiap roda relatif terhadap sumbu x positif
const float a2_ideal = 135;
const float a3_ideal = 225;
const float a4_ideal = 315;
const float a1 = 46.1691; //Sudut tiap roda relatif terhadap sumbu x positif
const float a2 = 130.2666;
const float a3 = 226.3972;
const float a4 = 316.6683;

const float r = 5; //Jari-jari roda
const float R = 23.5849; //Jari-jari roda terhadap pusat robot

/* ---- ENCODER ---- */
// float Vx, Vy, W = 0;
float pwm1, pwm2, pwm3, pwm4 = 0;
float Vreal1, Vreal2, Vreal3, Vreal4 = 0;
float Vfilt1, Vfilt2, Vfilt3, Vfilt4 = 0;
// float Setpoint1, Setpoint2, Setpoint3, Setpoint4 = 0; // Setpoint dalam RPM

volatile long fr_tics, fl_tics, bl_tics, br_tics;
volatile long prev_fr_tics, prev_fl_tics, prev_bl_tics, prev_br_tics;
volatile long prev_Timing, Timing = 0;
volatile long timePeriode;

/* ---- SETPOINT ---- */
const int max_linear_speed = 188; //vel/(2*r*PI) * 60; vel in cm/s
const int max_angular_speed = 188; //vel/360*(2*R)/2*r*60; vel in deg/s
const int min_motor_pwm = 25; //pwm min for the motor to start rotating

/*-------------------------Speed------------------------------*/
float target_linear_speed = 1800;
float target_angular_speed = 1200;
int Vx,Vy, Wr = 0;

// settling time PID
unsigned long settlingStart = 0;
bool settlingDone = false;
float settlingTime = 0;

/* ---------- PID VERSI 2 ----------*/
float Setpoint1 = 0, Setpoint2 = 0, Setpoint3 = 0, Setpoint4 = 0; // Target RPM
// Nilai Kp, Ki, Kd (Bisa Anda atur nanti saat tuning)
float Output1 = 0, Kp1 = 1, Ki1 = 0, Kd1 = 0;
float Output2 = 0, Kp2 = 1, Ki2 = 0, Kd2 = 0;
float Output3 = 0, Kp3 = 1, Ki3 = 0, Kd3 = 0;
float Output4 = 0, Kp4 = 1, Ki4 = 0, Kd4 = 0;

QuickPID motor1(&Vfilt1, &Output1, &Setpoint1, Kp1, Ki1, Kd1,
               QuickPID::pMode::pOnError, 
               QuickPID::dMode::dOnMeas, 
               QuickPID::iAwMode::iAwCondition, 
               QuickPID::Action::direct);
QuickPID motor2(&Vfilt2, &Output2, &Setpoint2, Kp2, Ki2, Kd2,
               QuickPID::pMode::pOnError, 
               QuickPID::dMode::dOnMeas, 
               QuickPID::iAwMode::iAwCondition, 
               QuickPID::Action::direct);
QuickPID motor3(&Vfilt3, &Output3, &Setpoint3, Kp3, Ki3, Kd3,
               QuickPID::pMode::pOnError, 
               QuickPID::dMode::dOnMeas, 
               QuickPID::iAwMode::iAwCondition, 
               QuickPID::Action::direct);
QuickPID motor4(&Vfilt4, &Output4, &Setpoint4, Kp4, Ki4, Kd4,
               QuickPID::pMode::pOnError, 
               QuickPID::dMode::dOnMeas, 
               QuickPID::iAwMode::iAwCondition, 
               QuickPID::Action::direct);












/* ---- PID ---- */
// 1m/s -> 0.475 11/8/5 0.01/0.0125/0.015
// float Output1 = 0, Kp1 = 0.55, Ki1 = 0.004, Kd1 = 0.01;
// float Output2 = 0, Kp2 = 0.55, Ki2 = 0.004, Kd2 = 0.01;
// float Output3 = 0, Kp3 = 0.55, Ki3 = 0.004, Kd3 = 0.01;
// float Output4 = 0, Kp4 = 0.55, Ki4 = 0.004, Kd4 = 0.01;

// QuickPID motor1(&Vfilt1, &Output1, &calc.Vwheel[0], Kp1, Ki1, Kd1,
//                QuickPID::pMode::pOnError,
//                QuickPID::dMode::dOnMeas,            
//                QuickPID::iAwMode::iAwCondition,  
//                QuickPID::Action::direct);
// QuickPID motor2(&Vfilt2, &Output2, &calc.Vwheel[1], Kp2, Ki2, Kd2,
//                QuickPID::pMode::pOnError,
//                QuickPID::dMode::dOnMeas,            
//                QuickPID::iAwMode::iAwCondition,  
//                QuickPID::Action::direct);
// QuickPID motor3(&Vfilt3, &Output3, &calc.Vwheel[2], Kp3, Ki3, Kd3,
//                QuickPID::pMode::pOnError,
//                QuickPID::dMode::dOnMeas,            
//                QuickPID::iAwMode::iAwCondition,  
//                QuickPID::Action::direct);
// QuickPID motor4(&Vfilt4, &Output4, &calc.Vwheel[3], Kp4, Ki4, Kd4,
//                QuickPID::pMode::pOnError,
//                QuickPID::dMode::dOnMeas,            
//                QuickPID::iAwMode::iAwCondition,  
//                QuickPID::Action::direct);

/* ---- KALMAN FILTER ---- */
/*e_mea: Measurement Uncertainty 
  e_est: Estimation Uncertainty 
  q: Process Noise*/
float e_mea = 22, q = 0.25, e_est = 22;

SimpleKalmanFilter Roda_1(e_mea, e_est, q);
SimpleKalmanFilter Roda_2(e_mea, e_est, q);
SimpleKalmanFilter Roda_3(e_mea, e_est, q);
SimpleKalmanFilter Roda_4(e_mea, e_est, q);

/* ---- SERIAL COM ---- */
struct __attribute__((packed)) STRUCTRX {
  char cmd;
  float Vx;
  float Vy;  
  float Wr;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float X; float Y; float tetha;
  float Vx; float Vy; float Wr;
} txStruct;

/* --- ROLLER CONTROL --- */
bool roller = false; // default false

/* --- CMD --- */
char command = ' ';
char prevCommand = ' ';
bool stop = true;
bool mirror = false;
bool reset_data = false;
bool PID_on = false;

/*-------------------------Debounce------------------------------*/
char prevStart, prevL2;

#endif