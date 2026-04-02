#include <cmath>
#include "Variable.h"

void MoveRobot() {
  int mode = true; // true = auto, false = manual

  if (rxStruct.cmd == 'a') {
    mode = !mode;
  }

  if (mode) {
    AutomationMode();
  } else {
    ManualMode();
  }
}

/* Untuk manual */
void ManualMode() {
  /*------------------------ 1. Atur Kecepatan ------------------------------*/
  // Fitur kecepatan Turbo / Normal
  if(rxStruct.cmd == 'i'){ 
    target_linear_speed = 5000;
    target_angular_speed = 3000;
  } else {
    target_linear_speed = 900; // Kecepatan linear default
    target_angular_speed = 600; // Kecepatan rotasi default
  }

  if (rxStruct.cmd == 'd') {
    digitalWrite(LED, !digitalRead(LED));
  }

  // Mengubah data analog DS4 (-1.0 s/d 1.0) menjadi nilai kecepatan RPM
  Vx = -rxStruct.Vx * target_linear_speed;
  Vy = -rxStruct.Vy * target_linear_speed;
  Wr = rxStruct.W * target_angular_speed;
  
  // Menghitung Kinematika
  calc.inverse_kinematics(Vx, Vy, Wr);

  // translate to global
  calc.converte_global(&Vx, &Vy);
    
  /*------------------------ 2. Logika Mirror ------------------------------*/
  // Misal rxStruct.cmd == 'j' dikirim dari DS4 untuk membalikkan arah robot
  if(prevL2 != rxStruct.cmd){
    if(rxStruct.cmd == 'j'){
      mirror = !mirror;
    }
    prevL2 = rxStruct.cmd;
  }

  /*------------------------ 3. Eksekusi Pergerakan Roda -------------------*/
  if (mirror) { 
    // Mode Mirror (Arah dibalik)
    rangkabawah.Movement(
      constrain(calc.Vwheel[2], -255, 255), 
      constrain(calc.Vwheel[3], -255, 255), 
      constrain(calc.Vwheel[0], -255, 255), 
      constrain(calc.Vwheel[1], -255, 255)
    );
  } else { 
    // Mode Normal
    rangkabawah.Movement(
      constrain(calc.Vwheel[0], -255, 255), 
      constrain(calc.Vwheel[1], -255, 255), 
      constrain(calc.Vwheel[2], -255, 255), 
      constrain(calc.Vwheel[3], -255, 255)
    );
  }

  if (roller) {
    rangkabawahtengah.RollerMovement(constrain(Vy, -255, 255));
  } else {
    rangkabawahtengah.RollerMovement(0);
  }
}

void AutomationMode(){
  Vx = round(rxStruct.Vx);
  Vy = round(rxStruct.Vy);
  Wr = round(rxStruct.Wr);

  // calc.update_angle(0);
  calc.inverse_kinematics(Vx, Vy, Wr);

  Setpoint1 = calc.Vwheel[0];
  Setpoint2 = calc.Vwheel[1];
  Setpoint3 = calc.Vwheel[2];
  Setpoint4 = calc.Vwheel[3];


  pwm1 += Output1; pwm2 += Output2; pwm3 += Output3; pwm4 += Output4;
  rangkabawah.Movement(pwm1, pwm2, pwm3, pwm4);
  // rangkabawah.Movement(Output1, Output2, Output3, Output4);

  // calc.forward_kinematics(Vfilt1, Vfilt2, Vfilt3, Vfilt4, false);
  // calc.forward_kinematics(ENCFR.read(), ENCFL.read(), ENCBL.read(), ENCBR.read(), true);
}