#include <cmath>
#include "Variable.h"

/* Untuk manual */
void MoveRobot() {
  /*------------------------ 1. Atur Kecepatan ------------------------------*/
  // Fitur kecepatan Turbo / Normal
  if(rxStruct.cmd == 'i'){ 
    target_linear_speed = 300;
    target_angular_speed = 150;
  } else {
    target_linear_speed = 150; // Kecepatan linear default
    target_angular_speed = 80; // Kecepatan rotasi default
  }

  // Mengubah data analog DS4 (-1.0 s/d 1.0) menjadi nilai kecepatan RPM
  Vx = -rxStruct.Vx * target_linear_speed;
  Vy = -rxStruct.Vy * target_linear_speed; // Negatif agar maju/mundur sesuai arah DS4
  Wr = rxStruct.W * target_angular_speed;

  // Menghitung Kinematika
  calc.inverse_kinematics(Vx, Vy, Wr);

  /*------------------------ 2. Logika Mirror ------------------------------*/
  // Misal rxStruct.cmd == 'j' dikirim dari DS4 untuk membalikkan arah robot
  if(prevL2 != rxStruct.cmd){
    if(rxStruct.cmd == 'j'){
      mirror = !mirror;
    }
    prevL2 = rxStruct.cmd;
  }

  /*------------------------ 3. Eksekusi Pergerakan Roda -------------------*/
  // if (mirror) { 
  //   // Mode Mirror (Arah dibalik)
  //   rangkabawah.Movement(
  //     constrain(calc.Vwheel[2], -4095, 4095), 
  //     constrain(calc.Vwheel[3], -4095, 4095), 
  //     constrain(calc.Vwheel[0], -4095, 4095), 
  //     constrain(calc.Vwheel[1], -4095, 4095)
  //   );
  // } else { 
  //   // Mode Normal
  //   rangkabawah.Movement(
  //     constrain(calc.Vwheel[0], -4095, 4095), 
  //     constrain(calc.Vwheel[1], -4095, 4095), 
  //     constrain(calc.Vwheel[2], -4095, 4095), 
  //     constrain(calc.Vwheel[3], -4095, 4095)
  //   );
  // }

  if (mirror) { 
    Setpoint1 = calc.Vwheel[2];
    Setpoint2 = calc.Vwheel[3];
    Setpoint3 = calc.Vwheel[0];
    Setpoint4 = calc.Vwheel[1];
  } else { 
    Setpoint1 = calc.Vwheel[0];
    Setpoint2 = calc.Vwheel[1];
    Setpoint3 = calc.Vwheel[2];
    Setpoint4 = calc.Vwheel[3];
  }

  /*------------------------ 4. Kontrol Roller ------------------------------*/
  if (roller) {
    rangkabawahtengah.RollerMovement(Vy); // Roller berputar sesuai Vy
  } else {
    rangkabawahtengah.RollerMovement(0);
  }
}

/* Untuk Otonom */
// void MoveRobot() {
//   // float Vx = rxStruct.Vx; // Konversi cm/s ke m/s
//   // float Vy = rxStruct.Vy; // Konversi cm/s ke m/s
//   // float W = rxStruct.W; // rad/s
//   calc.inverse_kinematics(Vx, Vy, W);
//   float wheel_radius = r / 100.0; // Konversi cm ke m
//   Setpoint1 = calc.Vwheel[0] / (2 * PI * wheel_radius) * 60;
//   Setpoint2 = calc.Vwheel[1] / (2 * PI * wheel_radius) * 60;
//   Setpoint3 = calc.Vwheel[2] / (2 * PI * wheel_radius) * 60;
//   Setpoint4 = calc.Vwheel[3] / (2 * PI * wheel_radius) * 60;
//   Serial.println(calc.Vwheel[0]);
//   calc.inverse_kinematics(Vx, Vy, W);
//   rangkabawah.Movement(constrain(calc.Vwheel[0], -4095, 4095), constrain(calc.Vwheel[1], -4095, 4095), constrain(calc.Vwheel[2], -4095, 4095), constrain(calc.Vwheel[3], -4095, 4095));
//   Serial.println(txStruct.Vx);
//   Serial.println(Setpoint1);
//   Serial.print("Vwheel[0]: "); Serial.print(calc.Vwheel[0]); Serial.print(" m/s, Setpoint1: "); Serial.print(Setpoint1); Serial.println(" RPM");
//   Serial.print("Vwheel[1]: "); Serial.print(calc.Vwheel[1]); Serial.print(" m/s, Setpoint2: "); Serial.print(Setpoint2); Serial.println(" RPM");
//   Serial.print("Vwheel[2]: "); Serial.print(calc.Vwheel[2]); Serial.print(" m/s, Setpoint3: "); Serial.print(Setpoint3); Serial.println(" RPM");
//   Serial.print("Vwheel[3]: "); Serial.print(calc.Vwheel[3]); Serial.print(" m/s, Setpoint4: "); Serial.print(Setpoint4); Serial.println(" RPM");
//   calc.forward_kinematics(Vfilt1, Vfilt2, Vfilt3, Vfilt4, false);
//   calc.forward_kinematics(ENCFR.read(), ENCFL.read(), ENCBL.read(), ENCBR.read(), true);
// }