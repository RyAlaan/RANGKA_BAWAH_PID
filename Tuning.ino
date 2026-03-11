#include "Variable.h"

void inputCommand() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length()) {
      String tok[10];
      int count = 0, start = 0;

      for (int i = 0; i <= (int)line.length() && count < 10; i++) {
        if (i == (int)line.length() || line.charAt(i) == ' ') {
          tok[count++] = line.substring(start, i);
          start = i + 1;
        }
      }

      // Perintah "A": Set PID semua motor -> Contoh: A 1.5 0.05 0.1
      if (count == 4 && toupper(tok[0].charAt(0)) == 'A') {
        float p = tok[1].toFloat();
        float i = tok[2].toFloat();
        float d = tok[3].toFloat();

        Kp1 = Kp2 = Kp3 = Kp4 = p;
        Ki1 = Ki2 = Ki3 = Ki4 = i;
        Kd1 = Kd2 = Kd3 = Kd4 = d;

        motor1.SetTunings(Kp1, Ki1, Kd1);
        motor2.SetTunings(Kp2, Ki2, Kd2);
        motor3.SetTunings(Kp3, Ki3, Kd3);
        motor4.SetTunings(Kp4, Ki4, Kd4);

        Serial.print(">> PID diset: Kp="); Serial.print(p);
        Serial.print(" Ki="); Serial.print(i);
        Serial.print(" Kd="); Serial.println(d);
      }

      // Perintah "R": Set Setpoint RPM tiap roda -> Contoh: R 150 150 150 150
      else if (count == 5 && toupper(tok[0].charAt(0)) == 'R') {
        Setpoint1 = tok[1].toFloat();
        Setpoint2 = tok[2].toFloat();
        Setpoint3 = tok[3].toFloat();
        Setpoint4 = tok[4].toFloat();

        Serial.print(">> Setpoint RPM: ");
        Serial.print(Setpoint1); Serial.print(" ");
        Serial.print(Setpoint2); Serial.print(" ");
        Serial.print(Setpoint3); Serial.print(" ");
        Serial.println(Setpoint4);
      }

      // Perintah "K": Set kecepatan robot -> Contoh: K 0 150 0
      else if (count == 4 && toupper(tok[0].charAt(0)) == 'K') {
        rxStruct.Vx = tok[1].toFloat();
        rxStruct.Vy = tok[2].toFloat();
        rxStruct.Wr = tok[3].toFloat();

        Serial.print(">> Kecepatan diset: Vx="); Serial.print(rxStruct.Vx);
        Serial.print(" Vy="); Serial.print(rxStruct.Vy);
        Serial.print(" Wr="); Serial.println(rxStruct.Wr);
      }

      // Perintah "W" (Start) dan "Q" (Stop)
      else {
        char c = toupper(line.charAt(0));
        if (c == 'Q') {
          stop = true;
          PID_on = false;
          rxStruct.Vx = 0;
          rxStruct.Vy = 0;
          rxStruct.Wr = 0;
          Setpoint1 = 0;
          Setpoint2 = 0;
          Setpoint3 = 0;
          Setpoint4 = 0;
          Serial.println(">> ROBOT BERHENTI DARURAT (STOP) <<");
        } else if (c == 'W') {
          stop = false;
          PID_on = true;
          Serial.println(">> ROBOT DINYALAKAN (START) <<");
        } else {
          Serial.println("Perintah Salah! Gunakan:");
          Serial.println("  A <Kp> <Ki> <Kd>          -> Set PID");
          Serial.println("  R <RPM1> <RPM2> <RPM3> <RPM4> -> Set RPM langsung");
          Serial.println("  K <Vx> <Vy> <Wr>          -> Set kecepatan robot");
          Serial.println("  W -> Start  |  Q -> Stop");
        }
      }
    }
  }
}
