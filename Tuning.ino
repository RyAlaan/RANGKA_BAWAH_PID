#include "Variable.h"

void inputCommand() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.length()) {
      String tok[10]; // Maksimal 10 kata
      int count = 0, start = 0;
      
      // Memecah teks berdasarkan spasi
      for (int i = 0; i <= line.length() && count < 10; i++) {
        if (i == line.length() || line.charAt(i) == ' ') {
          tok[count++] = line.substring(start, i);
          start = i + 1;
        }
      }

      // 1. Perintah "A": Set PID Semua Motor -> Contoh: A 1.5 0.05 0.1
      if (count == 4 && toupper(tok[0].charAt(0)) == 'A') {
        float p = tok[1].toFloat();
        float i = tok[2].toFloat();
        float d = tok[3].toFloat();
        
        Kp1 = Kp2 = Kp3 = Kp4 = p;
        Ki1 = Ki2 = Ki3 = Ki4 = i;
        Kd1 = Kd2 = Kd3 = Kd4 = d;
        
        // Terapkan langsung ke sistem PID
        motor1.SetTunings(Kp1, Ki1, Kd1);
        motor2.SetTunings(Kp2, Ki2, Kd2);
        motor3.SetTunings(Kp3, Ki3, Kd3);
        motor4.SetTunings(Kp4, Ki4, Kd4);
      }
      
      // 2. Perintah "R": Set Target RPM Roda -> Contoh: R 150 150 150 150
      else if (count == 5 && toupper(tok[0].charAt(0)) == 'R') {
        Setpoint1 = tok[1].toFloat();
        Setpoint2 = tok[2].toFloat();
        Setpoint3 = tok[3].toFloat();
        Setpoint4 = tok[4].toFloat(); 
      }

      else if (count == 4 && toupper(tok[0].charAt(0)) == 'K') {
        rxStruct.Vx = tok[1].toFloat();
        rxStruct.Vy = tok[2].toFloat();
        rxStruct.Wr = tok[3].toFloat();
      }
      
      // 3. Perintah "W" (Jalan) dan "Q" (Berhenti)
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
        }else {
          Serial.println("Perintah Salah! Gunakan: 'A <Kp> <Ki> <Kd>', 'R <RPM1> <RPM2> <RPM3> <RPM4>', 'W', atau 'Q'");
        }
      }
    }
  }
}



/* ------------------------------- PUNYA MAS ADMIN --------------------------------- */

// #include "Variable.h"

// float value1 = 0, value2 = 0, value3 = 0;

// Motor roda1(sel_fr, pwm_fr);
// Motor roda2(sel_fl, pwm  _fl);
// Motor roda3(sel_bl, pwm_bl);
// Motor roda4(sel_br, pwm_br);

// float norm_(float yaw) {
//   return fmod((yaw + 180), 360) - 180;
// }

// void setup() {
//   Serial.begin(115200);
//   digitalWrite(LED, LOW);
//   Motor::beginPWM(20000, 12); // Frekuensi 20kHz, resolusi 12-bit
//   pinMode(sel_fr, OUTPUT);
//   pinMode(pwm_fr, OUTPUT);
//   pinMode(sel_fl, OUTPUT);
//   pinMode(pwm_fl, OUTPUT);
//   pinMode(sel_bl, OUTPUT);
//   pinMode(pwm_bl, OUTPUT);
//   pinMode(sel_br, OUTPUT);
//   pinMode(pwm_br, OUTPUT);
//   PID_init();
//   // Inisialisasi objek
//   rangkabawah = LowerPart(sel_fr, pwm_fr, sel_fl, pwm_fl, sel_bl, pwm_bl, sel_br, pwm_br);
//   calc = Kinematics(a1, a2, a3, a4, r, R);
//   calc.set_ideal_value(a1_ideal, a2_ideal, a3_ideal, a4_ideal);
//   calc.PPR = PPR;
//   calc.init();
//   calc.update_angle(0);
//   // myTransfer.begin(Serial); // Komentari jika tidak digunakan
// }

// void loop() {
//   inputCommand();
//   if (!stop) {
//     if (millis() - input_prevmillis >= inputrate) {
//       prev_fr_tics = fr_tics; prev_fl_tics = fl_tics; prev_bl_tics = bl_tics; prev_br_tics = br_tics;
//       fr_tics = ENCFR.read(); fl_tics = ENCFL.read(); bl_tics = ENCBL.read(); br_tics = ENCBR.read();
//       prev_Timing = Timing;
//       Timing = micros();
//       Vreal1 = ((fr_tics - prev_fr_tics) / (((float)(Timing - prev_Timing)) / 1.0e6)) / PPR * 60.0;
//       Vreal2 = ((fl_tics - prev_fl_tics) / (((float)(Timing - prev_Timing)) / 1.0e6)) / PPR * 60.0;
//       Vreal3 = ((bl_tics - prev_bl_tics) / (((float)(Timing - prev_Timing)) / 1.0e6)) / PPR * 60.0;
//       Vreal4 = ((br_tics - prev_br_tics) / (((float)(Timing - prev_Timing)) / 1.0e6)) / PPR * 60.0;
//       Vfilt1 = Roda_1.updateEstimate(Vreal1);
//       Vfilt2 = Roda_2.updateEstimate(Vreal2);
//       Vfilt3 = Roda_3.updateEstimate(Vreal3);
//       Vfilt4 = Roda_4.updateEstimate(Vreal4);
//       MoveRobot();
//       PID_compute();
//       calc.forward_kinematics(Vfilt1, Vfilt2, Vfilt3, Vfilt4, false);
//       calc.forward_kinematics(ENCFR.read(), ENCFL.read(), ENCBL.read(), ENCBR.read(), true);
//       txStruct.X = (float)calc.dist_travel[0];
//       txStruct.Y = (float)calc.dist_travel[1];
//       txStruct.tetha = norm_((float)calc.dist_travel[2]);
//       txStruct.Vx = (float)calc.Vreal[0];
//       txStruct.Vy = (float)calc.Vreal[1];
//       txStruct.Wr = (float)calc.Vreal[2];
//       input_prevmillis = millis();
//     }
//   } else {
//     if (reset_data) {
//       ENCFR.readAndReset();
//       ENCFL.readAndReset();
//       ENCBL.readAndReset();
//       ENCBR.readAndReset();
//       calc.dist_travel[0] = 0;
//       calc.dist_travel[1] = 0;
//       calc.dist_travel[2] = 0;
//       reset_data = false;
//     }
//     rxStruct.Vx = 0;
//     rxStruct.Vy = 0;
//     rxStruct.W = 0;
//     rangkabawah.Movement(0, 0, 0, 0);
//   }
// }








// void inputCommand() {
//   if (Serial.available()) {
//     String line = Serial.readStringUntil('\n');
//     line.trim();
//     if (line.length()) {
//       String tok[MAXTOK];
//       int count = 0, start = 0;
//       for (int i = 0; i <= line.length() && count < MAXTOK; i++) {
//         if (i == line.length() || line.charAt(i) == ' ') {
//           tok[count++] = line.substring(start, i);
//           start = i + 1;
//         }
//       }
//       // Perintah untuk set PID semua motor
//       if (count == 4 && toupper(tok[0].charAt(0)) == 'A') {
//         float p = tok[1].toFloat();
//         float i = tok[2].toFloat();
//         float d = tok[3].toFloat();
//         if (p < 0 || p > 100 || i < 0 || i > 20 || d < 0 || d > 10) {
//           Serial.println("Nilai PID di luar batas (Kp: 0-100, Ki: 0-10, Kd: 0-10)");
//           return;
//         }
//         Kp1 = Kp2 = Kp3 = Kp4 = p;
//         Ki1 = Ki2 = Ki3 = Ki4 = i;
//         Kd1 = Kd2 = Kd3 = Kd4 = d;
//         motor1.SetTunings(Kp1, Ki1, Kd1);
//         motor2.SetTunings(Kp2, Ki2, Kd2);
//         motor3.SetTunings(Kp3, Ki3, Kd3);
//         motor4.SetTunings(Kp4, Ki4, Kd4);
//         Serial.print("All Motors Tunings → Kp="); Serial.print(p);
//         Serial.print(" Ki="); Serial.print(i);
//         Serial.print(" Kd="); Serial.println(d);
//       }
//       // Perintah untuk set PID per motor
//       else if (count == 4 && isDigit(tok[0].charAt(0))) {
//         int motorNum = tok[0].toInt();
//         float p = tok[1].toFloat();
//         float i = tok[2].toFloat();
//         float d = tok[3].toFloat();
//         if (p < 0 || p > 100 || i < 0 || i >= 20 || d < 0 || d > 10) {
//           Serial.println("Nilai PID di luar batas (Kp: 0-100, Ki: 0-10, Kd: 0-10)");
//           return;
//         }
//         if (motorNum == 1) {
//           Kp1 = p; Ki1 = i; Kd1 = d;
//           motor1.SetTunings(Kp1, Ki1, Kd1);
//         } else if (motorNum == 2) {
//           Kp2 = p; Ki2 = i; Kd2 = d;
//           motor2.SetTunings(Kp2, Ki2, Kd2);
//         } else if (motorNum == 3) {
//           Kp3 = p; Ki3 = i; Kd3 = d;
//           motor3.SetTunings(Kp3, Ki3, Kd3);
//         } else if (motorNum == 4) {
//           Kp4 = p; Ki4 = i; Kd4 = d;
//           motor4.SetTunings(Kp4, Ki4, Kd4);
//         } else {
//           Serial.println("Nomor motor salah (1-4)");
//           return;
//         }
//         Serial.print("Motor "); Serial.print(motorNum);
//         Serial.print(" Tunings → Kp="); Serial.print(p);
//         Serial.print(" Ki="); Serial.print(i);
//         Serial.print(" Kd="); Serial.println(d);
//       }
//       // Perintah untuk set setpoint robot (Vx, Vy)
//       else if (count == 3 && toupper(tok[0].charAt(0)) == 'S') {
//         rxStruct.Vx = tok[1].toFloat();
//         rxStruct.Vy = tok[2].toFloat();
//         rxStruct.W = 0.0; // Default rotasi nol
//         float Vx = rxStruct.Vx; // Konversi cm/s ke m/s
//         // float Vx = 2000; // Konversi cm/s ke m/s
//         float Vy = rxStruct.Vy; // Konversi cm/s ke m/s
//         float W = rxStruct.W; // rad/s
//         calc.inverse_kinematics(Vx, Vy, W);
//         Serial.print("Setpoint Vx: "); Serial.print(rxStruct.Vx); Serial.print(" cm/s");
//         Serial.print(" Vy: "); Serial.print(rxStruct.Vy); Serial.print(" cm/s");
//         Serial.print(" W: "); Serial.println(rxStruct.W); Serial.println(" rad/s");
//       }
//       // Perintah untuk set setpoint langsung per motor (RPM)
//       else if (count == 5 && toupper(tok[0].charAt(0)) == 'R') {
//         Setpoint1 = tok[1].toFloat();
//         Setpoint2 = tok[2].toFloat();
//         Setpoint3 = tok[3].toFloat();
//         Setpoint4 = tok[4].toFloat();
//         // Serial.println(Setpoint1);
//         // Serial.print("Setpoint1: "); Serial.print(Setpoint1); Serial.println(" RPM");
//         // Serial.print("Setpoint2: "); Serial.print(Setpoint2); Serial.println(" RPM");
//         // Serial.print("Setpoint3: "); Serial.print(Setpoint3); Serial.println(" RPM");
//         // Serial.print("Setpoint4: "); Serial.print(Setpoint4); Serial.println(" RPM");
//       }
//       // Perintah Q/W
//       else {
//         char c = toupper(line.charAt(0));
//         if (c == 'Q') {
//           stop = true;
//           Serial.println("Robot dihentikan");
//         } else if (c == 'W') {
//           stop = false;
//           Serial.println("Robot dijalankan");
//         } else {
//           Serial.println(F("Input salah. Ketik: A <Kp> <Ki> <Kd> atau <motor> <Kp> <Ki> <Kd> atau S <Vx> <Vy> atau R <Set1> <Set2> <Set3> <Set4> atau Q/W"));
//         }
//       }
//     }
//   }
// }