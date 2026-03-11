#include "Variable.h"

Motor roda1(sel_fr, pwm_fr);  // roda 4
// Motor roda2(sel_fl, pwm_fl);  // roda 2
// Motor roda3(sel_bl, pwm_bl);  // roda 1
// Motor roda4(sel_br, pwm_br);  // roda 3
// MotorMid roda5(rpwm_mid, lpwm_mid);

float norm_(float yaw) {
  return fmod((yaw + 180), 360) - 180;
}

void setup() {
  Serial.begin(115200);
  Motor::beginPWM(20000, 12);
  // MotorMid::beginPWM(2000, 12);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(rpwm_mid, OUTPUT);
  pinMode(lpwm_mid, OUTPUT);

  // OBJECT CONSTRUCTING
  rangkabawah = LowerPart(sel_fr, pwm_fr, sel_fl, pwm_fl, sel_bl, pwm_bl, sel_br, pwm_br);
  rangkabawahtengah = LowerPartMid(rpwm_mid, lpwm_mid);

  calc = Kinematics(a1, a2, a3, a4, r, R);
  calc.set_ideal_value(a1_ideal, a2_ideal, a3_ideal, a4_ideal);
  calc.PPR = PPR;
  calc.init();
  calc.update_angle(0);

  PID_init();

  // myTransfer.begin(Serial); // Terhubung ke Mini PC / Python
}

void loop() {
  /* ===================== TERIMA DATA DS4 ===================== */
  // receive();

  /* ===================== TERIMA PERINTAH TUNING ============== */
  inputCommand();

  /*=============================MAIN PROGRAM=============================*/
  if (!stop) {
    if (millis() - input_prevmillis >= inputrate) {

      // Memanggil pergerakan dari file MoveRobot.ino isinya inverse kinematic 
      MoveRobot();
      
      PID_compute();

      prev_fr_tics = fr_tics; prev_fl_tics = fl_tics;
      prev_bl_tics = bl_tics; prev_br_tics = br_tics;
      fr_tics = ENCFR.read(); fl_tics = ENCFL.read();
      bl_tics = ENCBL.read(); br_tics = ENCBR.read();
      prev_Timing = Timing; 
      Timing = micros();

      float dt = ((float)(Timing - prev_Timing)) / 1.0e6;
      Vreal1 = ((fr_tics - prev_fr_tics) / dt) / PPR * 60.0;
      Vreal2 = ((fl_tics - prev_fl_tics) / dt) / PPR * 60.0;
      Vreal3 = ((bl_tics - prev_bl_tics) / dt) / PPR * 60.0;
      Vreal4 = ((br_tics - prev_br_tics) / dt) / PPR * 60.0;

      Vfilt1 = Roda_1.updateEstimate(Vreal1);
      Vfilt2 = Roda_2.updateEstimate(Vreal2);
      Vfilt3 = Roda_3.updateEstimate(Vreal3);
      Vfilt4 = Roda_4.updateEstimate(Vreal4);

      // Odometri: forward kinematics dari RPM terfilter dan raw encoder
      calc.forward_kinematics(Vfilt1, Vfilt2, Vfilt3, Vfilt4, false);
      calc.forward_kinematics(ENCFR.read(), ENCFL.read(), ENCBL.read(), ENCBR.read(), true);

      txStruct.X = (float)calc.dist_travel[0];
      txStruct.Y = (float)calc.dist_travel[1];
      txStruct.tetha = norm_((float)calc.dist_travel[2]);
      txStruct.Vx = (float)calc.Vreal[0];
      txStruct.Vy = (float)calc.Vreal[1];
      txStruct.Wr = (float)calc.Vreal[2];
      input_prevmillis = millis();
    }
  } else {
    if (reset_data) {
      ENCFR.readAndReset();
      ENCFL.readAndReset();
      ENCBL.readAndReset();
      ENCBR.readAndReset();
      calc.dist_travel[0] = 0;
      calc.dist_travel[1] = 0;
      calc.dist_travel[2] = 0;
      reset_data = false;
    }
    Vx = 0;
    Vy = 0;
    Wr = 0;

    // fungsi PID_on == false pindah ke sini
    pwm1 = 0; pwm2 = 0; pwm3 = 0; pwm4 = 0;
    PID_reset();
    rangkabawah.Movement(constrain(calc.Vwheel[0], -4095, 4095), constrain(calc.Vwheel[1], -4095, 4095), constrain(calc.Vwheel[2], -4095, 4095), constrain(calc.Vwheel[3], -4095, 4095));

    rangkabawah.Movement(0, 0, 0, 0);
  }

  Serial.print("Base:"); Serial.print(0);

  Serial.print(",Setpoint1:");
  Serial.print(Setpoint1);
  Serial.print(",RPM1:");
  Serial.print(Vfilt1);
  Serial.print(",PWM1:");
  Serial.print(pwm1);
  Serial.println();

  Serial.print("Setpoint2:");
  Serial.print(Setpoint2);
  Serial.print(",RPM2:");
  Serial.print(Vfilt2);
  Serial.print(",PWM2:");
  Serial.print(pwm2);
  Serial.println();
}

  Serial.print("Setpoint3:");
  Serial.print(Setpoint3);
  Serial.print(",RPM3:");
  Serial.print(Vfilt3);
  Serial.print(",PWM3:");
  Serial.print(pwm3);
  Serial.println();
}

  Serial.print("Setpoint4:");
  Serial.print(Setpoint4);
  Serial.print(",RPM4:");
  Serial.print(Vfilt4);
  Serial.print(",PWM4:");
  Serial.print(pwm4);
  Serial.println();
}

/*============================DEBUG SERIAL============================*/
// static unsigned long lastPrint = 0;
// if (millis() - lastPrint > 100) {
//   Serial.print("DS4 Mode | Vy: "); Serial.print(Vy);
//   Serial.print("  Vx: "); Serial.print(Vx);
//   Serial.print("  Wr: "); Serial.println(Wr);
//   lastPrint = millis(); /
// }
// }

void receive() {
  if (myTransfer.available()) {
    int16_t recSize = 0;
    recSize = myTransfer.rxObj(rxStruct, recSize);
    if (rxStruct.cmd == 'a') stop = false;
    else if (rxStruct.cmd == 'b') stop = true;
  }
}

void transfer() {
  if (millis() - timePeriode >= 15) {
    int16_t sendSize = 0;
    struct __attribute__((packed)) STRUCT {
      float pwm1 = calc.Vwheel[0];
      float pwm2 = calc.Vwheel[1];
      float pwm3 = calc.Vwheel[2];
      float pwm4 = calc.Vwheel[3];
    } testStruct;
    sendSize = myTransfer.txObj(testStruct, sendSize);
    myTransfer.sendData(sendSize);
    timePeriode = millis();
  }
}