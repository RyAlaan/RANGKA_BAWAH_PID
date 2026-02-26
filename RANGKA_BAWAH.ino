#include "Variable.h"

Motor roda1(sel_fr, pwm_fr); // roda 4
Motor roda2(sel_fl, pwm_fl); // roda 2
Motor roda3(sel_bl, pwm_bl); // roda 1
Motor roda4(sel_br, pwm_br); // roda 3
MotorMid roda5(rpwm_mid, lpwm_mid);

float norm_(float yaw){
  return fmod((yaw + 180), 360) - 180;
}

void setup(){
  Serial.begin(115200);
  Motor::beginPWM(20000, 12);  
  
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

void loop(){
  /* ===================== TERIMA DATA DS4 ===================== */
  // receive();

  /* ===================== TERIMA PERINTAH TUNING ============== */
  inputCommand();

  /*=============================MAIN PROGRAM=============================*/
  if(!stop){
    if (millis() - input_prevmillis >= inputrate){
      
      // Memanggil pergerakan dari file MoveRobot.ino
      // MoveRobot();

      /* --- BACA ENCODER (Hanya untuk Odometri, Tidak Pakai PID) --- */
      prev_fr_tics = fr_tics; prev_fl_tics = fl_tics, prev_bl_tics = bl_tics; prev_br_tics = br_tics;
      fr_tics = ENCFR.read(); fl_tics = ENCFL.read(); bl_tics = ENCBL.read(); br_tics = ENCBR.read(); 
      prev_Timing = Timing;
      Timing = micros();
      
      Vreal1 = ((fr_tics - prev_fr_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
      Vreal2 = ((fl_tics - prev_fl_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
      Vreal3 = ((bl_tics - prev_bl_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
      Vreal4 = ((br_tics - prev_br_tics)/(((float)(Timing - prev_Timing))/1.0e6))/PPR*60.0;
      
      Vfilt1 = Roda_1.updateEstimate(Vreal1);
      Vfilt2 = Roda_2.updateEstimate(Vreal2);
      Vfilt3 = Roda_3.updateEstimate(Vreal3);
      Vfilt4 = Roda_4.updateEstimate(Vreal4);

      PID_compute();

      txStruct.X = (float) calc.dist_travel[0];
      txStruct.Y = (float) calc.dist_travel[1];
      txStruct.tetha = norm_((float) calc.dist_travel[2]);
      txStruct.Vx = (float) calc.Vreal[0];
      txStruct.Vy = (float) calc.Vreal[1];
      txStruct.Wr = (float) calc.Vreal[2];

      input_prevmillis = millis();
    }
  } else {
    // Jika robot di-stop (ditekan O di DS4)
    if(reset_data){
      ENCFR.readAndReset(); ENCFL.readAndReset(); ENCBL.readAndReset(); ENCBR.readAndReset();
      calc.dist_travel[0] = 0; calc.dist_travel[1] = 0; calc.dist_travel[2] = 0;
      reset_data = false;
    }
    Vx = 0; Vy = 0; Wr = 0;
    rangkabawah.Movement(0, 0, 0, 0);
  }

  /*============================DEBUG SERIAL============================*/
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 100) {
  //   Serial.print("DS4 Mode | Vy: "); Serial.print(Vy);
  //   Serial.print("  Vx: "); Serial.print(Vx);
  //   Serial.print("  Wr: "); Serial.println(Wr);
  //   lastPrint = millis(); // <--- TITIK KOMA SUDAH DITAMBAHKAN
  // }
}

void receive(){
  if(myTransfer.available()){
    int16_t recSize = 0;
    recSize = myTransfer.rxObj(rxStruct, recSize);

    // Mengekstrak karakter CMD dari Python
    if (rxStruct.cmd == 'a') {
      stop = false; // Robot nyala jika pencet X
    } else if (rxStruct.cmd == 'b') {
      stop = true;  // Robot mati jika pencet O
    }
  }  
} // <--- KURUNG KURAWAL BERLEBIH SUDAH DIHAPUS

void transfer(){
  if(millis() - timePeriode >= 15){
    int16_t sendSize = 0;
    struct __attribute__((packed)) STRUCT {
      float pwm1 = calc.Vwheel[0];
      float pwm2 = calc.Vwheel[1];
      float pwm3 = calc.Vwheel[2];
      float pwm4 = calc.Vwheel[3];
    }testStruct;

    sendSize = myTransfer.txObj(testStruct, sendSize);
    myTransfer.sendData(sendSize);
    timePeriode = millis();
  }
}