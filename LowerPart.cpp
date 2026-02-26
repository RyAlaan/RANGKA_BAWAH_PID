#include "LowerPart.h"
#include <cmath>   // fabsf

/* ctor default – tidak dipakai saat runtime, tapi dibutuhkan */
LowerPart::LowerPart()
: roda_1(0,0), roda_2(0,0), roda_3(0,0), roda_4(0,0)
{}

/* ctor lengkap – inisialisasi member lewat initializer-list */
LowerPart::LowerPart(byte sel_1, byte pwm_1,
                     byte sel_2, byte pwm_2,
                     byte sel_3, byte pwm_3,
                     byte sel_4, byte pwm_4)
: roda_1(sel_1, pwm_1),
  roda_2(sel_2, pwm_2),
  roda_3(sel_3, pwm_3),
  roda_4(sel_4, pwm_4)
{}

void LowerPart::Movement(float V1, float V2, float V3, float V4) {

  if (V1 > 0)       roda_1.ccw(static_cast<uint16_t>(fabsf(V1)));
  else if (V1 < 0)  roda_1.cw (static_cast<uint16_t>(fabsf(V1)));
  else              roda_1.stop();

  if (V2 > 0)       roda_2.ccw(static_cast<uint16_t>(fabsf(V2)));
  else if (V2 < 0)  roda_2.cw (static_cast<uint16_t>(fabsf(V2)));
  else              roda_2.stop();

  if (V3 > 0)       roda_3.ccw(static_cast<uint16_t>(fabsf(V3)));
  else if (V3 < 0)  roda_3.cw (static_cast<uint16_t>(fabsf(V3)));
  else              roda_3.stop();

  if (V4 > 0)       roda_4.ccw(static_cast<uint16_t>(fabsf(V4)));
  else if (V4 < 0)  roda_4.cw (static_cast<uint16_t>(fabsf(V4)));
  else              roda_4.stop();
}

/*------------------Lower Part Mid (Roller)---------------------*/
/* ctor default – tidak dipakai saat runtime, tapi dibutuhkan */
LowerPartMid::LowerPartMid()
: roda_5(0,0)
{}

/* ctor lengkap – inisialisasi member lewat initializer-list */
LowerPartMid::LowerPartMid(byte rpwm_mid, byte lpwm_mid)
: roda_5(rpwm_mid, lpwm_mid)
{}

void LowerPartMid::RollerMovement(float V5) {

  if (V5 > 0)       roda_5.ccw_mid(static_cast<uint16_t>(fabsf(V5)));
  else if (V5 < 0)  roda_5.cw_mid (static_cast<uint16_t>(fabsf(V5)));
  else              roda_5.stop_mid();
}