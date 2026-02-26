#include "Motor.h"

/* ------------  fungsi statis  ---------------- */
void Motor::beginPWM(uint32_t freq, uint8_t res)
{
  analogWriteResolution(res);     // berlaku global
  analogWriteFrequency(freq);     // berlaku global
}

/* ------------  konstruktor objek  ------------ */
Motor::Motor(uint8_t pin_selector, uint8_t pin_pwm)
: _sel(pin_selector), _pwm(pin_pwm)
{
  pinMode(_sel, OUTPUT);
  pinMode(_pwm, OUTPUT);
  analogWrite(_pwm, 0); // Ensure motor is stopped
}

/* ------------  aksi motor  ------------------- */
void Motor::cw(uint16_t duty)
{
  digitalWrite(_sel, HIGH);
  analogWrite(_pwm, duty);
}

void Motor::ccw(uint16_t duty)
{
  digitalWrite(_sel, LOW);
  analogWrite(_pwm, duty);
}

void Motor::stop()
{
  analogWrite(_pwm, 0);
}

/* ------------  motor tengah (roller)  ------- */
MotorMid::MotorMid(uint8_t pin_r_pwm, uint8_t pin_l_pwm)
: r_pwm(pin_r_pwm), l_pwm(pin_l_pwm)
{
  pinMode(r_pwm, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  analogWrite(r_pwm, 0);
  analogWrite(l_pwm, 0);
}

void MotorMid::cw_mid(uint16_t duty)
{
  analogWrite(r_pwm, duty);
  analogWrite(l_pwm, 0);
}

void MotorMid::ccw_mid(uint16_t duty)
{
  analogWrite(r_pwm, 0);
  analogWrite(l_pwm, duty);
}

void MotorMid::stop_mid()
{
  analogWrite(r_pwm, 0);
  analogWrite(l_pwm, 0);
}
