#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <HardwareTimer.h>

class Motor {
  public:
    /*   panggil sekali di setup()
         freq  = frekuensi PWM (Hz)
         res   = bit-resolusi PWM (1-16)  */
    static void beginPWM(uint32_t freq = 20000, uint8_t res = 12);
    /*   konstruktor objek motor
         pin_selector = pin untuk memilih arah putar
         pin_pwm      = pin untuk sinyal PWM  */
    Motor(uint8_t pin_selector, uint8_t pin_pwm);

    void cw (uint16_t duty);
    void ccw(uint16_t duty);
    void stop();

  private:
    uint8_t _sel, _pwm;
    uint8_t r_pwm, l_pwm; // Pins for middle roller motor
};

class MotorMid {
  public:
    /* motor tengah (roller) pakai 2 pin PWM: kanan & kiri */
    MotorMid(uint8_t pin_r_pwm, uint8_t pin_l_pwm);

    void cw_mid(uint16_t duty);
    void ccw_mid(uint16_t duty);
    void stop_mid();

  private:
    uint8_t r_pwm, l_pwm;
};

#endif