/*
  A4988.cpp - - Arduino library for using the A4988 stepper driver
  William Smith, 2014

  The A4988 stepper driver is for Pololu stepper driver boards
  and compatible clones. These boards use the Allegro A4988
  stepper motor driver IC. (see Allegro website for datasheet)

  This library diverges from others that are around, in that it
  assumes that the MS1, MS2, and MS3 pins are connected to gpio
  pins on the Arduino, allowing control over the microstepping
  modes.

  The A4988 is capable of microstepping down to 1/16 of a step,
  enabling fine control over the stepper motor. This fine control
  can be used in, among other things, 3D printers.

  This library provides an interface for setting the different
  step modes, going from full step down to 1/16 step, using a
  simple setter function, where the argument is 1,2,4,8, or 16.


     MS1   MS2   MS3
    -----------------
     low   low   low   Full step
     high  low   low   Half step
     low   high  low   1/4 step
     high  high  low   1/8 step
     high  high  high  1/16 step

  Note:
  Lower delay values can be used in the microstepping mode.
  Values as low as 25 usec can be used in the 1/16 mode
  with some motors.

*/

#ifndef __A4988_H__
#define __A4988_H__

#include <Arduino.h>
#include <inttypes.h>

#define FORWARD     0
#define BACKWARD    1
#define MICROSTEPS  16

class A4988 {
  public:
    // constructors:
    A4988(uint16_t motor_steps, uint8_t dir_pin, uint8_t enable_pin, uint8_t step_pin);

    // mover method:
    void enable(uint8_t enable);
    void setDirection(uint8_t direction);
    void step(uint16_t steps);
    void step(uint16_t steps, uint8_t dir);
    void microStep(void);
    void onestep(uint8_t dir);

  private:
    void stepMotor(uint8_t this_step);

    uint8_t  direction;               // Direction of rotation
    uint16_t motor_steps;             // number of steps motor has per revolution
    uint32_t step_delay;              // delay between steps, in us
    uint8_t  num_steps;               // total number of steps to step
    uint8_t  step_pin;                // pin which controls stepping
    uint8_t  dir_pin;
    uint8_t  enable_pin;
    uint8_t  isEnabled;
};

#endif //a4988_h
