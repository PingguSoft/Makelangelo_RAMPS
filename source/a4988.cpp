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

#include "Arduino.h"
#include "A4988.h"

// constructor to set up pins and initialize data
A4988::A4988(uint16_t motor_steps, uint8_t dir_pin, uint8_t enable_pin, uint8_t step_pin)
{
   this->enable_pin = enable_pin;
   this->dir_pin    = dir_pin;
   this->step_pin   = step_pin;

   // setup the pins on the microcontroller:
   pinMode(this->dir_pin, OUTPUT);
   pinMode(this->enable_pin, OUTPUT);
   pinMode(this->step_pin, OUTPUT);

   if(motor_steps != 0) {
      this->motor_steps = motor_steps;
   } else {
      this->motor_steps = 200;
   }

   this->step_delay = 200;
   this->isEnabled  = 1;
   digitalWrite(this->enable_pin, !this->isEnabled);
}

void A4988::enable(uint8_t enable)
{
   this->isEnabled = enable;
   digitalWrite(this->enable_pin, !this->isEnabled);
}

void A4988::setDirection(uint8_t direction)
{
   if(direction == 0) {
      digitalWrite(dir_pin, HIGH);
   } else {
      digitalWrite(dir_pin, LOW);
   }
}

void A4988::microStep(void)
{
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(this->step_delay);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(this->step_delay);
}

void A4988::step(uint16_t steps)
{
   uint32_t x;

   for (x = 0; x < steps * MICROSTEPS; x++) {
      this->microStep();
   }
}

void A4988::step(uint16_t steps, uint8_t dir)
{
   setDirection(dir);
   step(steps);
}

void A4988::onestep(uint8_t dir)
{
    step(1, dir);
}