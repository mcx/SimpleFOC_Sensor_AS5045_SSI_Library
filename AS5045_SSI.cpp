#include "AS5045_SSI.h"

/**
 *  AS5045 SSI mode class constructor
 * @param cs  SSI chip select pin
 * @param dout  SSI data output pin
 * @param clk  SSI bit clock pin
 */
AS5045_SSI::AS5045_SSI(int cs, int dout, int clk)
{
    ssi_chip_select_pin = cs;
    ssi_data_out_pin = dout;
    ssi_clock_pin = clk;
    cpr = 4096;
}

void AS5045_SSI::init()
{
    //setup pins
    pinMode(ssi_chip_select_pin, OUTPUT);
    pinMode(ssi_data_out_pin, INPUT);
    pinMode(ssi_clock_pin, OUTPUT);
    digitalWrite(ssi_chip_select_pin, HIGH);
    digitalWrite(ssi_clock_pin, HIGH);
//    delayMicroseconds(2);

	// velocity calculation init
	angle_prev = 0;
	velocity_calc_timestamp = _micros();

	// full rotations tracking number
	full_rotation_offset = 0;
	angle_data_prev = getRawCount();
}

//  Shaft angle calculation
//  angle is in radians [rad]
float AS5045_SSI::getAngle(){
  // raw data from the sensor
  float angle_data = getRawCount();

  // tracking the number of rotations
  // in order to expand angle range form [0,2PI]
  // to basically infinity
  float d_angle = angle_data - angle_data_prev;
  // if overflow happened track it as full rotation
  if(abs(d_angle) > (0.8f*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI;
  // save the current angle value for the next steps
  // in order to know if overflow happened
  angle_data_prev = angle_data;

  // return the full angle
  // (number of full rotations)*2PI + current sensor angle
  return full_rotation_offset + ( angle_data / (float)cpr) * _2PI;
}

// Shaft velocity calculation
float AS5045_SSI::getVelocity(){
  // calculate sample time
  unsigned long now_us = _micros();
  float Ts = (now_us - velocity_calc_timestamp)*1e-6f;
  // quick fix for strange cases (micros overflow)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // current angle
  float angle_c = getAngle();
  // velocity calculation
  float vel = (angle_c - angle_prev)/Ts;

  // save variables for future pass
  angle_prev = angle_c;
  velocity_calc_timestamp = now_us;
  return vel;
}

// function reading the raw counter of the magnetic sensor
int AS5045_SSI::getRawCount()
{
    return (int)(AS5045_SSI::read() >> 6);
}

#pragma GCC push_options
#pragma GCC optimize("O0")
void AS5045_SSI::ssi_delay()
{
#if defined(_STM32_DEF_)
    for (int i = 0; i < 2; i++);
#endif
}
#pragma GCC pop_options

/*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit word
  * Returns the value of the register
  */
word AS5045_SSI::read()
{
    word raw_value = 0;
    uint16_t inputstream = 0;
    uint16_t c;

    digitalWrite(ssi_chip_select_pin, HIGH);
    digitalWrite(ssi_clock_pin, HIGH);

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega2560__)
    AS5045_SSI::ssi_delay();
#endif

    digitalWrite(ssi_chip_select_pin, LOW);

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega2560__)
    AS5045_SSI::ssi_delay();
#endif

    digitalWrite(ssi_clock_pin, LOW);

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega2560__)
    AS5045_SSI::ssi_delay();
#endif

    for (c = 0; c < 18; c++)
    {
        digitalWrite(ssi_clock_pin, HIGH);

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega2560__)
        AS5045_SSI::ssi_delay();
#endif

        inputstream = digitalRead(ssi_data_out_pin);
        raw_value = ((raw_value << 1) + inputstream);

        digitalWrite(ssi_clock_pin, LOW);

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega2560__)
        AS5045_SSI::ssi_delay();
#endif
    }
    
    return raw_value;
}

/**
 * Closes the SSI connection
 */
void AS5045_SSI::close()
{
    pinMode(ssi_chip_select_pin, INPUT);
    pinMode(ssi_clock_pin, INPUT);
    pinMode(ssi_data_out_pin, INPUT);
}
