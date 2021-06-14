#ifndef _AS5045_SSI_H_
#define _AS5045_SSI_H_

#include <Arduino.h>
#include <SimpleFOC.h>

class AS5045_SSI : public Sensor
{
public:
    /**
     *  AS5045 SSI mode class constructor
     * @param cs  SSI chip select pin
     * @param dout  SSI data output pin
     * @param clk  SSI bit clock pin
     */
    AS5045_SSI(int cs, int dout, int clk);

    /** sensor initialise */
    void init();

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;

private:
    float cpr; //!< Maximum range of the magnetic sensor
    // SSI variables
    int ssi_data_out_pin;    //!< SSI data output pin
    int ssi_chip_select_pin; //!< SSI chip select pin
    int ssi_clock_pin;       //!< SSI bit clock pin
    // SSI functions
    void ssi_delay();
    /** Stop SSI communication */
    void close();
    /** Read SSI value */
    word read();

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset; //!<number of full rotations made
    float angle_data_prev;      //!< angle in previous position calculation step

    // velocity calculation variables
    float angle_prev;             //!< angle in previous velocity calculation step
    long velocity_calc_timestamp; //!< last velocity calculation timestamp
};

#endif
