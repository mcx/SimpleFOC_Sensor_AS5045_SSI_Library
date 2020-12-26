#include <SimpleFOC.h>
#include <SimpleFOC_AS5045_SSI.h>

AS5045_SSI sensor = AS5045_SSI(PA4, PA6, PA5);

void setup()
{
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop()
{
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}