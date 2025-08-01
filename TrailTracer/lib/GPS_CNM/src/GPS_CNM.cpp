/* GPS_CNM library by Brian Rashap
 */

#include "GPS_CNM.h"

/**
 * Constructor.
 */
GPS_CNM::GPS_CNM()
{
  // be sure not to call anything that requires hardware be initialized here, put those in begin()
}

/**
 * Example method.
 */
void GPS_CNM::begin()
{
    // initialize hardware
    Serial.println("called begin");
}

/**
 * Example method.
 */
void GPS_CNM::process()
{
    // do something useful
    Serial.println("called process");
    doit();
}

/**
* Example private method
*/
void GPS_CNM::doit()
{
    Serial.println("called doit");
}
