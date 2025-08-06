#pragma once

/* GPS_CNM library by Brian Rashap
 */

// This will load the definition for common Particle variable types
#include "Particle.h"

// This is your main class that users will import into their application
class GPS_CNM
{
public:
  /**
   * Constructor
   */
  GPS_CNM();

  /**
   * Example method
   */
  void begin();

  /**
   * Example method
   */
  void process();

private:
  /**
   * Example private method
   */
  void doit();
};
