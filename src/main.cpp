/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\ryan                                             */
/*    Created:      Wed Aug 31 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "robot-config.h"
#include "competition/opcontrol.h"
#include "competition/autonomous.h"

using namespace vex;

competition comp;

/**
 * The main method of the robot.
 * Do NOT use for general programming or initialization.
 * Instead use competition/opcontrol.cpp, competition/autonomous.cpp, or robot-config.cpp for programming or configuration.
 */
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  StartScreen(Brain.Screen, {page_one, page_two, page_three, page_four, page_five});
  comp.autonomous(autonomous);
  comp.drivercontrol(opcontrol);
  vexcodeInit();
}
