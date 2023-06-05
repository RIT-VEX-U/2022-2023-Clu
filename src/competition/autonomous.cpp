#include "../include/competition/autonomous.h"
#include "competition/autonomous_clu.h"
#include "../include/robot-config.h"
#include "../core/include/utils/math_util.h"

#define TURN_SPEED 0.6


//functions that define autos. construct a CommandController when called.
CommandController auto_loader_side();
CommandController prog_skills_loader_side();

#define MATCH

/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{

    while(imu.isCalibrating()){
      vexDelay(20);
    }

    #ifdef MATCH

    CommandController cmd1 = clu_auto_current_pt1();
    cmd1.run();


    if(imu.installed() == true)
    {
      // CommandController cmd2 = clu_auto_current_pt2();
      // cmd2.run();
    }

    #else
    CommandController cmd = clu_skills_current();
    cmd.run();
    #endif

    // flywheel_sys.stop();
    intake.stop();
    drive_sys.stop();

}

