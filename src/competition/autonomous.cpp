#include "../include/competition/autonomous.h"
#include "competition/autonomous_clu.h"
#include "../include/robot-config.h"
#include "../core/include/utils/math_util.h"

#define TURN_SPEED 0.6


//functions that define autos. construct a CommandController when called.
CommandController auto_loader_side();
CommandController prog_skills_loader_side();


/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{

    while(imu.isCalibrating()){
      vexDelay(20);
    }

    // CommandController *current_auto = NULL;

    // switch(curr_mode)
    // {
    //   case COMP:
    //   case TEST1:
    //   case TEST2:
    //     *current_auto = auto_non_loader_side();
    //     break;
    //   case AUTOSKILLS:
    //     *current_auto = prog_skills_non_loader_side();
    //     break;

    //   default:
    //     break;
    // }

    // if(current_auto != NULL)
    // {
    //   current_auto->run();
    //   delete current_auto;
    // }
    
    // auto_non_loader_side().run();
    prog_skills_non_loader_side().run();

    flywheel_sys.stop();
    intake.stop();
    drive_sys.stop();

}

