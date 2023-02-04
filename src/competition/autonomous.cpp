#include "../include/competition/autonomous.h"

/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{

    while(!imu.isCalibrating()){
     vexDelay(20);
    }

    // CommandController current_auto = get_chosen_auto();
    // current_auto.run();


}