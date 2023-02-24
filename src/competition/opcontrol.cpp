#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"
#include "testing.h"
#include "competition/autonomous_clu.h"

void print_to_screen()
{
  static timer tmr;

  if(tmr.time() > 1)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0, 0);
    Brain.Screen.print("Battery: %d%", Brain.Battery.capacity());
    Brain.Screen.print("Flywheel Temp: %2f F", flywheel.temperature());
    Brain.Screen.setCursor(1, 0);
    if(flywheel.installed())
      Brain.Screen.print("Flywheel Connected");
    else
      Brain.Screen.print("WARNING || FLYWHEEL DISCONNECTED!");
    Brain.Screen.print("Flywheel RPM: %2f", flywheel_sys.getRPM());
    tmr.reset();
  }
}

#define SHOTLENGTH 100
#define DELAYLENGTH 300
void tripleshot()
{
  for(int i = 0; i<3; i++)
  {
    intake.spin(fwd, 12, volt);
    vexDelay(SHOTLENGTH);
    intake.stop();
    vexDelay(DELAYLENGTH);
  }  
}

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  test1_opcontrol();
  // select_mode();
  // printf("angle: %d mode: %d\n",mode_switch.value(pct), curr_mode);
  // return;
  // Select Mode
  // switch(curr_mode)
  // {
  //   case TEST1:
  //     test1_opcontrol();
  //     return;
  //   case TEST2:
  //     test2_opcontrol();
  //     return;

  //   default:
  //     // Competition, or other defaults just continue
  //     break;
  // }

  // Initialization
  printf("starting\n");
  fflush(stdout);

  // Intake - R1
  main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); 
  main_controller.ButtonR1.released([](){intake.stop();});

  // Shoot - R2
  main_controller.ButtonR2.pressed([](){intake.spin(fwd, 9.5, volt);}); 
  main_controller.ButtonR2.released([](){intake.stop();});

  // Flap - Y
  main_controller.ButtonY.pressed([](){
    static bool flapUp = false;
    flapUp = !flapUp;
    flapup_solenoid.set(flapUp);
  });

  // Single Shot - L2
  main_controller.ButtonL2.pressed([](){
    intake.spin(fwd, 12, volt);
    vexDelay(SHOTLENGTH);
    intake.stop();
    vexDelay(DELAYLENGTH);
    });
 
  // Flywheel set RPM 
  flywheel_sys.spinRPM(4000);
  odometry_sys.end_async();
  
  flap_up();
  timer tmr;

  // int i = 0;
  
  VisionAimCommand visaim;

  // Periodic
  while(true)
  {
    // print_to_screen();
    // i++;
    // if (i % 5 == 0)
    // {
    //   main_controller.Screen.setCursor(0, 0);
    //   main_controller.Screen.clearScreen();
    //   main_controller.Screen.print("fw rpm: %f", flywheel_sys.getRPM());
    //   main_controller.Screen.setCursor(2, 0);
    //   main_controller.Screen.print("fw temp: %.1ff", flywheel.temperature(vex::fahrenheit));
    //   main_controller.Screen.setCursor(4, 0);
    //   main_controller.Screen.print("bat fw : %.2fv %.2fv", Brain.Battery.voltage(vex::volt), flywheel.voltage(volt));
    // }
    // ========== DRIVING CONTROLS ==========
    if(!main_controller.ButtonX.pressing())
      drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    else
      visaim.run();
    // drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);
    
    // ========== MANIPULATING CONTROLS ==========
    if(main_controller.ButtonL1.pressing() && main_controller.ButtonL2.pressing()
      && main_controller.ButtonR1.pressing() && main_controller.ButtonR2.pressing())
    {
      endgame_solenoid.set(true);
    }

    if(main_controller.ButtonDown.pressing())
      flywheel_sys.stop();
    else if(main_controller.ButtonUp.pressing())
      flywheel_sys.spinRPM(4000);

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }
  
}