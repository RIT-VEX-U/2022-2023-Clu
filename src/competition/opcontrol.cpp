#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"
#include "testing.h"
#include "competition/autonomous_clu.h"
#define TESTING_BUILD

void print_to_screen()
{
  static timer tmr;

  if (tmr.time() > 1)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0, 0);
    Brain.Screen.print("Battery: %d%", Brain.Battery.capacity());
    Brain.Screen.print("Flywheel Temp: %2f F", flywheel.temperature());
    Brain.Screen.setCursor(1, 0);
    if (flywheel.installed())
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
  for (int i = 0; i < 3; i++)
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
  // test1_opcontrol();
  // programmers_opcontrol();
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

#define INTAKE_NORMAL 9.5
#define INTAKE_OVERFILL 9.5
#define FLYWHEEL_NORMAL 4000
#define FLYWHEEL_OVERFILL 3000

  static std::atomic<double> cur_intake_volt(INTAKE_NORMAL);
  static std::atomic<double> cur_flywheel_rpm(FLYWHEEL_NORMAL);

  // Intake - R1
  main_controller.ButtonR1.pressed([](){ intake.spin(reverse, 12, volt); });
  main_controller.ButtonR1.released([](){ intake.stop(); });

  // Shoot - R2
  main_controller.ButtonR2.pressed([](){ intake.spin(fwd, cur_intake_volt, volt); });
  main_controller.ButtonR2.released([](){ intake.stop(); });

  // Flap - Y
  main_controller.ButtonY.pressed([]()
  {
    static bool flapUp = false;
    flapUp = !flapUp;
    flapup_solenoid.set(flapUp); 
  });

  // Single Shot - L2
  main_controller.ButtonL2.pressed([]()
  {
    intake.spin(fwd, 9.5, volt);
    vexDelay(SHOTLENGTH);
    intake.stop();
    vexDelay(DELAYLENGTH); 
  });

  main_controller.ButtonLeft.pressed([]()
  {
    cur_intake_volt = INTAKE_OVERFILL;
    cur_flywheel_rpm = FLYWHEEL_OVERFILL;

    flywheel_sys.spinRPM(cur_flywheel_rpm); 
  });

  main_controller.ButtonRight.pressed([]()
  {
    cur_intake_volt = INTAKE_NORMAL;
    cur_flywheel_rpm = FLYWHEEL_NORMAL;

    flywheel_sys.spinRPM(cur_flywheel_rpm); 
  });

  // Flywheel set RPM
  flywheel_sys.spinRPM(cur_flywheel_rpm);
  odometry_sys.end_async();

  flap_up();
  timer tmr;

  VisionAimCommand visaim;
  int i = 0;
  double time = 0.0;
  // Periodic
  while (true)
  {

    // ========== DRIVING CONTROLS ==========
    if (!main_controller.ButtonX.pressing())
      drive_sys.drive_tank(main_controller.Axis3.position() / 100.0, main_controller.Axis2.position() / 100.0);
    else
      visaim.run();
    // drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);

    // ========== MANIPULATING CONTROLS ==========
    if (main_controller.ButtonL1.pressing() && main_controller.ButtonL2.pressing() && main_controller.ButtonR1.pressing() && main_controller.ButtonR2.pressing())
    {
      endgame_solenoid.set(true);
    }

    if (main_controller.ButtonDown.pressing())
    {
      flywheel_sys.stop();
    }
    else if (main_controller.ButtonUp.pressing())
    {
      flywheel_sys.spinRPM(cur_flywheel_rpm);
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========

    vexDelay(20);
  }
}