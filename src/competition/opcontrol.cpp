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
  // tuning_opcontrol();

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
#define INTAKE_FLAPDOWN 8.5
#define FLYWHEEL_NORMAL 3200
#define FLYWHEEL_OVERFILL 2650
#define FLYWHEEL_FLAPDOWN 3250

  static std::atomic<double> cur_intake_volt(INTAKE_NORMAL);
  static std::atomic<int> cur_flywheel_rpm(FLYWHEEL_NORMAL);

  // Intake - R1
  main_controller.ButtonR1.pressed([](){ intake.spin(reverse, 12, volt); });
  main_controller.ButtonR1.released([](){ intake.stop(); });

  // Shoot - R2
  main_controller.ButtonR2.pressed([](){ intake.spin(fwd, cur_intake_volt, volt); });
  main_controller.ButtonR2.released([](){ intake.stop(); });

  // Flap - Y
  main_controller.ButtonY.pressed([]()
  {
    static bool flapDown = false;
    static double saved_intake = INTAKE_NORMAL;
    static int saved_flywheel = FLYWHEEL_NORMAL;
    flapDown = !flapDown;
    flapup_solenoid.set(flapDown); 

    if(!flapDown)
    {
      // FLAP UP - restore last intake / flywheel
      cur_intake_volt = saved_intake;
      cur_flywheel_rpm = saved_flywheel;
    } else
    {
      // FLAP DOWN - save the currrent and switch to flapdown mode
      saved_intake = cur_intake_volt;
      saved_flywheel = cur_flywheel_rpm;

      cur_intake_volt = INTAKE_FLAPDOWN;
      cur_flywheel_rpm = FLYWHEEL_FLAPDOWN;
    }
    
    flywheel_sys.spinRPM(cur_flywheel_rpm);
  });

  // Single Shot - L2
  main_controller.ButtonL2.pressed([]()
  {
    intake.spin(fwd, cur_intake_volt, volt);
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

  VisionAimCommand visaim(false, 155, 5);
  int i = 0;
  double time = 0.0;
  // Periodic
  while (true)
  {
    // ========== DRIVING CONTROLS ==========
    if (!main_controller.ButtonX.pressing())
    {
      // if (abs(main_controller.Axis3.position()) > 5 || abs(main_controller.Axis2.position()) > 5)
      // {
        drive_sys.drive_tank(main_controller.Axis3.position() / 100.0, main_controller.Axis2.position() / 100.0);
      // }
      // else
      // {
      //   static PID::pid_config_t brake_cfg {.p = .001};
      //   static PID l_brake_pid(brake_cfg), r_brake_pid(brake_cfg);
        
      //   l_brake_pid.set_target(0);
      //   r_brake_pid.set_target(0);
      //   l_brake_pid.update(left_motors.velocity(velocityUnits::rpm));
      //   r_brake_pid.update(right_motors.velocity(velocityUnits::rpm));
        
      //   drive_sys.drive_tank(l_brake_pid.get(), r_brake_pid.get());

      // }
    } 
    else
    {
      visaim.run();
    }
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