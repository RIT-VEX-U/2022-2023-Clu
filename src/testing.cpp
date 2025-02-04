#include "testing.h"
#include "tuning.h"
#include "vex.h"
#include "robot-config.h"
#include "competition/autonomous_clu.h"
#include "automation.h"
#include "competition/autonomous.h"

void test1_opcontrol()
{
  target_red = true;
  vision_enabled = true;
  // Test1: Match Auto Testing
  // while(imu.isCalibrating()){}

  autonomous();
  // Set up screen stuff
  // vexDelay(5000);
  
  tune_shooting();
  
  programmers_opcontrol();
}

void test2_opcontrol()
{
  programmers_opcontrol();
}

void tuning_opcontrol()
{
  while (imu.isCalibrating())
  {
  }
  while(true)
  {
    // tune_odometry_wheel_diam();
    // tune_odometry_wheelbase();
    // tune_flywheel_ff();
    tune_flywheel_pid();
    // tune_drive_ff_ks(DRIVE);
    // tune_drive_ff_kv(DRIVE, 0.03);
    // tune_drive_motion_maxv(TURN);
    // tune_drive_motion_accel(TURN, 700);
    // tune_drive_pid(DRIVE);

    // if(main_controller.ButtonR1.pressing())
    //   intake.spin(directionType::rev, 12, volt);
    // else
    //   intake.stop();

    // auto pos = odometry_sys.get_position();
    // main_controller.Screen.clearScreen();
    // main_controller.Screen.setCursor(0, 0);
    // main_controller.Screen.print("(%.3f, %.3f) : %.3f", pos.x, pos.y, pos.rot);
    // printf("X: %f, Y: %f, R: %f\n", pos.x, pos.y, pos.rot);
    // motion_t cur_motion = turn_fast_mprofile.get_motion();
    // double s = (cur_motion.pos != 0? 24 + cur_motion.pos : 0);
    // double s = cur_motion.pos + 180; //(cur_motion.pos != 0? 90 + cur_motion.pos : 0) + 90;
    // printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", tmr.time(sec), s, pos.rot, fabs(cur_motion.vel), fabs(odometry_sys.get_angular_speed_deg()), cur_motion.accel, odometry_sys.get_angular_accel_deg());

    // if(main_controller.ButtonX.pressing())
    //   drive_sys.drive_tank(.11, -.11);

    // DEBUG
    // printf("left enc: %f, right enc: %f\n", left_enc.rotation(rotationUnits::rev), right_enc.rotation(rotationUnits::rev));

    vexDelay(20);
  }
}

void programmers_opcontrol()
{
  flywheel_sys.stop();
  intake.stop();

  // flywheel_sys.spinRPM(3800);
  VisionAimCommand visaim(false, 155, 10);
  SpinRollerCommand rol_cmd;
  position_t pos;
  main_controller.Screen.clearScreen();
  roller_sensor.setLightPower(100, pct);
  while (true)
  {
    // Odometry Info
    pos = odometry_sys.get_position();
    static bool hasfinished = false;
    // Vision Aim Tuning
    if(main_controller.ButtonB.pressing())// && hasfinished == false && rol_cmd.run())
    {
      visaim.run();
      // hasfinished = true;
    } else if (main_controller.ButtonB.pressing() == false)
    {
      // hasfinished = false;
      drive_sys.drive_arcade(main_controller.Axis3.position() / 200.0, main_controller.Axis1.position() / 200.0);
    }

    // Debug Out
    roller_sensor.setLightPower(100, pct);
    Pepsi rol = get_roller_scored();
    string rol_str = (rol == RED) ? "R" : (rol == BLUE) ? "B" : "N";
    printf("X: %2f, Y: %2f, R: %2f, Rol: %s|%0f\n",
       pos.x, pos.y, pos.rot, rol_str.c_str(), roller_sensor.hue());    

    // Flap Controls
    static bool flap_is_up = false;
    static bool buttonX_newpress = true;
    if (buttonX_newpress && main_controller.ButtonX.pressing())
    {
      flap_is_up = !flap_is_up;

      if (flap_is_up)
        flap_up();
      else
        flap_down();

      buttonX_newpress = false;
    }
    else if (!main_controller.ButtonX.pressing())
    {
      buttonX_newpress = true;
    }

    // Intake Controls
    static bool intake_btn_pressing = false;
    static double intake_speed = 9.5;

    if (!intake_btn_pressing)
    {
      if (main_controller.ButtonA.pressing())
        intake_speed += .5;
      else if (main_controller.ButtonY.pressing())
        intake_speed -= .5;
    }

    intake_btn_pressing = main_controller.ButtonA.pressing() || main_controller.ButtonY.pressing();

    if (main_controller.ButtonR2.pressing())
      intake.spin(directionType::fwd, intake_speed, volt);
    else if (main_controller.ButtonR1.pressing())
      intake.spin(directionType::rev, 12, volt);
    else
      intake.stop();

    // Flywheel Speed Control
    static bool flywheel_btn_pressing = false;
    static int flywheel_setpt;
    if (!flywheel_btn_pressing){
      if (main_controller.ButtonRight.pressing())
        flywheel_setpt += 50;
      else if (main_controller.ButtonLeft.pressing())
        flywheel_setpt -= 50;
      else if (main_controller.ButtonDown.pressing())
        flywheel_setpt = 0;
      else if (main_controller.ButtonUp.pressing())
        flywheel_setpt = 3600;
    }

    flywheel_btn_pressing = main_controller.ButtonLeft.pressing() || main_controller.ButtonRight.pressing() || main_controller.ButtonUp.pressing() || main_controller.ButtonDown.pressing();

    // Debug Info
    if (flywheel_btn_pressing)
    {
      main_controller.Screen.clearLine(1);
      main_controller.Screen.setCursor(1, 0);
      main_controller.Screen.print("RPM: %d", flywheel_setpt);
      if(flywheel_setpt == 0)
        flywheel_sys.stop();
      else
        flywheel_sys.spinRPM(flywheel_setpt);
      if(flywheel_setpt == 0)
        flywheel_sys.stop();
      else
        flywheel_sys.spinRPM(flywheel_setpt);
    }

    if (intake_btn_pressing)
    {
      main_controller.Screen.clearLine(2);
      main_controller.Screen.setCursor(2, 0);
      main_controller.Screen.print("Intake: %f", intake_speed);
    }

    static timer screen_tmr;
    if (screen_tmr.time(timeUnits::msec) > 500)
    {
      main_controller.Screen.clearLine(3);
      main_controller.Screen.setCursor(3, 0);
      main_controller.Screen.print("Flywheel Temp: %2f", flywheel.temperature(temperatureUnits::fahrenheit));

      screen_tmr.reset();
    }
    vexDelay(20);
  }
}