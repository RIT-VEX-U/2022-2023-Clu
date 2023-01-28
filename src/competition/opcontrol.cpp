#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"
#include "automation.h"

void tuning()
{
  // tune_odometry_wheel_diam();
  // tune_odometry_wheelbase();
  // tune_flywheel_ff();
  // tune_drive_ff_ks(TURN);
  // tune_drive_ff_kv(TURN, 0.12);
  // tune_drive_motion_maxv(TURN);
  // tune_drive_motion_accel(TURN, 700);
  // tune_drive_pid(TURN);
  
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
int print_odom(){
  while(true){
    auto pos = odometry_sys.get_position();
    printf("(%.6f, %.6f) : %.6f\n", pos.x, pos.y, pos.rot);
    printf("(RPM: %.6f\n", flywheel_sys.getRPM());
    
    vexDelay(20);
  }
  return 0;
}
CommandController auto_non_loader_side_test(){
    int non_loader_side_shot_rpm = 3200;  // [measure]
    CommandController nlsa;
    position_t start_pos = {.x = 128, .y = 89, .rot = 90}; // [measure]
    nlsa.add(new OdomSetPosition(odometry_sys, start_pos));

   nlsa.add(new SpinRawCommand(flywheel, 12));
   // Arrow 1 -------------------
   nlsa.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 127.9, 114, fwd, 1)); // [measure]
   nlsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 0, .6)); // [measure]
    
    
    // Arrow 2 -------------------
    nlsa.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 4, fwd, 1)); // [measure]
    nlsa.add(new SpinRollerCommandAUTO(drive_sys, roller));
    nlsa.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 12, reverse, 1)); // [measure]

    // Spin and shoot
    nlsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 225, .6)); //[measure]
    nlsa.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 89, 76.5, directionType::fwd, 1)); //[ measure]
    nlsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 145, 0.6)); // [measure]

    nlsa.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    nlsa.add(new ShootCommand(intake, 3, .25)); // [measure]
    
    return nlsa;
}

void auto_test(){
  while(imu.isCalibrating()){
    vexDelay(20);
  }
  vex::task odom_p(print_odom);
  CommandController tauto = auto_non_loader_side_test();
  tauto.run();
}
/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  auto_test();
  return;
  
  // Initialization
  printf("starting\n");
  fflush(stdout);
  double oneshot_time = .05;//Change 1 second to whatever is needed
  bool oneshotting = false;
  
  //flywheel_sys.spin_manual(3000);//TODO measure speed that is needed
  main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); // Intake
  main_controller.ButtonR1.released([](){intake.stop();});
  main_controller.ButtonR2.pressed([](){intake.spin(fwd, 12, volt);}); // Shoot
  main_controller.ButtonR2.released([](){intake.stop();}); // Shoot
  
  main_controller.ButtonL2.pressed([](){
    intake.spin(fwd, 12, volt);
    vexDelay(SHOTLENGTH);
    intake.stop();
    vexDelay(DELAYLENGTH);
    }); //Single Shoot
  main_controller.ButtonL1.pressed([](){roller.spin(reverse, 12, volt);}); //Roller
  main_controller.ButtonL1.released([](){roller.stop();});
  main_controller.ButtonUp.pressed([](){odometry_sys.set_position();});
  //flywheel_sys.spinRPM(4000);
  flywheel_sys.stop();
  //flywheel.spin(fwd, 11.5, volt);
  timer tmr;
  // Periodic
  while(true)
  {
    double motor_temp = flywheel.temperature(fahrenheit);
    double bat_volt = Brain.Battery.voltage();
    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.print("mot tmp: %.1f", motor_temp);
    main_controller.Screen.setCursor(2, 2);
    main_controller.Screen.print("bat vol: %.1f", bat_volt);
    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    // drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);
    
    // ========== MANIPULATING CONTROLS ==========


    if(main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
    {
      endgame_solenoid.set(true);
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }

}