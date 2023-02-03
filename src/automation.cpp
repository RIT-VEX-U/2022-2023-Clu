#include "../include/automation.h"
#include "robot-config.h"


/**
* Construct a SpinRollerCommand
* @param drive_sys the drive train that will allow us to apply pressure on the rollers
* @param roller_motor The motor that will spin the roller
*/
SpinRollerCommandAUTO::SpinRollerCommandAUTO(TankDrive &drive_sys, vex::motor &roller_motor): roller_motor(roller_motor), drive_sys(drive_sys){};

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinRollerCommandAUTO::run() {
    const double roller_cutoff_threshold = .05; //revolutions // [measure]
    const double num_revolutions_to_spin_motor = -2; //revolutions // [measure]
    const double drive_power = .2; // [measure]

    // Initialize start and end position if not already
    if (!func_initialized){
        start_pos = roller_motor.position(vex::rev);
        target_pos = start_pos + num_revolutions_to_spin_motor;
        func_initialized = true;
    }    

    // Calculate error
    double current_pos = roller_motor.position(vex::rev);
    double error = target_pos-current_pos;

    // If we're close enough, call it here.
    if (fabs(error)>roller_cutoff_threshold < roller_cutoff_threshold){
        func_initialized = false;
        roller_motor.stop();
        return true;
    }

    vex::directionType dir = fwd;
    if (error<0){
      dir = reverse;
    }
    // otherwise, do a P controller
    roller_motor.spin(dir, 8, vex::volt);
    drive_sys.drive_tank(drive_power, drive_power);
    return false;
}

/**
* Construct a ShootCommand
* @param firing_motor The motor that will spin the disk into the flywheel
*/
ShootCommand::ShootCommand(vex::motor &firing_motor, double seconds_to_shoot, double volt): firing_motor(firing_motor), seconds_to_shoot(seconds_to_shoot), volt(volt){}

/**
 * Run the intake motor backward to move the disk into the flywheel
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool ShootCommand::run(){
  if (!func_initialized){
    tmr.reset();
    func_initialized = true;
  }

  if (tmr.time(vex::seconds) > seconds_to_shoot){
    func_initialized = false;
    firing_motor.stop();
    return true;
  }
  printf("Shooting at %f RPM\n", flywheel_sys.getRPM());
  firing_motor.spin(vex::fwd, volt, vex::volt); //TODO figure out if this needs to be negated to slap it into the flywheel
  return false;
}


/**
* Construct a StartIntakeCommand
* @param intaking_motor The motor that will pull the disk into the robot
* @param intaking_voltage The voltage at which to run the intake motor
*/
StartIntakeCommand::StartIntakeCommand(vex::motor &intaking_motor, double intaking_voltage):intaking_motor(intaking_motor), intaking_voltage(intaking_voltage){}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 
*/
bool StartIntakeCommand::run(){
  intaking_motor.spin(vex::reverse, intaking_voltage, vex::volt); 
  return true;
}

SpinRawCommand::SpinRawCommand(vex::motor &flywheel_motor, double voltage):flywheel_motor(flywheel_motor), voltage(voltage){}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 
*/
bool SpinRawCommand::run(){
  flywheel_motor.spin(vex::fwd, voltage, vex::volt); 
  return true;
}

/**
* Construct a StartIntakeCommand
* @param intaking_motor The motor that will be stopped
*/
StopIntakeCommand::StopIntakeCommand(vex::motor &intaking_motor):intaking_motor(intaking_motor){}

/**
 * Run the StopIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool StopIntakeCommand::run(){
  intaking_motor.stop(); 
  return true;
}

EndgameCommand::EndgameCommand(vex::digital_out &solenoid): solenoid(solenoid){}
bool EndgameCommand::run(){
  solenoid.set(true);
  return true;
}

PID::pid_config_t vis_pid_cfg = {
  .p = 0,
  .d = 0,
  .deadband = 0,
  .on_target_time = 0
};

#define VISION_CENTER 0
#define NOT_DETECTED_TIME 2
#define MAX_SPEED 0.5

VisionAimCommand::VisionAimCommand(vision &cam, vision::signature &sig, TankDrive &drive_sys): cam(cam), sig(sig), drive_sys(drive_sys), pid(vis_pid_cfg){}

/**
 * Run the VisionAimCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
*/
bool VisionAimCommand::run()
{
  // If the camera isn't installed, move on to the next command
  if(!cam.installed())
    return true;
  
  cam.takeSnapshot(sig, 1);
  if(cam.objectCount > 0)
  {
    pid.set_target(VISION_CENTER);
    pid.set_limits(-MAX_SPEED, MAX_SPEED);

    vision::object obj = cam.largestObject;
    double out = pid.update(obj.centerX);
    drive_sys.drive_tank(out, -out);

    if(pid.is_on_target())
      return true;

  } else
  {
    drive_sys.stop();
  }

  return false;
}